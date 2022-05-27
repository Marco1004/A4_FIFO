/*
 * Paulo Pedreiras, 2022/02
 * Zephyr: Simple thread creation example (3)
 * 
 * One of the tasks is periodc, the other two synchronzie via a fifo 
 * 
 * Base documentation:
 *      https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/reference/kernel/index.html
 * 
 */


#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>


/*ADC definitions and includes*/
#include <hal/nrf_saadc.h>
#define ADC_NID DT_NODELABEL(adc) 
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_CHANNEL_ID 1  

#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1 

#define BUFFER_SIZE 1

/* Other defines */
#define TIMER_INTERVAL_MSEC 1000 /* Interval between ADC samples */

/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024

/* Thread scheduling priority */
#define thread_In_prio 1
#define thread_Filter_prio 1
#define thread_Out_prio 1

/* Thread periodicity (in ms)*/
#define SAMP_PERIOD_MS 1000

/* Create thread stack space */
K_THREAD_STACK_DEFINE(thread_In_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_Filter_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_Out_stack, STACK_SIZE);
  
/* Create variables for thread data */
struct k_thread thread_In_data;
struct k_thread thread_Filter_data;
struct k_thread thread_Out_data;

/* Create task IDs */
k_tid_t thread_In_tid;
k_tid_t thread_Filter_tid;
k_tid_t thread_Out_tid;

/* Create fifos*/
struct k_fifo fifo_1;
struct k_fifo fifo_2;

/* Create fifo data structure and variables */
struct data_item_t {
    void *fifo_reserved;    /* 1st word reserved for use by FIFO */
    uint16_t data;          /* Actual data */
};

/* Thread code prototypes */
void Input(void *argA , void *argB, void *argC);
void Filter(void *argA , void *argB, void *argC);
void Output(void *argA , void *argB, void *argC);

/* ADC channel configuration */
static const struct adc_channel_cfg my_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL_ID,
	.input_positive = ADC_CHANNEL_INPUT
};

struct k_timer my_timer;
const struct device *adc_dev = NULL;
static uint16_t adc_sample_buffer[BUFFER_SIZE];

static int adc_sample(void)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution = ADC_RESOLUTION,
	};

	if (adc_dev == NULL) {
            printk("adc_sample(): error, must bind to adc first \n\r");
            return -1;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret) {
            printk("adc_read() failed with code %d\n", ret);
	}	

	return ret;
}

void config(void){
    int err=0;

    /* ADC setup: bind and initialize */
    adc_dev = device_get_binding(DT_LABEL(ADC_NID));
	if (!adc_dev) {
        printk("ADC device_get_binding() failed\n");
    } 
    err = adc_channel_setup(adc_dev, &my_channel_cfg);
    if (err) {
        printk("adc_channel_setup() failed with error code %d\n", err);
    }
    
    /* It is recommended to calibrate the SAADC at least once before use, and whenever the ambient temperature has changed by more than 10 ?C */
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
   
}

/* Main function */
void main(void) {

    config();
    
    /* Create/Init fifos */
    k_fifo_init(&fifo_1);
    k_fifo_init(&fifo_2);
        
    /* Create tasks */
    thread_In_tid = k_thread_create(&thread_In_data, thread_In_stack,
        K_THREAD_STACK_SIZEOF(thread_In_stack), Input,
        NULL, NULL, NULL, thread_In_prio, 0, K_NO_WAIT);

    thread_Filter_tid = k_thread_create(&thread_Filter_data, thread_Filter_stack,
        K_THREAD_STACK_SIZEOF(thread_Filter_stack), Filter,
        NULL, NULL, NULL, thread_Filter_prio, 0, K_NO_WAIT);

    thread_Out_tid = k_thread_create(&thread_Out_data, thread_Out_stack,
        K_THREAD_STACK_SIZEOF(thread_Out_stack), Output,
        NULL, NULL, NULL, thread_Out_prio, 0, K_NO_WAIT);

    
    return;

} 

/* Thread code implementation */
void Input(void *argA , void *argB, void *argC)
{
    /* Timing variables to control task periodicity */
    int64_t fin_time=0, release_time=0;

    /* Other variables */
    int16_t input;
    struct data_item_t data_1;
    int err=0;
    
    printk("Thread A init (periodic)\n");

    /* Compute next release instant */
    release_time = k_uptime_get() + SAMP_PERIOD_MS;
    
    /* Thread loop */
    while(1) {
        
        /* Do the workload */
        err=adc_sample();
        if(err) {
            printk("adc_sample() failed with error code %d\n\r",err);
        }
        else {
            if(adc_sample_buffer[0] > 1023) {
                printk("adc reading out of range\n\r");
            }
            else {
                input= (uint16_t)(1000*adc_sample_buffer[0]*((float)3/1023));
            }
        }
        data_1.data = input;
        k_fifo_put(&fifo_1, &data_1);
        printk("Thread A data in fifo_ab: %d\n",data_1.data);  
       
        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += SAMP_PERIOD_MS;

        }
    }

}

void Filter(void *argA , void *argB, void *argC)
{
    /* Local variables */
    int16_t filter_out;
    struct data_item_t *data_1;
    struct data_item_t data_2;

    printk("Thread B init (sporadic, waits on a semaphore by task A)\n");
    while(1) {
        data_1 = k_fifo_get(&fifo_1, K_FOREVER);
        filter_out= data_1->data +100;
        data_2.data = filter_out;
        k_fifo_put(&fifo_2, &data_2);
        printk("Thread B set fifo bc value to: %d \n",data_2.data);     
  }
}

void Output(void *argA , void *argB, void *argC)
{
    /* Local variables */
    int16_t out;
    struct data_item_t *data_2;

    printk("Thread C init (sporadic, waits on a semaphore by task A)\n");
    while(1) {
        data_2 = k_fifo_get(&fifo_2, K_FOREVER);
        out= data_2->data;         
        printk("Task C read bc value: %d\n",data_2->data);
            
  }
}

