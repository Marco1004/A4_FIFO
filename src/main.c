/** \file main.c
* \brief Sistema de tempo real utilizando threads/tasks sincronizadas por FIFOs.
*
*  Este código permite a leitura do valor em A0, utilizando a ADC.
*  Faz uma filtragem (Média total, remove outliars, média).  
*  Utiliza o valor final para ajustar um sinal PWM aplicado ao LED1.
*  A sincronização é feita através de bloqueio á espera de valor no FIFO.
*
* \author Daniel Barra de Almeida        85111
* \author Marco  Antonio da Silva Santos 83192
* \date 30/05/2022
*/

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <drivers/pwm.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>
#include <stdio.h>


/*ADC definitions and includes*/
#include <hal/nrf_saadc.h>

#define ADC_NID DT_NODELABEL(adc) 
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_CHANNEL_ID 1  

#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1 

/* Other defines */
#define TIMER_INTERVAL_MSEC 1000 /* Interval between ADC samples */
#define BUFFER_SIZE 1
#define VECTOR_SIZE 10            /* Filter Local-Vector Size */

/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024

/* Thread scheduling priority */
#define thread_In_prio 1
#define thread_Filter_prio 1
#define thread_Out_prio 1

/* Thread periodicity (in ms)*/
#define SAMP_PERIOD_MS 1000           //---------------------SAMPLING PERIOD----------------------

/* Refer to dts file */
#define PWM0_NID DT_NODELABEL(pwm0) 
#define GPIO0_NID DT_NODELABEL(gpio0) 
#define BOARDLED_PIN 0xD /* Pin at which LED is connected. Addressing is direct (i.e., pin number) */

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

/* Thread code prototypes */
void Input(void *argA , void *argB, void *argC);
void Filter(void *argA , void *argB, void *argC);
void Output(void *argA , void *argB, void *argC);

/* Create fifos*/
struct k_fifo fifo_1;
struct k_fifo fifo_2;

/* Create fifo data structure and variables */
struct data_item_t {
    void *fifo_reserved;    /* 1st word reserved for use by FIFO */
    uint16_t data;          /* Actual data */
};

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

/* PWM configuration */
const struct device *gpio0_dev;         /* Pointer to GPIO device structure */
const struct device *pwm0_dev;          /* Pointer to PWM device structure */
unsigned int pwmPeriod_us = 1000;       /* PWM priod in us */
unsigned int dcValue = 33;              /* Duty-cycle in % */

/** \brief  Amostragem da ADC.
*  Faz a leitura de uma amostra pela ADC.
*
* Esta função faz a leitura de uma amostra do sinal analógico
* presente na porta A0 da placa.(Valor ente 0 e 1023 V)
*/
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

/** \brief  Configuração.
*  Configura a interface da placa.
*
* Esta função faz as configurações necessárias
* para a interface da placa ser utilizada adequadamente.
*/
void config(void)
{

    int err=0;                              /* Error value variable */

    /* Bind to GPIO 0 */
    gpio0_dev = device_get_binding(DT_LABEL(GPIO0_NID));
    if (gpio0_dev == NULL) {
        printk("Failed to bind to GPIO0\n\r");        
        return;
    }
    /* Bind PWM0 */
    pwm0_dev = device_get_binding(DT_LABEL(PWM0_NID));
    if (pwm0_dev == NULL) {
	printk("Error: Failed to bind to PWM0\n\r");
	return;
    }
    else  {
        printk("Bind to PWM0 successful\n\r");            
    }

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

/** \brief  Função principal.
*  Funcionando com FIFOs
*
* Esta é a funcao principal que logo de incio chama a funcção de configuração.
* São inicializados os FIFOs e criadas as Threads/Tasks para o funcionamento
* 
*/
void main(void)
{
    
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

/** \brief  Thread Input.
*  Funcionamento de aquisição de informação.
*
* Leitura da ADC.
* Conversão da escala de 0 a 1023 para 0 a 3000 (correspondente de 0 a 3 V).
* Inserir o valor lido no FIFO1.
* Esperar pelo fim do periodo de amostragem.
* Repete.
* 
*/
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
        printk("\x1b[2J");  /* Clear screen */
        printk("\x1b[H");   // Send cursor to home

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
        //printk("Thread A data in fifo_ab: %d\n",data_1.data);  
        
        

        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += SAMP_PERIOD_MS;

        }
    }

}

/** \brief  Thread Filter.
*  Funcionamento de filtragem da informação.
*
* Esperar que haja um valor disponivel no FIFO1.
* Retirar o valor presente no FIFO1 e adicioná-lo a um vetor local, retirando o mais antigo.
* Filtragem (Média total, remove outliars, média dos restantes).
* Inserir resultado num segundo FIFO2.
* Repete.
* 
*/
void Filter(void *argA , void *argB, void *argC)
{
    /* Local variables */
    struct data_item_t *data_1;
    struct data_item_t data_2;
    static int local_vect[VECTOR_SIZE] = {0,0,0,0,0};
    static int avg_total;
    static int avg_rem;
    int sum;
    int remaining_samples;

    //printk("Thread B init (sporadic, waits on a semaphore by task A)\n");
    while(1) {
        data_1 = k_fifo_get(&fifo_1, K_FOREVER);

        for (int i = 0; i<VECTOR_SIZE-1; i++){ //Local_Vector[] <- Shared memory 1
          local_vect[i] = local_vect[i+1];

        }
        local_vect[VECTOR_SIZE-1] = data_1->data;

        sum = 0;
        avg_total = 0;
        for (int i = 0; i<VECTOR_SIZE; i++){ // Media com todas as amostras
          sum = sum + local_vect[i];

        }
        avg_total = sum/VECTOR_SIZE;
        printk("AVG_Total %d\n",avg_total);

        sum = 0;
        remaining_samples = 0;
        for (int i = 0; i<VECTOR_SIZE; i++){ // Media sem outliars
          if (local_vect[i] < 1.1*avg_total && local_vect[i] > 0.9*avg_total){
            sum = sum + local_vect[i];

            remaining_samples++;
          }
        }
        printk("Remaining Samples Consideradas -> %d\n",remaining_samples);

        if (remaining_samples>0){
          avg_rem = sum/remaining_samples;
        }
        else{
          printk("ALL OUTLIAR\n");
        }
        
        data_2.data=avg_rem;

        k_fifo_put(&fifo_2, &data_2);
        //printk("Thread B set fifo bc value to: %d \n",data_2.data);     
  }
}

/** \brief  Thread Output.
*  Funcionamento de saída da informação.
*
* Esperar que haja um valor disponivel no FIFO2.
* Ler e retirar o valor presente no FIFO2.
* Mostrar no terminal o valor lido.
* Ajustar o duty-cycle do PWM aplicado ao LED1.
* Repete.
* 
*/
void Output(void *argA , void *argB, void *argC)
{
    /* Local variables */
    int16_t out;
    struct data_item_t *data_2;
    int ret=0;

    //printk("Thread C init (sporadic, waits on a semaphore by task A)\n");
    while(1) {
        data_2 = k_fifo_get(&fifo_2, K_FOREVER);
        out= data_2->data;         
        //printk("Task C read bc value: %d\n",data_2->data);
        dcValue = (100*out)/3000;
        printk("Output: %d\n",out);
        printk("PWM DC value set to %u %%\n",dcValue);
        ret = pwm_pin_set_usec(pwm0_dev, BOARDLED_PIN, pwmPeriod_us,(unsigned int)((pwmPeriod_us*dcValue)/100), PWM_POLARITY_NORMAL);
        if (ret) {
          printk("Error %d: failed to set pulse width\n", ret);
        }
            
  }
}

