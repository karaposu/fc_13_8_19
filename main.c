/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "core_cm4.h"
#include <stdint.h>


#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

char device_active=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

void usart_init(void);
void usart_rx_check(void);
void usart_process_data(const void* data, size_t len);
void usart_send_string(const char* str);

static uint8_t usart_rx_dma_buffer[64];
uint8_t king_buffer[64];


uint8_t volatile emergency_shut_down=0;
uint8_t volatile emergency_shut_down_button=0;
extern uint8_t emergency_state;


void usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;
 //__HAL_DMA_GET_COUNTER(huart1.hdmarx);
    /* Calculate current position in buffer */
//	huart1.hdmarx->Instance->NDTR ;
    pos = ARRAY_LEN(usart_rx_dma_buffer) -  __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /* We are in "linear" mode */
            /* Process data directly by subtracting "pointers" */
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else {
            /* We are in "overflow" mode */
            /* First process data to the end of buffer */
            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            /* Check and continue with beginning of buffer */
            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
    }
    old_pos = pos;                              /* Save current position as old */

    /* Check and manually update if we reached end of buffer */
    if (old_pos == ARRAY_LEN(usart_rx_dma_buffer)) {
        old_pos = 0;
    }
}
void usart_process_data(const void* data, size_t len) {
    const uint8_t* d = data;
	     uint8_t* vb ;
	int index_of_king=0;
    while (len--) {

			
		king_buffer[index_of_king]   =*d++;
			index_of_king++;
			//		* d ++ =*vb++;
      //  LL_USART_TransmitData8(USART3, *d++);
      //  while (!LL_USART_IsActiveFlag_TXE(USART3));
    }
		
   // while (!LL_USART_IsActiveFlag_TC(USART3));
}



 long volatile packet_received_systemtime_new=0;
 long volatile packet_received_systemtime_old=0;

 long systemtime_new=0;
 long systemtime_old=0;

 uint8_t wth=0;


                         /**   p253/COBS VARIABLES**/

 //p253_t p253 ;//uint8_t  n0f8 ;uint8_t n0f16;uint8_t noff ;  uint8_t total ; uint8_t noff16 ;uint8_t sbai ; uint8_t fbai; uint8_t fbai16;

const unsigned long  size_of_packet=NOF_8BIT+NOF_16BIT*2  + NOF_FRAC+  (NOF_8BIT+NOF_16BIT-1)/8+1  +(( NOF_FRAC-1)/8)+1 +NOF_FRAC16+  (( NOF_FRAC16-1)/8)+1;
const uint8_t  size_of_cobs=size_of_packet+2;

         
uint8_t frac_act_indx[NOF_8BIT]={1,0,0,0,1};
uint8_t frac_act_indx16[NOF_16BIT]={0,0,1};
uint8_t example_received_package[17]={255 ,1 ,2, 3, 40 ,5, 232 ,3 ,208, 7 ,184, 11, 10 ,50 ,30,17 ,4}; 
uint8_t debug_array[30]; 
    
float  data_raw8bit[30]={0};   
float  data_raw16bit[30]={0};   
//float  master_debug[30]={0};  
float volatile bahat[30]={0};   

uint8_t stuffed_example[19]={18,255,1,2,3,40,5,232,3,208,7,184,11,10,50,30,17,4,0}; 
uint8_t decoded_cobs_packet[size_of_packet_macro];
extern uint8_t  decoded_cobs_packet_surfaced[size_of_packet_macro];
	
uint8_t  received_data[19];

	int	parity_of_incomingdata=1;
												/**   BNO055 VARIABLES**/


uint8_t buf_rec[1]={0};

float gx,gy,gz,heigth;
struct bno055_euler_float_t eulerAngles;

int16_t euler_h,euler_p,euler_r;
u8 debug=5;
uint32_t adc_value=0;
uint8_t temp = 3;
uint8_t range;
uint8_t  g_usart1_rx_buf[9];



																	/**   XBEE (uart 1 ) TRANSMITTER VARIBALES**/
volatile uint8_t un_decoded_data[19];

//volatile uint8_t data_received_from_uart_1[19];
volatile uint8_t trasnlation_array_uart_1[size_of_cobs_macro];
uint8_t Rx_indx_for_uart_1, rx_byte_for_uart_1[2],Transfer_cplt_uart_1 ,Rx_Buffer_uart_1[size_of_cobs_macro+5] ,packet_arrived_uart_1 ;
uint8_t tryout19[19];

uint16_t dir_A=0;
uint16_t total_number_of_packet_sent_by_transmitter=0;
float pid_tunning_variable=0;


																	/**  LIDAR RANGE SENSOR VARIBALES**/
uint16_t	lidar_distance;
uint8_t data_received_from_uart_2[size_of_cobs_macro];
uint8_t Rx_indx_for_uart_2, rx_byte_for_uart_2[2],Transfer_cplt_uart_2 ,Rx_Buffer_uart_2[size_of_cobs_macro+5] ,packet_arrived_uart_2 ;





																	/**  ENGINE PID VARIBALES**/
float rounded_down=0;
float old_axis_error_gx=0;
float current_axis_error_gx=0;
float old_axis_error_gy=0;
float current_axis_error_gy=0;
float old_axis_error_gz=0;
float current_axis_error_gz=0;
float old_axis_error_heigth=0;
float current_axis_error_heigth=0;

struct bno055_euler_float_t eulerAngles;
 
uint16_t  thr=0;
uint16_t  min_thr=950;
uint16_t  midlvl_thr=1200;
uint16_t effect_a ,effect_b,effect_c,effect_d; 
int16_t rx_debugger;

float error_slow ;
float error_slower;
    	
uint16_t  take_off_v=0;
uint16_t  take_off_v_old=0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) ; 




extern ADC_HandleTypeDef hadc1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
  //I2C_HandleTypeDef  hi2c;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); //


typedef struct {
char *name;
uint8_t *id;
	uint8_t *connected_event;
} Arguments;

typedef struct {
char *name;
Arguments *args;
} Event;

typedef struct {
char *name;
void (*handler)(void*);
} Handler;
	
Handler *listeners;
Event *poll;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Init(&hadc1);
 
	p253_t p253 ;
		
	p253_init3(&p253,  NOF_8BIT,  NOF_16BIT,  NOF_FRAC, NOF_FRAC16,frac_act_indx,frac_act_indx16);  
	
  struct bno055_t bno055;
	if  (mpu_initialise(&bno055)==1    )  {HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_12 ,GPIO_PIN_SET);}		HAL_Delay(1000);HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_12 ,GPIO_PIN_RESET);
	
	
HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_1);
HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_2);
HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_3);
HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_4);


TIM3->CCR1=min_thr;    //  PA6
TIM3->CCR2=min_thr;    //PA7
TIM3->CCR3=min_thr;   //PB0
TIM3->CCR4=min_thr;     //PB1
	
thr=min_thr; //writing min throttle value at the beginning for arming purposes
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	 

	 
__HAL_UART_ENABLE_IT(&huart1,UART_IT_TC);
//  HAL_UART_Receive_DMA(&huart1,(uint8_t*)rx_byte_for_uart_1,1);
HAL_UART_Receive_DMA(&huart1,(uint8_t*)tryout19,19);
	 
		
		
		//	HAL_UART_Receive_DMA(&huart2, g_usart1_rx_buf, 9);

//__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); //UART_IT_IDLE
	 //lidar_distance=	TF_RX_Proc(g_usart1_rx_buf, 9);
	 
	 	HAL_TIM_Base_Start_IT(&htim1);HAL_Delay(1);
	 
  while (1)
  {  HAL_Delay(20);
		
		 systemtime_new=HAL_GetTick()/1000;
   	parity_of_incomingdata=1;
//	HAL_ADC_Start(&hadc1);HAL_ADC_PollForConversion(&hadc1, 10) ;
//  HAL_ADC_GetValue(&hadc1); HAL_ADC_Stop(&hadc1);
//	adc_value = HAL_ADC_GetValue(&hadc1);
		/**paritiy CONTROL AND Take and process incoming data**/ 


		for(int i=0;i< size_of_cobs_macro-2; i++){
	    if ( (tryout19[i]!=0) )   {}  //gelen verilerde 0 yoksa bisey yapma
	  	else {parity_of_incomingdata=0;break;}	
	 }
		 if  (tryout19[size_of_cobs_macro-1]!=0)   {parity_of_incomingdata=0;}	 
	 
if(parity_of_incomingdata==1){


		
			UnStuffData(tryout19, size_of_cobs_macro,decoded_cobs_packet) ;
			p253_perform_depackage(  p253,decoded_cobs_packet, data_raw8bit,data_raw16bit);
				
		take_off_v=data_raw8bit[1];
		emergency_shut_down_button=data_raw8bit[2];
		dir_A=data_raw16bit[0];
		total_number_of_packet_sent_by_transmitter=data_raw16bit[1];	
		pid_tunning_variable=data_raw16bit[2];
}
		/**                              **/
		
   /**Read gyro(yaw pitch roll) valeus from sensor **/ 
		process_gyro();
		bno055_convert_float_euler_hpr_deg(&eulerAngles);
	  gx=	eulerAngles.h; gy=	eulerAngles.p ; gz=	eulerAngles.r; heigth= lidar_distance;
		
	//lidar_distance=	TF_RX_Proc(g_usart1_rx_buf, 9);
		 
		
//	effect_a=	pid_calculate( gx  ,0, 1,0 ,0 , 0 ,  old_axis_error_gx  , current_axis_error_gx); old_axis_error_gx=current_axis_error_gx;
//	effect_b=		pid_calculate( gy  ,0, 1,0 ,0 , 0 ,  old_axis_error_gy  , current_axis_error_gy); old_axis_error_gy=current_axis_error_gy;
//	effect_c=	pid_calculate( gz  ,0, 1,0 ,0 , 0 ,  old_axis_error_gz  , current_axis_error_gz); old_axis_error_gz=current_axis_error_gz;
//	effect_d=		pid_calculate( heigth  ,50, 1,0 ,0 , 0 ,  old_axis_error_heigth  , current_axis_error_heigth); old_axis_error_heigth=current_axis_error_heigth;
//	

effect_b=pid_calculate2(gy  ,0, pid_tunning_variable/1023*100,0 ,0  );
		
//for(int i =0 ; i<size_of_cobs ;i ++){ data_received_from_uart_1[i]=		un_decoded_data2[i];}
		
		
if(take_off_v!=0){
	thr=dir_A;
}
		
			thr	=thr+150*(take_off_v-take_off_v_old);



		
		 TIM3->CCR1=thr   -effect_b - effect_c  ;    //  PA6
		 TIM3->CCR2=thr    +effect_c-effect_a  ;    //PA7
		 TIM3->CCR3=thr			+effect_c+effect_a				;   //PB0
		 TIM3->CCR4=thr			+effect_b-effect_c		;     //PB1
		 
		 COMM_STATUS_UPDATE( );
		 
		 
		 if(emergency_shut_down_button==1  || emergency_shut_down==1 )
{ 

		 TIM3->CCR1=min_thr ;    //  PA6
		 TIM3->CCR2=min_thr ;
		 TIM3->CCR3=min_thr ;
		 TIM3->CCR4=min_thr ;
	
	take_off_v=0;
	take_off_v_old=0;
	
}


		 
		 
		 // paket gelir.  1. sn de .   2. sn de bir paket daha gelir. 2 paket geldi. 3. sn olur ama paket yok. 4. sn olur paket yok .    imdiki
//zamandan son paket gelme zamanini çikarirsin.
		 take_off_v_old=take_off_v;
		 
		HAL_Delay(10);
		
		
		
		
		
		
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
