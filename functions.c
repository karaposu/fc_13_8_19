#include "functions.h"
#include "main.h"

#define  siz 19
extern uint8_t wth;
extern I2C_HandleTypeDef hi2c1;

extern UART_HandleTypeDef huart2 ; 
 extern UART_HandleTypeDef huart1 ;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;
extern int16_t rx_debugger;
 
extern int16_t euler_h,euler_p,euler_r;


extern uint8_t counter ,buffrec[5],cout_doku;
extern  uint8_t Rx_indx, Rx_data[2], TBuffer[150],Transfer_cplt ,Rx_Buffer[30] ;
extern	void resolve_package( unsigned char *ptr);

 uint16_t	counter_tim2=0;  //isiklardan sorumlu counter. 
volatile  uint8_t	package_arriving=0;
 volatile uint16_t packet_arrived=0;
volatile   uint16_t packet_arrived_previous=0;

#define TFMINI_DATA_Len             9
#define TFMINT_DATA_HEAD            0x59



extern uint8_t temp;
extern uint8_t range;
//uint8_t	 RecBuf[8];
	//uint8_t	 is_working[3];
		uint8_t	 is_working_pg[3];
		
//	/**  LIDAR VARIBALES**/	
//		 uint16_t packet_arrived=0;
//  uint16_t packet_arrived_previous=0;
//extern uint8_t counter ,buffrec[5],cout_doku;
//extern  uint8_t Rx_indx, Rx_data[2], TBuffer[150],Transfer_cplt ,Rx_Buffer[30] ;
//extern  uint8_t un_decoded_data[13];
//		
//	

/**   XBEE RECEIVER VARIBALES**/
extern const  uint8_t  size_of_cobs;
	 uint16_t packet_arrived2=0;
  uint16_t packet_arrived_previous2=0;
extern uint8_t	 RecBuf2[8];


extern uint8_t un_decoded_data2[  size_of_cobs_macro ]   ;
extern uint8_t counter2 ,buffrec2[5],cout_doku2;
extern uint8_t Rx_indx2, Rx_data2[2], TBuffer2[150],Transfer_cplt2 ,Rx_Buffer2[30] ;

extern uint8_t un_decoded_data3[  size_of_cobs_macro ]   ;


void UnStuffData(volatile const unsigned char *ptr, unsigned long length,unsigned char *dst) 
{ 
	
	
  const unsigned char *end = ptr + length; 
  while (ptr < end) 
  { 
  int i, code = *ptr++; for (i=1; i<code; i++) *dst++ = *ptr++; if (code < 0xFF) *dst++ = 0; } 

}


uint16_t TF_RX_Proc(uint8_t *buf, uint32_t len)
{
    uint32_t i = 0;
    uint8_t chk_cal = 0;
    uint16_t cordist = 0;

    if(TFMINI_DATA_Len == len)
    {
        if((TFMINT_DATA_HEAD == buf[0])&&(TFMINT_DATA_HEAD == buf[1]))
        {
            for(i = 0; i < (TFMINI_DATA_Len - 1); i++)
            {
                chk_cal += buf[i];
            }

            if(chk_cal == buf[TFMINI_DATA_Len - 1])
            {
                cordist = buf[2] | (buf[3] << 8);
                      return   cordist;
                /*cordist > TFMINI_ACTION_DIST cm, PA8 set Low;
                  cordist <= TFMINI_ACTION_DIST cm, PA8 set High.*/
                
            }
        }
    }
}




	extern uint8_t  received_data[19];
extern uint8_t example_data[19];
extern uint8_t trasnlation_array_uart_1[size_of_cobs_macro];
volatile uint8_t data_received_from_uart_1[19];

extern uint8_t Rx_indx_for_uart_1, rx_byte_for_uart_1[2],Transfer_cplt_uart_1 ,Rx_Buffer_uart_1[30],packet_arrived_uart_1 ; ;
extern uint8_t decoded_cobs_packet[size_of_packet_macro];
extern float volatile data_raw8bit[30];   
extern float volatile data_raw16bit[30];  
 uint8_t  decoded_cobs_packet_surfaced[size_of_packet_macro];
// extern p253_t p253 ;


 extern volatile long packet_received_systemtime_new;
 extern volatile long packet_received_systemtime_old;

		void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  
{
	

	 if (huart->Instance == USART1)  //current UART
       {
				 packet_arrived++;  
				
		
    packet_received_systemtime_old= packet_received_systemtime_new;
	  packet_received_systemtime_new=HAL_GetTick()/1000;
		
		     HAL_TIM_Base_Start_IT(&htim2);
			 }
	
}












//		void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  
//{


//	
//    if (huart->Instance == USART1)  //current UART
//        {
//					
//				int a=0;
//				   
//       if (Rx_indx_for_uart_1==0) {for (int i=0;i<30;i++) Rx_Buffer_uart_1[i]=254;}   //clear Rx_Buffer before receiving new data 

//        if (rx_byte_for_uart_1[0]!=a) //if received data different from 0
//            {
//							Transfer_cplt_uart_1=0;
//						 
//            Rx_Buffer_uart_1[Rx_indx_for_uart_1++]=rx_byte_for_uart_1[0];    //add data to Rx_Buffer
//            }
//        else            //if received data = 0
//            {
//           
//								 		 		for( int i=0; i < 19; i++ ){
//												data_received_from_uart_1[i]=	 *( Rx_Buffer_uart_1 + i ) ;  }
//											packet_arrived_uart_1++;
//											
//											//	UnStuffData(data_received_from_uart_1, size_of_cobs_macro,decoded_cobs_packet) ;
//												 for( int i=0; i < 19; i++ ){trasnlation_array_uart_1[i]=	 		data_received_from_uart_1[i] ;  }
//										
//					
//												if (packet_arrived_uart_1==1) //if received data different from 0
//            {
//													HAL_TIM_Base_Start_IT(&htim2);
//												
//						}
//												
//							Rx_indx_for_uart_1=0;
//							 Transfer_cplt_uart_1=1;
//							 
//					 
//							 
//            }
//		HAL_UART_Receive_DMA(&huart1, rx_byte_for_uart_1, 1);
//          }
//				}
		







				 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
											
																					/********************TIMER2 Fonskyonu  ( sadece ledler için )************************************************************/							
											if (htim->Instance == TIM2) {
																	
																	
											//Buraya her girdiginde x. led sönecek x+1 . led yanacak *******/
														
	                      if (package_arriving== 1){
      
																				
																			switch(counter_tim2){
																			
																			case 0:
																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
																			break;
																			
																			case 1:
																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
																			break;
																			
																			case 2:
																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
																			break;
																			
																			case 3: 
																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
																			break;
																			}

																		counter_tim2++;		
																			
												 						if(counter_tim2==4)	{
																									
																		counter_tim2=0;
																		}			


																	}																		
																				
																			

							}						
											
							
							
							
							
							
								if (htim->Instance == TIM1) {
									
								COMM_STATUS_UPDATE( );
										if (packet_arrived>packet_arrived_previous) {
											wth++;
												package_arriving=1;
										}
										else{
											package_arriving=0;
										}
									packet_arrived_previous=packet_arrived;
									
									
								}
																									
																											
																											
			
}			
				
				
		




				
		
		void process_gyro()

{
	
	
//	 bno055_read_euler_hrp( bno055_euler_t *euler)
    bno055_read_euler_h( &euler_h );

    bno055_read_euler_p( &euler_p );

    bno055_read_euler_r( &euler_r );

}

s8	hal_i2c_write( u8 dev_addr,u8  reg_addr,u8  *data,   u8  length) {
	

	u8 buffer[8];
	u8 i =0;
	
	buffer[0] =reg_addr;  
	for( i =0;i<length;i++)  // 0 için buffer_global(1) e data_gl[0]  i atiyor.    // 1 için 
	 {
	   buffer[i+1]=  (*data+i)  ;
	 }
												
	//HAL_I2C_Master_Transmit(&hi2c1,dev_addr,buffer,length+1,100);
	
		if  (HAL_I2C_Master_Transmit(&hi2c1,dev_addr,buffer,length+1,100)==HAL_OK  ){return 0;}
   else {return -1;}
}





s8 hal_i2c_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 r_len) {
	
u8 i =0;  u8 array[8] = {0};   array[0] = reg_addr;
	
	HAL_I2C_Master_Transmit(&hi2c1,dev_addr,&reg_addr,1,100);
	if  (	HAL_I2C_Master_Receive(	&hi2c1,dev_addr, array, r_len, 100)==HAL_OK  ){
			
				for (i = 0; i < r_len; i++)
	    	*(reg_data + i) = array[i];return 0;
		}
   else {return -1; }

		
}
void hal_i2c_delay(u32 a) {
	
	HAL_Delay(a);
  
}
		
//				
//void process_gyro()

//{
//	//uint8_t	 is_working_pg[3];
//	 is_working_pg[0] =2;
////	 bno055_read_euler_hrp( bno055_euler_t *euler);
// is_working_pg[0] =bno055_read_euler_h( &euler_h );

//   // bno055_read_euler_p( &euler_p );

//   // bno055_read_euler_r( &euler_r );

//}
//		
//		


	char mpu_initialise(struct bno055_t *bno055 ) {
	    uint8_t	 is_working[3];
				if  (HAL_I2C_IsDeviceReady(&hi2c1,0x50, 3, 1000)==HAL_OK  )
			{ is_working[0]=1;
			HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_13 ,GPIO_PIN_SET);
			}
				HAL_Delay(10);
			
	
    bno055->dev_addr = 0x50;
    bno055->bus_read = hal_i2c_read;
    bno055->bus_write = hal_i2c_write;
    bno055->delay_msec = hal_i2c_delay;
	
			
			 bno055_init( bno055 );//	HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_14 ,GPIO_PIN_SET);
      bno055_set_operation_mode( BNO055_OPERATION_MODE_NDOF );  HAL_Delay(100);

    // bno055_get_operation_mode(&RecBuf[3] );  HAL_Delay(15);
	  bno055_get_operation_mode(&is_working[1] );   HAL_Delay(15);
			
			if  (is_working[1]  ==BNO055_OPERATION_MODE_NDOF  ){  is_working[1]=1;   }
			else {   is_working[1]=0; }
			
			 // 	HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_13 ,GPIO_PIN_SET);
			
bno055_set_accel_range( 0x01 );
bno055_get_accel_range( &range );
bno055_get_accel_unit( &temp );
bno055_set_gyro_range( 0x04 );
bno055_get_gyro_range( &range );
   
			
			
		    if  (is_working[0]  ==1  && is_working[1]  ==1     ){  
   
         
			return 1;

		}	
			
			
			
			
			
			
			
}

float round_v(float var) 
{ 
    // 37.66666 * 100 =3766.66 
    // 3766.66 + .5 =37.6716    for rounding off value 
    // then type cast to int so value is 3766 
    // then divided by 100 so the value converted into 37.66 
    float value = (int)(var * 100 + .5); 
	// return  ((float32_t ) value  ) /100; 
	
   // return (float)value / 100; 
} 

//	char mpu_initialise() {
//	    uint8_t	 is_working[3];
//				if  (HAL_I2C_IsDeviceReady(&hi2c1,0x50, 3, 1000)==HAL_OK  )
//			{ is_working[0]=1;
//			HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_13 ,GPIO_PIN_SET);
//			}
//				HAL_Delay(10);
//			
//		extern  struct bno055_t bno055;
//    bno055.dev_addr = 0x50;
//    bno055.bus_read = hal_i2c_read;
//    bno055.bus_write = hal_i2c_write;
//    bno055.delay_msec = hal_i2c_delay;
//	
//			
//			 bno055_init( &bno055 );//	HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_14 ,GPIO_PIN_SET);
//			
//}

//	char mpu_initialise() {
//	    uint8_t	 is_working[3];
//				if  (HAL_I2C_IsDeviceReady(&hi2c1,BNO055_I2C_ADDR3, 3, 1000)==HAL_OK  )
//			{device_active=1;  is_working[0]=1;
//			
//			}
//				HAL_Delay(10);
//			
//			 struct bno055_t bno055;
//    bno055.dev_addr = BNO055_I2C_ADDR3;
//    bno055.bus_read = hal_i2c_read;
//    bno055.bus_write = hal_i2c_write;
//    bno055.delay_msec = hal_i2c_delay;
//		
//    bno055_init( &bno055 );	//HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_15 ,GPIO_PIN_SET);
//			
//		bno055_set_operation_mode( BNO055_OPERATION_MODE_NDOF );  HAL_Delay(100);

//    // bno055_get_operation_mode(&RecBuf[3] );  HAL_Delay(15);
//	  bno055_get_operation_mode(&is_working[1] );   HAL_Delay(15);
//			
//			if  (is_working[1]  ==BNO055_OPERATION_MODE_NDOF  ){  is_working[1]=1;   }
//			else {   is_working[1]=0; }
//			
//			 // 	HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_13 ,GPIO_PIN_SET);
//			
//bno055_set_accel_range( 0x01 );
//bno055_get_accel_range( &range );
//bno055_get_accel_unit( &temp );
//bno055_set_gyro_range( 0x04 );
//bno055_get_gyro_range( &range );
//   
//			
//			
//		    if  (is_working[0]  ==1  && is_working[1]  ==1     ){  
//   
//         
//			return 1;

//		}	
//			
//			
//			
//}
//							 



	