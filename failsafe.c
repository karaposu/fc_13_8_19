#include "failsafe.h"
#include "main.h"

/*
This block is in charge of detecting any event that could
jeopardize the integrity of the aircraft and reacts executing a
protocol were a parachute is released in order to help the UAV
land safely. The alarms that trigger the emergency protocol are
the following:
- Signal lost: This happens when the user’s control
signal is poor or has been lost.
- Lack of synchrony in RF signal: The signal sent by the
user to the aircraft lacks of synchrony.
Telecommunication and Video area designed a
module to detect such event.
- Noisy signal: Noise in the signal that affects the
aircrafts control.
- Critical Speed: Detected by the air speed sensor. If the
aircraft´s speed is too low lift is lost. Data provided by
sensors are compared with a critical theoretical value.
- Critical battery level: The emergency system
compares the battery sensor´s signal with a theoretical
critical value.
- Activated by the user: There will be a switch in the
user’s controller that will activate the emergency
system if he thinks is necessary.

If any of the five emergency conditions are activated, a three
seconds timer will be activated in the emergency system block.
This time is a pre caution in case of possible sensor malfunction
or misreading. If the variable that has been activated does not
return to its normal value, a flag is set on high in the control unit
by the emergency system module. This high value will cause
the control unit to change its current state to emergency state.
The next sequence will be executed once the emergency state
is on:

- Brushless motor off
- All servomotors change the control surfaces positions
back to neutral position.
- Three seconds timer is activated in order to wait for
the UAV’s own aerodynamics stabilize it
- Parachute releasing mechanism is activated
- All servo motors are turned off
*/
 extern long volatile packet_received_systemtime_new;
 extern long volatile packet_received_systemtime_old;
extern uint8_t volatile emergency_shut_down;
//typedef enum
//{
//  SIGNAL_LOST     = 0x00U,  /*!< When the signal is lost for 5 seconds */
//  SIGNAL_WEAK         = 0x01U,  /*!< When the signal is lost for 1 seconds */
//  SIGNAL_CORRUPTED   = 0x02U,  /*!< When received signal has wrong parity (noise) */
//  CRITICAL_SPEED                   = 0x03U,  /*!< Measured speed is above x mps in euler axis */
//  CRITICAL_BATTERY_LEVEL = 0x04U,  /*!< Battery low */
//  EMERGENCY_SHUTD0WN                  = 0x05U,  /*!< User button activated         */
//  ALL_IS_OK              = 0x06U   /*!< No emergency sitiation exist       */
//} EMERGENCY_T;

typedef struct 
{
 uint8_t SIGNAL_LOST;            /*!< When the signal is lost for 5 seconds */
	uint8_t SIGNAL_WEAK;         /*!< When the signal is lost for 1 seconds */
 uint8_t  SIGNAL_CORRUPTED ;  /*!< When received signal has wrong parity (noise) */
 uint8_t  CRITICAL_SPEED    ;  /*!< Measured speed is above x mps in euler axis */
 uint8_t  CRITICAL_BATTERY_LEVEL ;  /*!< Battery low */
 uint8_t  EMERGENCY_SHUTD0WN   ;     /*!< User button activated         */
 
} EMERGENCY_STATUS;

EMERGENCY_STATUS  ES;

//int emergency_button_status, int current_packet_parity ,int vx,int vy,int vz ,int battery_voltage
	
 uint8_t COMM_STATUS_UPDATE( )
		{
				long temp=HAL_GetTick()/1000-packet_received_systemtime_new;
				//long temp=packet_received_systemtime_new-packet_received_systemtime_old;
				if(temp>5){
						 ES.SIGNAL_LOST=1;ES.SIGNAL_WEAK=0;}
				else if(		(temp<=5 ) && 	(temp>2 )){
						 ES.SIGNAL_LOST=0;   ES.SIGNAL_WEAK=1;}
				else 
						ES.SIGNAL_LOST=0;   ES.SIGNAL_WEAK=0;
	    	
					if(ES.SIGNAL_LOST==1)  emergency_shut_down=1;else {emergency_shut_down=0;}
				
				
		}
		


//void  Emergency_Status_Update() 
//{ 
// 
//	COMM_STATUS_UPDATE();
//}
	

// 
//                         switch(1){
//																			
//																			case SIGNAL_LOST:
//																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
//																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
//																			break;
//																			
//																			case 1:
//																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
//																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
//																			break;
//																			
//																			case 2:
//																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
//																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
//																			break;
//																			
//																			case 3: 
//																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
//																			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
//																			break;
//																			}

