#ifndef FAILSAFE_H_ 
#define FAILSAFE_H_ 

#include "main.h" 
	
//	uint8_t emergency_state;
	
//typedef enum
//{
//  SIGNAL_LOST     = 0x00U,  /*!< When the signal is lost for 5 seconds */
//  SIGNAL_WEAK         = 0x01U,  /*!< When the signal is lost for 1 seconds */
//  SIGNAL_CORRUPTED   = 0x02U,  /*!< When received signal has wrong parity (noise) */
//  CRITICAL_SPEED                   = 0x03U,  /*!< Measured speed is above x mps in euler axis */
//  CRITICAL_BATTERY_LEVEL = 0x04U,  /*!< Battery low */
//  EMERGENCY_SHUTD0WN                  = 0x05U,  /*!< User button activated         */
//  ALL_IS_OK              = 0x06U   /*!< No emergency sitiation exist       */
//} EMERGENCY;

 extern uint8_t COMM_STATUS_UPDATE( ) ;

#endif