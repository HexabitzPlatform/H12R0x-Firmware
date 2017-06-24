/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved
		
    File Name     : H07R0.c
    Description   : Header file for module H07R0.
										Analog inputs 0 - 5V/10V (AD628ARZ) 
*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H07R0_H
#define H07R0_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H07R0

/* Port-related definitions */
#define	NumOfPorts		4
#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1 
#define _P2 
#define _P4 
#define _P5 

/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#define _Usart4 1

/* Port-UART mapping */
#define P1uart &huart4
#define P2uart &huart2
#define P3uart &huart3
#define P4uart &huart1

/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT	GPIOA
#define	USART1_RX_PORT	GPIOA
#define	USART1_AF				GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT	GPIOA
#define	USART2_RX_PORT	GPIOA
#define	USART2_AF				GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT	GPIOB
#define	USART3_RX_PORT	GPIOB
#define	USART3_AF				GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT	GPIOA
#define	USART4_RX_PORT	GPIOA
#define	USART4_AF				GPIO_AF4_USART4


/* Module-specific Definitions */
#define	_AI_PIN				GPIO_PIN_0
#define	_AI_PORT			GPIOB
#define	_AI_Channel		ADC_CHANNEL_8

/* H01R0_Status Type Definition */  
typedef enum 
{
  H07R0_OK = 0,
	H07R0_ERR_UnknownMessage = 1,
	H07R0_ERROR = 255
} Module_Status;

/* Indicator LED */
#define _IND_LED_PORT		GPIOB
#define _IND_LED_PIN		GPIO_PIN_14


/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;


/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);




/* -----------------------------------------------------------------------
	|														Message Codes	 														 	|
   ----------------------------------------------------------------------- 
*/



	
/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/



/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/




#endif /* H07R0_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
