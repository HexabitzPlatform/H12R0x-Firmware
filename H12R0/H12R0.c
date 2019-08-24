/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved

    File Name     : H12R0.c
    Description   : Source code for module H12R0.
										Analog inputs 0 - 5V/10V (AD628ARZ) 
		
		Required MCU resources : 
		
			>> USARTs 1,2,3,4 for module ports.
			>> ADC_IN8 for AD628ARZ.
			
*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;


/* Module exported parameters ------------------------------------------------*/
module_param_t modParam[NUM_MODULE_PARAMS] = {{.paramPtr=NULL, .paramFormat=FMT_FLOAT, .paramName=""}};


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/	


/* Create CLI commands --------------------------------------------------------*/




/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/

/* --- H12R0 module initialization. 
*/
void Module_Init(void)
{	
	/* Array ports */
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
	
	/* ADC */
	//MX_ADC_Init();
  
}

/*-----------------------------------------------------------*/

/* --- H12R0 message processing task. 
*/
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift)
{
	Module_Status result = H12R0_OK;
	
	switch (code)
	{

		default:
			result = H12R0_ERR_UnknownMessage;
			break;
	}			

	return result;	
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands 
*/
void RegisterModuleCLICommands(void)
{

}

/*-----------------------------------------------------------*/

/* --- Get the port for a given UART. 
*/
uint8_t GetPort(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART4)
			return P1;
	else if (huart->Instance == USART2)
			return P2;
	else if (huart->Instance == USART3)
			return P3;
	else if (huart->Instance == USART1)
			return P4;
		
	return 0;
}


/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/



/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/



/*-----------------------------------------------------------*/


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
