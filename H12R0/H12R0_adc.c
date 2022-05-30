/*
 BitzOS (BOS) V0.2.6 - Copyright (C) 2017-2022 Hexabitz
 All rights reserved

 File Name     : H12R0_dma.c
 Description   : Peripheral ADC setup source file.
 */
/* Includes ------------------------------------------------------------------*/
#include <H12R0_adc.h>

void ADC_Channel_config(void){

	ADC_ChannelConfTypeDef sConfig = {0};
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {

	  }
	      GPIO_InitStruct.Pin = Voltage_input_Pin;
	      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	      GPIO_InitStruct.Pull = GPIO_NOPULL;
	      HAL_GPIO_Init(Voltage_input_GPIO_Port, &GPIO_InitStruct);
}


