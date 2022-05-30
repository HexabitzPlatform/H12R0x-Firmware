/*
 BitzOS (BOS) V0.2.6 - Copyright (C) 2017-2022 Hexabitz
 All rights reserved

 File Name     : H12R0_dma.c
 Description   : Peripheral ADC setup header file.
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H12R0_ADC_H_
#define H12R0_ADC_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

DMA_HandleTypeDef hdma_adc;
ADC_HandleTypeDef hadc;

#define Voltage_input_Pin GPIO_PIN_0
#define Voltage_input_GPIO_Port GPIOB

#endif /* H12R0_ADC_H_ */
