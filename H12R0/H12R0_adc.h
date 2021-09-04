/*
    BitzOS (BOS) V0.2.5 - Copyright (C) 2017-2021 Hexabitz
    All rights reserved

    File Name     : H12R0_dma.c
    Description   : Peripheral ADC setup header file.
*/
#ifndef H12R0_ADC_H_
#define H12R0_ADC_H_



#endif /* H12R0_ADC_H_ */

#include "stm32f0xx_hal.h"

DMA_HandleTypeDef hdma_adc;
ADC_HandleTypeDef hadc;

#define Voltage_input_Pin GPIO_PIN_0
#define Voltage_input_GPIO_Port GPIOB
