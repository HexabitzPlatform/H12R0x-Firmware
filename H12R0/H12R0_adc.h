/*
 * H09R0_adc.h
 *
 *  Created on: ١٧‏/٠٣‏/٢٠٢١
 *      Author: shift
 */

//#ifndef H09R0_ADC_H_
//#define H09R0_ADC_H_


#ifndef H12R0_ADC_H_
#define H12R0_ADC_H_



#endif /* H12R0_ADC_H_ */

#include "stm32f0xx_hal.h"

DMA_HandleTypeDef hdma_adc;
ADC_HandleTypeDef hadc;

#define Voltage_input_Pin GPIO_PIN_0
#define Voltage_input_GPIO_Port GPIOB
