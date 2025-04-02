/*
 * HX711.h
 *
 *  Created on: Mar 30, 2025
 *      Author: shrutishah
 */

#ifndef HX711_H
#define HX711_H

#include <stdint.h>
#include "stm32l4xx_hal.h"  // Adjust for your STM32 series


void HX711_Init(TIM_HandleTypeDef *timer);
int32_t HX711_Read(void);
//void HX711_PowerDown(void);
//void HX711_WakeUp(void);
//void HX711_SetCalibration(int32_t offset, float scale);
int HX711_GetWeight(void);
void delay_us(uint16_t us);

#endif  // HX711_H

