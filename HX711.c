/*
 * HX711.c
 *
 *  Created on: Mar 30, 2025
 *      Author: shrutishah
 */

#include "HX711.h"
#include "stm32l4xx_hal.h"  // Adjust for your STM32 series
#include <stdio.h>

#define HX711_TIMEOUT 1000000  // Prevent infinite loop

// GPIO Definitions (Update for your STM32 board)
#define HX711_DATA_GPIO   GPIOB
#define HX711_DATA_PIN    GPIO_PIN_0   // HX711 DT → PA1

#define HX711_CLK_GPIO    GPIOA
#define HX711_CLK_PIN     GPIO_PIN_0   // HX711 SCK → PA2

static int32_t OFFSET = 0;  // Calibration Offset
static float SCALE_FACTOR = 1;  // Calibration Scale Factor
TIM_HandleTypeDef *HX711_Timer;

void delay_us(uint16_t us) { //0.01 of a microsecond
	__HAL_TIM_SET_COUNTER(HX711_Timer, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(HX711_Timer) < us){
		continue;
	}
		//;  // wait for the counter to reach the us input in the parameter
}

void HX711_Init(TIM_HandleTypeDef *timer) {
    // Ensure SCK starts LOW
	HX711_Timer = timer;
	HAL_TIM_Base_Start(HX711_Timer);
    HAL_GPIO_WritePin(HX711_CLK_GPIO, HX711_CLK_PIN, GPIO_PIN_RESET);
}


int32_t HX711_Read(void) {
    int32_t data = 0;

    // 1. Wait until HX711 is ready (DATA goes LOW)
    uint32_t timeout = HX711_TIMEOUT;
    //Data pin goes low when data is ready
    while (HAL_GPIO_ReadPin(HX711_DATA_GPIO, HX711_DATA_PIN)) {
        if (--timeout == 0) {
            printf("HX711 Timeout! Check wiring.\n");
            return -1;
        }
    }
    delay_us(12);

    // 2. Read 24 bits (MSB first)
    for (uint8_t i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(HX711_CLK_GPIO, HX711_CLK_PIN, GPIO_PIN_SET);
        delay_us(5);
        data = (data << 1) | HAL_GPIO_ReadPin(HX711_DATA_GPIO, HX711_DATA_PIN);
        delay_us(20);
        HAL_GPIO_WritePin(HX711_CLK_GPIO, HX711_CLK_PIN, GPIO_PIN_RESET);
        delay_us(25);
    }

    // 3. Set Gain (128 by default → 1 extra clock pulse)
    HAL_GPIO_WritePin(HX711_CLK_GPIO, HX711_CLK_PIN, GPIO_PIN_SET);
    delay_us(25);
    HAL_GPIO_WritePin(HX711_CLK_GPIO, HX711_CLK_PIN, GPIO_PIN_RESET);

    // 4. Convert signed 24-bit value
    if (data & 0x800000) {  // If MSB is 1 (negative number)
        data |= 0xFF000000;  // Sign extend to 32-bit
    }

    return data;
}

//void HX711_PowerDown(void) {
//    HAL_GPIO_WritePin(HX711_CLK_GPIO, HX711_CLK_PIN, GPIO_PIN_SET);  // SCK HIGH for >60µs
//    HAL_Delay(1);
//}
//
//void HX711_WakeUp(void) {
//    HAL_GPIO_WritePin(HX711_CLK_GPIO, HX711_CLK_PIN, GPIO_PIN_RESET);  // SCK LOW wakes HX711
//    HAL_Delay(1);
//}
//
//void HX711_SetCalibration(int32_t offset, float scale) {
//    OFFSET = offset;
//    SCALE_FACTOR = scale;
//}

float HX711_GetWeight(void) {
    int32_t raw_data = HX711_Read();
    if (raw_data == -1) return -1;  // Error case

    return (raw_data - OFFSET) / SCALE_FACTOR;
}


