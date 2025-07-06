/*
 * it.c
 *
 *  Created on: Jun 22, 2025
 *      Author: bhoomika
 */

#include "main.h"
extern TIM_HandleTypeDef htimer3;

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htimer3);
}

