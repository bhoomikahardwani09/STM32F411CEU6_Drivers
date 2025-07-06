/*
 * it.c
 *
 *  Created on: Jun 22, 2025
 *      Author: bhoomika
 */

#include "main.h"

void SysTick_Handler(void){

	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


