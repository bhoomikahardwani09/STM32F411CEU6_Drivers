/*
 * msp.c
 *
 *  Created on: Jun 22, 2025
 *      Author: bhoomika
 */

#include "stm32f4xx_hal.h"

//LOW LEVEL PROCESSOR SPECIFIC INITIALISATIONS
void HAL_MspInit(void)
{
	//1. Set up the priority grouping of the arm cortex mx processor
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	//2.enable the required system exceptions of the arm cortex mx processor
	//Enabled the system exception : USGFAULTENA, BUSFAULTENA, MEMFAULTENA
	SCB->SHCSR |= 0x7 << 16;

	//3.configure the priority for the system exceptions
	//default priority will be set to 0
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htimer)
{
	//enable the clock for TIM3 peripheral//
	__HAL_RCC_TIM3_CLK_ENABLE();

	//enable IRQ of TIM3//
	HAL_NVIC_EnableIRQ(TIM3_IRQn);

	//setup the priority for the IRQ no.//
	HAL_NVIC_SetPriority(TIM3_IRQn, 15,0);

}

