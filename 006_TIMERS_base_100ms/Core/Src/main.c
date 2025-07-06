/*
 * main.c
 *
 *  Created on: Jun 22, 2025
 *      Author: bhoomika
 */

#include "main.h"
#include "string.h"
#include <stdio.h>

#define TRUE   1
#define FALSE  0

void Error_Handler(void);

void SystemClockConfig(void);

void TIMER3_Init(void);

void GPIO_Init(void);

TIM_HandleTypeDef htimer3;

int main(void)
{

	HAL_Init();

	SystemClockConfig();

	GPIO_Init();

	TIMER3_Init();

	//start the timer
	if(HAL_TIM_Base_Start(&htimer3) != HAL_OK){
		Error_Handler();
	}

	while(1)
	{
		//to check the update event. The timer status register will be notified.
		//so loop until Update Event flag is set
		while(! (TIM3->SR & TIM_SR_UIF) );
		//the required delay has been elapsed and the user code can be executed

		TIM3->SR &= ~(1 << 0); //clearing the bit
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	}

	return 0;

}

void SystemClockConfig(void)
{

}

//high level initializations//
void TIMER3_Init(void)
{
//	__HAL_RCC_TIM3_CLK_ENABLE();
	htimer3.Instance         = TIM3;
	htimer3.Init.Prescaler   = 24;
	htimer3.Init.Period      = 64000 - 1;
	htimer3.Init.CounterMode = TIM_COUNTERMODE_UP;
	if( HAL_TIM_Base_Init(&htimer3) != HAL_OK){
		Error_Handler();
	}

}

void GPIO_Init(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef LED;
	LED.Mode = GPIO_MODE_OUTPUT_PP;
	LED.Pin  = GPIO_PIN_13;
	LED.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &LED);

//	GPIO_InitTypeDef tim;
//	tim.Alternate = GPIO_AF2_TIM3;
//	tim.Pin       = GPIO_PIN_6;
//	tim.Mode      = GPIO_MODE_AF_PP;
//	HAL_GPIO_Init(GPIOA, &tim);

}

void Error_Handler(void)
{
	while(1);
}

