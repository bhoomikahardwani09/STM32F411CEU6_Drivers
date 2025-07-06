/*
 * msp.c
 *
 *  Created on: Jun 30, 2025
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

void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef timoc_ch_gpio;
	//enable the peripheral clock
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	//Configure GPIOs to behave as TIM2 channel 1,2,3,4
	timoc_ch_gpio.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3;
	timoc_ch_gpio.Mode      = GPIO_MODE_AF_PP;
	timoc_ch_gpio.Pull      = GPIO_NOPULL;
	timoc_ch_gpio.Speed     = GPIO_SPEED_FREQ_LOW;
	timoc_ch_gpio.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &timoc_ch_gpio);

	timoc_ch_gpio.Pin       = GPIO_PIN_10;
	timoc_ch_gpio.Mode      = GPIO_MODE_AF_PP;
	timoc_ch_gpio.Pull      = GPIO_NOPULL;
	timoc_ch_gpio.Speed     = GPIO_SPEED_FREQ_LOW;
	timoc_ch_gpio.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOB, &timoc_ch_gpio);

	//NVIC settings
	HAL_NVIC_SetPriority(TIM2_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_UART;
	//This function is for the low level initializations of the USART2 peripherals.

	//1. Enable the clock for the USART2 peripherals
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	//2. Do the pin muxing configurations
	//PA2 => TX, PA3 => RX
	GPIO_UART.Pin        = GPIO_PIN_2;
	GPIO_UART.Mode       = GPIO_MODE_AF_PP;
	GPIO_UART.Pull       = GPIO_PULLUP;
	GPIO_UART.Speed      = GPIO_SPEED_FREQ_LOW;
	GPIO_UART.Alternate  = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_UART);

	GPIO_UART.Pin        = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOA, &GPIO_UART);

	//3.Enable the IRQ and set up the priority (NVIC Settings)
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_SetPriority(USART2_IRQn, 15, 0);

}

