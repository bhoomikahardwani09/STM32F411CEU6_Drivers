/*
 * main.c
 *
 *  Created on: June 30, 2025
 *      Author: bhoomika
 */

#include "main.h"
#include "string.h"
#include <stdio.h>

#define TRUE   1
#define FALSE  0

void Error_Handler(void);

void SystemClockConfig(uint8_t clock_freq);

void TIMER2_Init(void);

void UART2_Init(void);

void GPIO_Init(void);

TIM_HandleTypeDef htimer2;
UART_HandleTypeDef h_UART2;

uint32_t pulse1_val = 25000; //500Hz
uint32_t pulse2_val = 12500; //1000Hz
uint32_t pulse3_val = 6250;  //2000Hz
uint32_t pulse4_val = 3125;  //4000Hz
uint32_t ccr_val;
int main(void)
{
	HAL_Init();

	SystemClockConfig(SYS_CLK_FREQ_50MHZ);

	GPIO_Init();

	TIMER2_Init();

//	UART2_Init();

	if(HAL_TIM_OC_Start_IT(&htimer2, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_TIM_OC_Start_IT(&htimer2, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_TIM_OC_Start_IT(&htimer2, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_TIM_OC_Start_IT(&htimer2, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}

	while(1);

	return 0;
}

void SystemClockConfig(uint8_t clock_freq)
{
	RCC_OscInitTypeDef osc_init;
	RCC_ClkInitTypeDef clk_init;
	uint32_t flatency = 0;

	osc_init.OscillatorType       = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
	osc_init.HSEState             = RCC_HSE_ON;
	osc_init.LSEState             = RCC_LSE_ON;
	osc_init.PLL.PLLState         = RCC_PLL_ON;
	osc_init.PLL.PLLSource        = RCC_PLLSOURCE_HSE;

	switch(clock_freq)
	{
	case SYS_CLK_FREQ_50MHZ:
	{
		osc_init.PLL.PLLM = 25;
		osc_init.PLL.PLLN = 100;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLQ = 2;

		clk_init.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
		clk_init.AHBCLKDivider  = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

		flatency = FLASH_ACR_LATENCY_1WS;

		break;

	}
	case SYS_CLK_FREQ_80MHZ:
	{
		osc_init.PLL.PLLM = 25;
		osc_init.PLL.PLLN = 160;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLQ = 2;

		clk_init.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
		clk_init.AHBCLKDivider  = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

		flatency = FLASH_ACR_LATENCY_3WS;

		break;

	}
	case SYS_CLK_FREQ_100MHZ:
	{
		//enable the clock for power controller//
		__HAL_RCC_PWR_CLK_ENABLE();

		//set regulator voltage scale as 1//
		__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

//		//turn on the over drive mode of the voltage regulator//
		osc_init.PLL.PLLM = 25;
		osc_init.PLL.PLLN = 200;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLQ = 2;

		clk_init.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
		clk_init.AHBCLKDivider  = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

		flatency = FLASH_ACR_LATENCY_4WS;

		break;
	}
	default:
		return;

	}

	//enables the PLL engine//
	if(HAL_RCC_OscConfig(&osc_init) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_RCC_ClockConfig(&clk_init, flatency) != HAL_OK)
	{
		Error_Handler();
	}

	//SYSTICK Configurations//
	//to enable the interrupts for every 1ms//
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

//high level initializations//
void TIMER2_Init(void)
{
	TIM_OC_InitTypeDef timer2OC_Config;
	htimer2.Instance         = TIM2;
	htimer2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htimer2.Init.Period      = 0xFFFFFFFF;
	htimer2.Init.Prescaler   = 1;

	if(HAL_TIM_OC_Init(&htimer2) != HAL_OK)
	{
		Error_Handler();
	}

	timer2OC_Config.OCMode      = TIM_OCMODE_TOGGLE;
	timer2OC_Config.OCPolarity  = TIM_OCPOLARITY_HIGH;
	timer2OC_Config.Pulse       = pulse1_val;

	if(HAL_TIM_OC_ConfigChannel(&htimer2, &timer2OC_Config, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	timer2OC_Config.Pulse       = pulse2_val;
	if(HAL_TIM_OC_ConfigChannel(&htimer2, &timer2OC_Config, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}

	timer2OC_Config.Pulse       = pulse3_val;
	if(HAL_TIM_OC_ConfigChannel(&htimer2, &timer2OC_Config, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}

	timer2OC_Config.Pulse       = pulse4_val;
	if(HAL_TIM_OC_ConfigChannel(&htimer2, &timer2OC_Config, TIM_CHANNEL_4) != HAL_OK)
	{
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
}

void UART2_Init(void){
	h_UART2.Instance         = USART2;
	h_UART2.Init.BaudRate    = 115200;
	h_UART2.Init.WordLength  = UART_WORDLENGTH_8B;
	h_UART2.Init.StopBits    = UART_STOPBITS_1;
	h_UART2.Init.Parity      = UART_PARITY_NONE;
	h_UART2.Init.HwFlowCtl   = UART_HWCONTROL_NONE;
	h_UART2.Init.Mode        = UART_MODE_TX_RX;

	if(HAL_UART_Init(&h_UART2) != HAL_OK)
	{	//Problem has been occured
		Error_Handler();
	}
}

//Output Compare callback in non-blocking mode
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	//TIM2_CH1 toggling with freq 500Hz
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		ccr_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, ccr_val + pulse1_val);
	}
	//TIM2_CH2 toggling with freq 1kHz
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		ccr_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, ccr_val + pulse2_val);

	}
	//TIM2_CH3 toggling with freq 2kHz
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		ccr_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, ccr_val + pulse3_val);
	}
	//TIM2_CH4 toggling with freq 4kHz
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		ccr_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, ccr_val + pulse4_val);
	}
}

void Error_Handler(void)
{
	while(1);
}

