/*
 * main.c
 *
 *  Created on: June 23, 2025
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

void LSE_Configuration(void);

uint32_t input_capture[2] = {0};
uint8_t count = 1;
uint8_t is_capture_done = FALSE;

TIM_HandleTypeDef htimer2;
UART_HandleTypeDef h_UART2;

int main(void)
{
	uint32_t cap_diff = 0;
	double timer2_cnt_freq = 0, timer2_cnt_res = 0, user_signal_time_period = 0, user_signal_freq = 0;
	char msg[100];

	HAL_Init();

	SystemClockConfig(SYS_CLK_FREQ_50MHZ);

	GPIO_Init();

	TIMER2_Init();

	UART2_Init();

	LSE_Configuration();

	if(HAL_TIM_IC_Start_IT(&htimer2, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	while(1)
	{
		if(is_capture_done)
		{
			cap_diff = (input_capture[1] > input_capture[0])?
					    (input_capture[1] - input_capture[0]):
						( 0xFFFFFFFF - input_capture[0] + input_capture[1] );
		}
		timer2_cnt_freq         = (HAL_RCC_GetPCLK1Freq() * 1) / (htimer2.Init.Prescaler + 1);
		timer2_cnt_res          = 1.0 / timer2_cnt_freq;
		user_signal_time_period = cap_diff * timer2_cnt_res;
		user_signal_freq        = (1.0 / user_signal_time_period);
		uint32_t i_p = (uint32_t)(user_signal_freq);
		uint32_t f_p = (uint32_t)((user_signal_freq - i_p)*1000);

		sprintf(msg, "cap1=%lu cap2=%lu diff=%lu\r\n", input_capture[0], input_capture[1], cap_diff);
		HAL_UART_Transmit(&h_UART2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		sprintf(msg, "frequency of signal = %d.%03d\r\n",i_p,f_p);
		HAL_UART_Transmit(&h_UART2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	}

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
	TIM_IC_InitTypeDef timer2IC_Config;
	htimer2.Instance         = TIM2;
	htimer2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htimer2.Init.Period      = 0xFFFFFFFF;
	htimer2.Init.Prescaler   = 1;

	if( HAL_TIM_IC_Init(&htimer2) != HAL_OK)
	{
		Error_Handler();
	}

	timer2IC_Config.ICPolarity  = TIM_ICPOLARITY_RISING;
	timer2IC_Config.ICFilter    = 0;
	timer2IC_Config.ICPrescaler = TIM_ICPSC_DIV1;
	timer2IC_Config.ICSelection = TIM_ICSELECTION_DIRECTTI;

	if(HAL_TIM_IC_ConfigChannel(&htimer2, &timer2IC_Config, TIM_CHANNEL_1) != HAL_OK)
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


void LSE_Configuration(void)
{
#if 0
	RCC_OscInitTypeDef osc_init;
	osc_init.OscillatorType       = RCC_OSCILLATORTYPE_LSE;
	osc_init.LSEState             = RCC_LSE_ON;
	if(HAL_RCC_OscConfig(&osc_init) != HAL_OK)
	{
		Error_Handler();
	}
#endif
//	Selects the clock source to output on MCO1 pin(PA8) or on MCO2 pin(PC9).
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_LSE, RCC_MCODIV_1);
}

//@brief  Input Capture callback in non-blocking mode
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(!is_capture_done){
		if(count == 1)
		{
			input_capture[0] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
			count++;
		}
		else if(count == 2)
		{
			input_capture[1] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
			count = 1;
			is_capture_done = TRUE;
		}

	}

}
void Error_Handler(void)
{
	while(1);
}

