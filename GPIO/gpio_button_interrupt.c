/*
 * gpio_button_interrupt.c
 *
 *  Created on: May 23, 2025
 *      Author: bhoomika
 */


#include "stm32f411ceu6.h"
#include "stm32f411ceu6_gpio_driver.h"

void delay(void){
	for(uint32_t i = 0; i< 100000; i++);
}

int main(void){
	GPIOx_HANDLE_t GPIO_LED, GPIO_BTN;
//	memset(&GPIO_LED, 0, sizeof(GPIO_LED));
	GPIO_LED.pGPIOx                              = GPIOC;
	GPIO_LED.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN13;
	GPIO_LED.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;
	GPIO_LED.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;
	GPIO_LED.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OPEN_DRAIN_Config;
	GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;

	GPIO_BTN.pGPIOx                              = GPIOD;
	GPIO_BTN.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN5;
	GPIO_BTN.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IT_FT;
	GPIO_BTN.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;
	GPIO_BTN.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PU;

	GPIO_Init(&GPIO_LED);
	GPIO_Init(&GPIO_BTN);

	//IRQ configuration
	GPIO_IRQPriority(IRQ_NO_EXTI9_5, NVIC_IRQ_PR15);
	GPIO_IRQConfig(IRQ_NO_EXTI9_5, ENABLE);

	void EXTI9_5_IRQHandler(void){
    delay();
	GPIO_IRQHandling(GPIO_PIN5);
	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN13);
	}

	return 0;

}
