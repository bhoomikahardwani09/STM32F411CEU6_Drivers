/*
 * 001LED_Toggle.c
 *
 *  Created on: Dec 24, 2024
 *      Author: bhoomika
 */
#include "stm32f411ceu6.h"
#include "stm32f411ceu6_gpio_driver.h"

void delay(void){
	for(uint32_t i = 0; i< 5000000/2; i++);
}

int main(void){
	GPIOx_HANDLE_t GPIO_LED;
	GPIO_LED.pGPIOx                              = GPIOC;
	GPIO_LED.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN13;
	GPIO_LED.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;
	GPIO_LED.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;
	GPIO_LED.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OPEN_DRAIN_Config;
	GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PU;

	GPIO_PeriphCLKCtrl(GPIOC, ENABLE);
	GPIO_Init(&GPIO_LED);
	while(1){
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN13);
		delay();
	}
	return 0;

}
