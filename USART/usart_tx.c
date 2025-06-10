/*
 * usart_tx.c
 *
 *  Created on: Jun 8, 2025
 *      Author: bhoomika
 */

//PA0 => USART2_CTS
//PA1 => USART2_RTS
//PA2 => USART2_TX
//PA3 => USART2_RX
//PA4 => USART2_CK
#include<stdio.h>
#include<string.h>
#include "stm32f411ceu6.h"
USART_Handle_t USART_handle;

char msg[1024] = "UART TX testing...";

void delay(uint32_t dfactor){
	for(uint32_t i = 0; i < dfactor/2; i++);
}

void USART2_GPIO_INIT(void){
	GPIOx_HANDLE_t USART_Pins;
	USART_Pins.pGPIOx                                = GPIOA;
	USART_Pins.GPIO_PinConfig.GPIO_PinMode           = GPIO_MODE_ALTFN;
	USART_Pins.GPIO_PinConfig.GPIO_PinAltFunMode     = 7;
	USART_Pins.GPIO_PinConfig.GPIO_PinOPType         = GPIO_PUSH_PULL_Config;
	USART_Pins.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PU;
	USART_Pins.GPIO_PinConfig.GPIO_PinSpeed          = GPIO_SPEED_FAST;

	//pin configurations so that GPIO pins behave as USART2 pins
//	//serial clock
//	USART_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN4;
//	GPIO_Init(&USART_Pins);
//
//	//USART2_CTS
//	USART_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN0;
//	GPIO_Init(&USART_Pins);
//
//	//USART2_RTS
//	USART_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN1;
//    GPIO_Init(&USART_Pins);

    //USART2_TX
    USART_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN2;
    GPIO_Init(&USART_Pins);

    //USART2_RX
	USART_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN3;
	GPIO_Init(&USART_Pins);

}

void USART2_INIT(void){

	USART_handle.pUSARTx                            = USART2;
	USART_handle.USART_Config.USART_Baudrate        = USART_STD_BAUD_115200;
	USART_handle.USART_Config.USART_ParityControl   = USART_PARITY_DI;
	USART_handle.USART_Config.USART_Stopbits        = USART_STOPBITS_1;
	USART_handle.USART_Config.WordLength            = USART_WORDLEN_8BITS;
	USART_handle.USART_Config.USART_HWFlowControl   = USART_HW_FLOW_CTRL_NONE;
	USART_handle.USART_Config.USART_Mode            = USART_MODE_TX;

	USART_Init(&USART_handle);
}

void GPIOButtonLEDInit(void){
	GPIOx_HANDLE_t GPIO_LED, GPIOBtn;
	GPIOBtn.pGPIOx                              = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PU;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;

	GPIO_LED.pGPIOx                              = GPIOC;
	GPIO_LED.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN13;
	GPIO_LED.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;
	GPIO_LED.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;
	GPIO_LED.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OPEN_DRAIN_Config;
	GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;
	GPIO_Init(&GPIO_LED);

	GPIO_Init(&GPIOBtn);
}



int main(){

	USART2_GPIO_INIT();
	USART2_INIT();
	GPIOButtonLEDInit();
	USART_PeripheralControl(USART_handle.pUSARTx, ENABLE);


	while(1){

		while(GPIO_ReadfromInputPin(GPIOA, GPIO_PIN0));
		delay(500000);
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN13);
		delay(500000);
		USART_SendData(&USART_handle, (uint8_t*)msg, strlen(msg));

	}

	return 0;

}
