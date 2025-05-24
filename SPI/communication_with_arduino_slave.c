/*
 * communication_with_arduino_slave.c
 *
 *  Created on: May 14, 2025
 *      Author: bhoomika
 */
#include<string.h>
#include "stm32f411ceu6.h"


void delay(uint32_t dfactor){
	for(uint32_t i = 0; i < dfactor/2; i++);
}
void SPI2_GPIO_INIT(void){
	GPIOx_HANDLE_t SPI_Pins;
	SPI_Pins.pGPIOx = GPIOB;
	SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_PUSH_PULL_Config;
	SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//pin configurations so that GPIO pins behave as SPI2 pins
	//serial clock
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN13;
	GPIO_Init(&SPI_Pins);

	//MOSI
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN15;
	GPIO_Init(&SPI_Pins);

	//MISO
//	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN14;
//    GPIO_Init(&SPI_Pins);

    //NSS
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN12;
    GPIO_Init(&SPI_Pins);

}

void SPI2_INIT(void){
	SPI_Handle_t SPI2_handle;
	SPI2_handle.pSPIx = SPI2;
	SPI2_handle.SPIConfig.SPI_DeviceMode = SPI_Device_Mode_Master;
	SPI2_handle.SPIConfig.SPI_BusConfig = SPI_BusConfig_FD;
	SPI2_handle.SPIConfig.SPI_SCLKspeed = SPI_SCLK_Speed_Div8;
	SPI2_handle.SPIConfig.SPI_DFF = SPI_DFF_8bits;
	SPI2_handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_handle.SPIConfig.SPI_SSM = SPI_SSM_DI;//hardware slave management is enabled
	SPI_Init(&SPI2_handle);
}

void GPIOButtonInit(void){
	GPIOx_HANDLE_t GPIOBtn;
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&GPIOBtn);
}
int main(){
	char user_data[] = "hello world";
	GPIOButtonInit();
	//1. function to initialize GPIO pins to work as SPI2 pins
	SPI2_GPIO_INIT();

	SPI2_INIT();

	//enabling the Slave Select O/P Enabled
	SPI_SSOE_Config(SPI2, ENABLE);

	while(1){
//		while(! GPIO_ReadfromInputPin(GPIOA, GPIO_PIN0) );
//		delay(500000);
		//enabling the SPI2 peripheral
		SPI_Peripheral_Control(SPI2, ENABLE);
		//to send the bits of data the slave is going to receive
		uint8_t data_len = strlen(user_data);
		SPI_SendData(SPI2, &data_len, 1);

		SPI_SendData(SPI2,(uint8_t*)user_data, strlen(user_data));
		//to confirm that SPI is not busy before disabling the peripheral
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
		SPI_Peripheral_Control(SPI2, DISABLE);

	}
	return 0;


}

