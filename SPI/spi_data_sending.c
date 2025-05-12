/*
 * spi_data_sending.c
 *
 *  Created on: Apr 1, 2025
 *      Author: bhoomika
 */
//PB12 = SPI2_NSS
//PB13 = SPI_SCLK
//PB14 = SPI_MISO
//PB15 = SPI_MOSI
#include<string.h>
#include "stm32f411ceu6.h"
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
//    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN112;
//    GPIO_Init(&SPI_Pins);

}

void SPI2_INIT(void){
	SPI_Handle_t SPI2_handle;
	SPI2_handle.pSPIx = SPI2;
	SPI2_handle.SPIConfig.SPI_DeviceMode = SPI_Device_Mode_Master;
	SPI2_handle.SPIConfig.SPI_BusConfig = SPI_BusConfig_FD;
	SPI2_handle.SPIConfig.SPI_SCLKspeed = SPI_SCLK_Speed_Div2;
	SPI2_handle.SPIConfig.SPI_DFF = SPI_DFF_8bits;
	SPI2_handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_handle.SPIConfig.SPI_SSM = SPI_SSM_EN;
	SPI_Init(&SPI2_handle);
}

int main(){
	char user_data[] = "hello world";
	//1. function to initialize GPIO pins to work as SPI2 pins
	SPI2_GPIO_INIT();

	SPI2_INIT();
	//this makes NSS pin to +Vdd
	SPI_SSI_Config(SPI2, ENABLE);
	//enabling the SPI2 peripheral
	SPI_Peripheral_Control(SPI2, ENABLE);

	SPI_SendData(SPI2,(uint8_t*)user_data, strlen(user_data));
	SPI_Peripheral_Control(SPI2, DISABLE);
	while(1);
	return 0;


}

