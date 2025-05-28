/*
 * i2c_comm.c
 *
 *  Created on: May 28, 2025
 *      Author: bhoomika
 */

#include<stdio.h>
#include<string.h>
#include<stm32f411ceu6.h>

#define my_addr 0x61
#define SlaveAddr 0x68
//PB6 => SCL in AF04
//PB7 => SDA in AF04

I2C_Handle_t I2C1_handle;
uint8_t data[] = "sending data\n";

void delay(uint32_t dfactor){
	for(uint32_t i = 0; i < dfactor/2; i++);
}

void I2C_GPIOInits(void){
	GPIOx_HANDLE_t I2C_Pins;
	I2C_Pins.pGPIOx                                = GPIOB;
	I2C_Pins.GPIO_PinConfig.GPIO_PinMode           = GPIO_MODE_ALTFN;
	I2C_Pins.GPIO_PinConfig.GPIO_PinAltFunMode     = 4;
	I2C_Pins.GPIO_PinConfig.GPIO_PinOPType         = GPIO_OPEN_DRAIN_Config;
	I2C_Pins.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PU;
	I2C_Pins.GPIO_PinConfig.GPIO_PinSpeed          = GPIO_SPEED_FAST;

	//SCL
	I2C_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN6;
	GPIO_Init(&I2C_Pins);
	//SDA
	I2C_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN7;
	GPIO_Init(&I2C_Pins);
}

void I2C1_Inits(void){
	I2C1_handle.I2C_Config.I2C_ACKControl    = I2C_ACK_EN;
	I2C1_handle.I2C_Config.I2C_DeviceAdress  = my_addr;
	I2C1_handle.I2C_Config.I2C_FMDutyCycle   = I2C_FM_DUTY_2;
	I2C1_handle.I2C_Config.I2C_SCLSpeed      = I2C_SCL_SPEED_SM;
	I2C_Init(&I2C1_handle);
}
void GPIOButtonInit(void){
	GPIOx_HANDLE_t GPIOBtn;
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PU;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;

	GPIO_Init(&GPIOBtn);
}


int main(){
	I2C_GPIOInits();
	I2C1_Inits();
	GPIOButtonInit();
	//enable the I2C peripheral
	I2C_Peripheral_Control(I2C1, ENABLE);
	while(1){
	   //wait for button press
	   while(! GPIO_ReadfromInputPin(GPIOA, GPIO_PIN0));
	   delay(500000);
	  //Send the data
	   I2C_MasterSendData(&I2C1_handle, data, strlen((char*)data), SlaveAddr);

	}

}
