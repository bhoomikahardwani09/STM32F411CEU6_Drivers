/*
 * i2c_stm32_slave.c
 *
 *  Created on: Jun 5, 2025
 *      Author: bhoomika
 */

#include<stdio.h>
#include<string.h>
#include<stm32f411ceu6.h>

#define SlaveAddr 0x68
#define my_addr  SlaveAddr

uint8_t tx_data[32] = "STM32 in slave mode..";

I2C_Handle_t I2C1_handle;



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
	GPIOBtn.pGPIOx                              = GPIOA;
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

	//I2C IRQ Configurations
	I2C_IRQConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SLave_EnDi_callback_events(I2C1, ENABLE);

	//enable the I2C peripheral
	I2C_Peripheral_Control(I2C1, ENABLE);
	//ACK bit is set when PE = 1
	I2C_ManageACKing(I2C1, ENABLE);


	while(1){

	}

	return 0;

}

void I2C1_EV_IRQHandler(void){

	I2C_EV_IRQHandling(&I2C1_handle);
}
 void I2C1_ER_IRQHandler(void){

	 I2C_ER_IRQHandling(&I2C1_handle);
}

 void I2C_Application_event_callback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent){

	 static uint8_t count = 0;
	 static uint8_t cmd_code = 0;

	 if(AppEvent == I2C_EV_DATA_REQ){
		 //master request for data and slave needs to transmit the data

		 if(cmd_code == 0x51){
			 //send the length info to the master

			 I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)tx_data));
		 }
		 else if(cmd_code == 0x52){
			 //send the data

			 I2C_SlaveSendData(pI2CHandle->pI2Cx, tx_data[count ++]);
		 }


	 }
	 else if(AppEvent == I2C_EV_DATA_RCV){
		 //master sent some data and slave needs to read it.
		 cmd_code = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);


	 }
	 else if(AppEvent == I2C_ER_AF){
		 // ACK Failure only happens during slave transmission.
		 //Master sends the NACK.

		 cmd_code = 0xff;
		 count = 0;

	 }
	 else if(AppEvent == I2C_EV_STOP){
		 //this only happens at the time of slave reception. Master ends the communication

	 }

 }

