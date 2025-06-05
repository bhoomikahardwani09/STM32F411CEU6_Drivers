/*
 * i2c_master_receive_interrupt.c
 *
 *  Created on: Jun 4, 2025
 *      Author: bhoomika
 */

#include<stdio.h>
#include<string.h>
#include<stm32f411ceu6.h>

#define my_addr   0x61
#define SlaveAddr 0x68
uint8_t rcv_data[32];

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

	//I2C IRQ configurations
	I2C_IRQConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQConfig(IRQ_NO_I2C1_ER, ENABLE);

	//enable the I2C peripheral
	I2C_Peripheral_Control(I2C1, ENABLE);
	//ACK bit is set when PE = 1
	I2C_ManageACKing(I2C1, ENABLE);

	uint8_t cmd_code, len;

	while(1){
		//wait until the button is pressed
		while(! GPIO_ReadfromInputPin(GPIOA, GPIO_PIN0) );
		delay(500000);

		cmd_code = 0x51; //code to read the length info from the transmitter

		while( I2C_MasterSendData_IT(&I2C1_handle, &cmd_code, 1, SlaveAddr, 1) != I2C_READY );

		while( I2C_MasterReceiveData_IT(&I2C1_handle, &len, 1, SlaveAddr, 1) != I2C_READY );

		cmd_code = 0x52; //code to read the data

		while( I2C_MasterSendData_IT(&I2C1_handle, &cmd_code, 1, SlaveAddr, 1) != I2C_READY );

		while( I2C_MasterReceiveData_IT(&I2C1_handle, rcv_data, len, SlaveAddr, 0) != I2C_READY );

		rcv_data[len + 1] = '0\n';
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

	 if(AppEvent == I2C_EV_TX_OVER){

	 }
	 else if(AppEvent == I2C_EV_RX_OVER){

	 }
	 if(AppEvent == I2C_ER_AF){
		 I2C_CloseTransmission(pI2CHandle);
		 //generate the stop condition to release the bus
		 I2C_GenrateStopCondition(I2C1);
		 //hanging the application
		 while(1);

	 }

 }











