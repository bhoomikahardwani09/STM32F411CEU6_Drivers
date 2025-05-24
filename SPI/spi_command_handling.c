/*
 * spi_command_handling.c
 *
 *  Created on: May 16, 2025
 *      Author: bhoomika
 */

#include<string.h>
#include "stm32f411ceu6.h"

#define CMD_LED_CTRL     0x50
#define CMD_SENSOR_READ  0x51
#define CMD_LED_READ     0x52
#define CMD_PRINT        0x53
#define CMD_ID_READ      0x54

#define LED_ON              1
#define LED_OFF             0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4
#define LED           9

void delay(uint32_t dfactor){
	for(uint32_t i = 0; i < dfactor/2; i++);
}
void SPI2_GPIO_INIT(void){
	GPIOx_HANDLE_t SPI_Pins;
	SPI_Pins.pGPIOx                                 = GPIOB;
	SPI_Pins.GPIO_PinConfig.GPIO_PinMode            = GPIO_MODE_ALTFN;
	SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode      = 5;
	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType          = GPIO_PUSH_PULL_Config;
	SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl     = GPIO_NO_PUPD;
	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed           = GPIO_SPEED_FAST;

	//pin configurations so that GPIO pins behave as SPI2 pins
	//serial clock
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN13;
	GPIO_Init(&SPI_Pins);

	//MOSI
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN15;
	GPIO_Init(&SPI_Pins);

	//MISO
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN14;
    GPIO_Init(&SPI_Pins);

    //NSS
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN12;
    GPIO_Init(&SPI_Pins);

}

void SPI2_INIT(void){
	SPI_Handle_t SPI2_handle;
	SPI2_handle.pSPIx                         = SPI2;
	SPI2_handle.SPIConfig.SPI_DeviceMode      = SPI_Device_Mode_Master;
	SPI2_handle.SPIConfig.SPI_BusConfig       = SPI_BusConfig_FD;
	SPI2_handle.SPIConfig.SPI_SCLKspeed       = SPI_SCLK_Speed_Div8;
	SPI2_handle.SPIConfig.SPI_DFF             = SPI_DFF_8bits;
	SPI2_handle.SPIConfig.SPI_CPOL            = SPI_CPOL_LOW;
	SPI2_handle.SPIConfig.SPI_CPHA            = SPI_CPHA_LOW;
	SPI2_handle.SPIConfig.SPI_SSM             = SPI_SSM_DI;//hardware slave management is enabled
	SPI_Init(&SPI2_handle);
}

void GPIOButtonInit(void){
	GPIOx_HANDLE_t GPIOBtn;
	GPIOBtn.pGPIOx                                  = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode             = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber           = GPIO_PIN0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl      = GPIO_PU;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed            = GPIO_SPEED_FAST;

	GPIO_Init(&GPIOBtn);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if(ackbyte == 0xF5) return 1;
	return 0;
}

int main(){
//	char user_data[] = "hello world";
	uint8_t dummy_byte = 0xff;
	uint8_t dummy_read;
	GPIOButtonInit();
	//1. function to initialize GPIO pins to work as SPI2 pins
	SPI2_GPIO_INIT();

	SPI2_INIT();

	//enabling the Slave Select O/P Enabled
	SPI_SSOE_Config(SPI2, ENABLE);

	while(1){
		//////////////1.CMD_LED_CTRL///////////////////
		while(! GPIO_ReadfromInputPin(GPIOA, GPIO_PIN0) );//button is pressed
		delay(500000);
		//enabling the SPI2 peripheral
		SPI_Peripheral_Control(SPI2, ENABLE);

		//sending the 1st cmd :- CMD_LED_CTRL
		uint8_t cmd_code = CMD_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		SPI_SendData(SPI2, &cmd_code, 1);

		//do the dummy read to clear the RXNE flag as when master sends the data it also receives
		//so the RXNE flag is set(not empty)
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		//sending dummy_byte of 1 byte
		SPI_SendData(SPI2, &dummy_byte, 1);

		//read the ACK byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte)){     //if ackbyte == 1 then we'll send the arguments
			args[0] = LED;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);
		}

		//////////2. CMD_SENSOR_READ = reads the analog value of supplied pin////////

		while(! GPIO_ReadfromInputPin(GPIOA, GPIO_PIN0) );  //button is pressed
		delay(500000);

		cmd_code = CMD_SENSOR_READ;
		SPI_SendData(SPI2, &cmd_code, 1);   //send command
		//do the dummy read to clear the RXNE flag as when master sends the data it also receives
		//so the RXNE flag is set(not empty)
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		//sending dummy_byte of 1 byte
		SPI_SendData(SPI2, &dummy_byte, 1);

		//read the ACK byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		if(SPI_VerifyResponse(ackbyte)){     //if ackbyte == 1 then we'll send the arguments
			args[0] = ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI2, args, 1);

			//dummy read to clear off the RXNE flag
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			delay(500000);
			//send some dummy byte to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_byte, 1);

			//slave takes some time to read the analog value(it does ADC conversion on that pin), so master should
			//wait sometime before generating the dummy bit to fetch the result

			uint8_t analog_read_data;
			SPI_ReceiveData(SPI2, &analog_read_data, 1);

		}
		//////////////3. CMD_LED_READ //////////
		while(! GPIO_ReadfromInputPin(GPIOA, GPIO_PIN0) );//button is pressed
		delay(500000);

		cmd_code = CMD_LED_READ;
		SPI_SendData(SPI2, &cmd_code, 1);//sending the comand

		//do the dummy read to clear the RXNE flag as when master sends the data it also receives
		//so the RXNE flag is set(not empty)
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//sending dummy byte to receive response from the slave
		SPI_SendData(SPI2, &dummy_byte, 1);

		//read the ACK byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte)){
			args[0] = LED;

			//sending args
			SPI_SendData(SPI2, args, 1);

			//dummy read to clear off the RXNE flag
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//send some dummy byte to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_byte, 1);

			uint8_t digital_led_read;
			SPI_ReceiveData(SPI2, &digital_led_read, 1);
		}

		//////////////4.CMD_PRINT//////////////////
		while(! GPIO_ReadfromInputPin(GPIOA, GPIO_PIN0) );//button is pressed
		delay(500000);

		cmd_code = CMD_PRINT;
		SPI_SendData(SPI2, &cmd_code, 1);//sending the comand

		//do the dummy read to clear the RXNE flag as when master sends the data it also receives
		//so the RXNE flag is set(not empty)
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//sending dummy byte to receive response from the slave
		SPI_SendData(SPI2, &dummy_byte, 1);

		//read the ACK byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		if(SPI_VerifyResponse(ackbyte)){
			uint8_t msg[] = "heyy";
			args[0] = strlen((char*)msg);

			//sending the length
			SPI_SendData(SPI2, args, 1);

			//dummy read to clear off the RXNE flag
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			delay(500000);

			//sending the msg
			for(int i = 0; i < args[0]; i++){
				SPI_SendData(SPI2, &msg[i], 1);
				SPI_ReceiveData(SPI2, &dummy_read, 1);
			}

		}

		////////////5. CMD_ID_READ///////////
		while(! GPIO_ReadfromInputPin(GPIOA, GPIO_PIN0) );//button is pressed
		delay(500000);

		cmd_code = CMD_ID_READ;
		SPI_SendData(SPI2, &cmd_code, 1);//sending the comand

		//do the dummy read to clear the RXNE flag as when master sends the data it also receives
		//so the RXNE flag is set(not empty)
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//sending dummy byte to receive response from the slave
		SPI_SendData(SPI2, &dummy_byte, 1);

		//read the ACK byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t id[10];
		uint32_t i = 0;
		if(SPI_VerifyResponse(ackbyte)){

			//read 10 bytes id from the slave
			for(i = 0; i < 10; i++){
				//sending dummy bytes to fetch data from the slave
				SPI_SendData(SPI2, &dummy_byte, 1);
				SPI_ReceiveData(SPI2, &id[i], 1);
			}
			id[10] = '\0';

		}

		//to confirm that SPI is not busy before disabling the peripheral
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
		SPI_Peripheral_Control(SPI2, DISABLE);

	}
	return 0;

}


