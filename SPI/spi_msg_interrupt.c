/*
 * spi_msg_interrupt.c
 *
 *  Created on: May 21, 2025
 *      Author: bhoomika
 */


#include<stdio.h>
#include<string.h>
#include<stm32f411ceu6.h>

SPI_Handle_t SPI2_handle;
uint8_t read_byte;
volatile uint8_t rcv_stop = 0;
volatile uint8_t data_available = 0; //this flag will be set in the interrupt handler of the arduino interrupt GPIO
#define MAX_LEN 500
char rcvbuffer[MAX_LEN];

void delay(uint32_t dfactor){
	for(uint32_t i = 0; i < dfactor/2; i++);
}

void SPI2_GPIO_INIT(void){
	GPIOx_HANDLE_t SPI_Pins;
	SPI_Pins.pGPIOx                                = GPIOB;
	SPI_Pins.GPIO_PinConfig.GPIO_PinMode           = GPIO_MODE_ALTFN;
	SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode     = 5;
	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType         = GPIO_PUSH_PULL_Config;
	SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_NO_PUPD;
	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed          = GPIO_SPEED_FAST;

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
	SPI2_handle.pSPIx = SPI2;
	SPI2_handle.SPIConfig.SPI_DeviceMode   = SPI_Device_Mode_Master;
	SPI2_handle.SPIConfig.SPI_BusConfig    = SPI_BusConfig_FD;
	SPI2_handle.SPIConfig.SPI_SCLKspeed    = SPI_SCLK_Speed_Div8;
	SPI2_handle.SPIConfig.SPI_DFF          = SPI_DFF_8bits;
	SPI2_handle.SPIConfig.SPI_CPOL         = SPI_CPOL_LOW;
	SPI2_handle.SPIConfig.SPI_CPHA         = SPI_CPHA_LOW;
	SPI2_handle.SPIConfig.SPI_SSM          = SPI_SSM_DI;   //hardware slave management is enabled
	SPI_Init(&SPI2_handle);
}

//this function will configure the GPIO pin over which SPI peripheral issues data available interrupts
void Slave_GPIO_InterruptPinInit(void){
	GPIOx_HANDLE_t SPI_Int;
	memset(&SPI_Int, 0, sizeof(SPI_Int));
	SPI_Int.pGPIOx                               = GPIOD;
	SPI_Int.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_IT_FT;
	SPI_Int.GPIO_PinConfig.GPIO_PinNumber        = GPIO_PIN6;
	SPI_Int.GPIO_PinConfig.GPIO_PinSpeed         = GPIO_SPEED_LOW;
	SPI_Int.GPIO_PinConfig.GPIO_PinPuPdControl   = GPIO_PU;

	GPIO_Init(&SPI_Int);

	GPIO_IRQPriority(IRQ_NO_EXTI9_5, NVIC_IRQ_PR15);
	GPIO_IRQConfig(IRQ_NO_EXTI9_5, ENABLE);

}

int main(){
	uint8_t dummy = 0xff;

	Slave_GPIO_InterruptPinInit();

	SPI2_GPIO_INIT();   //enables the GPIO pins to function as SPI2 pins

	SPI2_INIT();     // initialise the SPI2 peripheral parameters

	//SSOE = 1 => makes the NSS o/p enable and toggle with SPE pin
	// when SPE = 1 , NSS = 0
	// when SPE = 0 , NSS = 1

	SPI_SSOE_Config(SPI2, ENABLE);

	SPI_IRQConfig(IRQ_NO_SPI2, ENABLE);

	while(1){
		rcv_stop = 0;
		while(! data_available);
		GPIO_IRQConfig(IRQ_NO_EXTI9_5, DISABLE); //disabling the interrupt iver this line untill we read the data through SPI slave

		SPI_Peripheral_Control(SPI2, ENABLE);

		while(!rcv_stop){
			//fetch data from the SPI peripheral byte by byte in an interrupt mode
			while(SPI_SendData_IT(&SPI2_handle, &dummy, 1) == SPI_BUSY_IN_TX);
			while(SPI_ReceiveData_IT(&SPI2_handle, &read_byte, 1) == SPI_BUSY_IN_RX);
		}

		//confirm the SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//disable the SPI2 peripheral
		SPI_Peripheral_Control(SPI2, DISABLE);

		data_available = 0;

		GPIO_IRQConfig(IRQ_NO_EXTI9_5, ENABLE);

	}
	return 0;
}

//runs when a data byte is received fro the peripheral over SPI
void SPI2_IRQHandler(void){
	SPI_IRQHandling(&SPI2_handle);
}

void SPI_Application_event_callback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent){

	static uint32_t i = 0;
	// in the RX complete event , copy data into the rcv buffer, '\0' indicates end of msg (rcv_stop = 1)
	if(AppEvent == SPI_EVENT_RX_CMPLT){
		rcvbuffer[i++] = read_byte;
		if(read_byte == '\0' || (i == MAX_LEN)){
			rcv_stop = 1;
			rcvbuffer[i - 1] = '\0';
			i = 0;
		}
	}
 }

//slave data available interrupt handler
void EXTI9_5_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN6);
	data_available = 1;
}
