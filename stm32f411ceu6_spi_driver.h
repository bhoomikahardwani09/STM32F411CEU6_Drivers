/*
 * stm32f411ceu6_spi_driver.h
 *
 *  Created on: Apr 1, 2025
 *      Author: bhoomika
 */

#ifndef INC_STM32F411CEU6_SPI_DRIVER_H_
#define INC_STM32F411CEU6_SPI_DRIVER_H_

#include "stm32f411ceu6.h"

//Configuration structure for SPIx peripheral
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SCLKspeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

//Handle Structure for SPIx peripheral
typedef struct{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t  *pTxBuffer;
	uint8_t  *pRxBuffer;
	uint8_t   TxLen;
	uint8_t   RxLen;
	uint8_t   TxState;
	uint8_t   RxState;
}SPI_Handle_t;

//device modes of SPI
#define SPI_Device_Mode_Master   1
#define SPI_Device_Mode_Slave    0
//bus configurations for SPI
#define SPI_BusConfig_FD         1
#define SPI_BusConfig_HD         2
#define SPI_BusConfig_SimplexRX  3
//SPI_CLKSpeed(BR[2:0] : we'll configure this prescalar)
#define SPI_SCLK_Speed_Div2      0
#define SPI_SCLK_Speed_Div4      1
#define SPI_SCLK_Speed_Div8      2
#define SPI_SCLK_Speed_Div16     3
#define SPI_SCLK_Speed_Div32     4
#define SPI_SCLK_Speed_Div64     5
#define SPI_SCLK_Speed_Div128    6
#define SPI_SCLK_Speed_Div256    7
//SPI_DFF
#define SPI_DFF_8bits  0   //(by default)
#define SPI_DFF_16bits 1
//CPOL & CPHA
#define SPI_CPOL_HIGH  1
#define SPI_CPOL_LOW   0

#define SPI_CPHA_HIGH  1
#define SPI_CPHA_LOW   0

//SPI_SSM
#define SPI_SSM_EN  1
#define SPI_SSM_DI  0

//SPI Status register flags
//Masking details
#define SPI_TX_FLAG    (1 << 1)
#define SPI_RXNE_FLAG  (1 << 0)
#define SPI_BUSY_FLAG  (1 << 7)

//SPI Application States
#define SPI_READY        0
#define SPI_BUSY_IN_RX   1
#define SPI_BUSY_IN_TX   2

//possible SPI application events
#define SPI_EVENT_TX_CMPLT  1
#define SPI_EVENT_RX_CMPLT  2
#define SPI_EVENT_OVR_CMPLT 3

////////////////////////////////APIs supported by this driver//////////////////////////////
//enabling the peripheral clock
void SPI_PeriphCLKCtrl(SPI_RegDef_t *pSPIx, uint8_t ENorDI);

//Init & DeInit
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//Data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

//IRQ Configuration & ISR Handling
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t ENorDI);
void SPI_IRQPriority(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);   //INTERRUPT FOR A PARTICULAR PIN NUMBER WILL BE MANAGED

//other peripheral control APIs
void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t ENorDI);
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDI);
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDI);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

void SPI_Application_event_callback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);
#endif /* INC_STM32F411CEU6_SPI_DRIVER_H_ */
