/*
 * stm32f411ceu6_i2c_driver.h
 *
 *  Created on: May 25, 2025
 *      Author: bhoomika
 */

#ifndef INC_STM32F411CEU6_I2C_DRIVER_H_
#define INC_STM32F411CEU6_I2C_DRIVER_H_

#include "stm32f411ceu6.h"

typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAdress;
	uint8_t I2C_ACKControl;
	uint8_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer; //stores the application Tx buffer address
	uint8_t *pRxBuffer; //stores the application Rx buffer address
	uint32_t Txlen;     //stores the Tx len
	uint32_t Rxlen;     //stores the Rx len
	uint8_t TxRxState;  //stores the communication state
	uint8_t DeviceAddr;
	uint32_t RxSize;    //stores the Rx size
	uint8_t Sr;         //stores the repeated start value
}I2C_Handle_t;

//@I2C_SCLSpeed
#define I2C_SCL_SPEED_SM     100000
#define I2C_SCL_SPEED_FM4k   400000
#define I2C_SCL_SPEED_SM2k   200000

//@I2C_ACKControl
#define I2C_ACK_EN   1
#define I2C_ACK_DI   0

//@I2C_FMDutyCycle
#define I2C_FM_DUTY_2     0
#define I2C_FM_DUTY_16_9  1

//I2C application states
#define I2C_READY       0
#define I2C_BUSY_IN_RX  1
#define I2C_BUSY_IN_TX  2

//////I2C Application Events Macros/////////
#define I2C_EV_TX_OVER    0
#define I2C_EV_RX_OVER    1
#define I2C_EV_STOP       2
#define I2C_ER_BERR       3
#define I2C_ER_ARLO       4
#define I2C_ER_AF         5
#define I2C_ER_OVR        6
#define I2C_ER_TIMEOUT    7
#define I2C_EV_DATA_REQ   8
#define I2C_EV_DATA_RCV   9

////////////////////////////////APIs supported by this driver//////////////////////////////
//enabling the peripheral clock
void I2C_PeriphCLKCtrl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);

void I2C_GenrateStopCondition(I2C_RegDef_t *pI2Cx);
//Init & DeInit
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

//Data send and receive for MASTER
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t sr);


//Data send and receive for SLAVE
void I2C_SlaveSendData(I2C_Handle_t *pI2CHandle, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_Handle_t *pI2CHandle);

uint8_t I2C_MasterSendData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t sr);
uint8_t I2C_MasterReceiveData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t sr);

//IRQ Configuration & ISR Handling
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t ENorDI);
void I2C_IRQPriority(uint8_t IRQNumber, uint8_t IRQPriority);

void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle);
void I2C_CloseReception(I2C_Handle_t *pI2CHandle);

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

//other peripheral control APIs
void I2C_ManageACKing(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);
void I2C_Peripheral_Control(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flag);
void I2C_SLave_EnDi_callback_events(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);
void I2C_Application_event_callback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);

#endif /* INC_STM32F411CEU6_I2C_DRIVER_H_ */
