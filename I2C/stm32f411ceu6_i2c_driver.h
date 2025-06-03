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


////////////////////////////////APIs supported by this driver//////////////////////////////
//enabling the peripheral clock
void I2C_PeriphCLKCtrl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);

//Init & DeInit
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

//Data send and receive
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t sr);

uint8_t I2C_MasterSendData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t sr);
uint8_t I2C_MasterReceiveData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t sr);

//IRQ Configuration & ISR Handling
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t ENorDI);
void I2C_IRQPriority(uint8_t IRQNumber, uint8_t IRQPriority);


//other peripheral control APIs
void I2C_ManageACKing(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);
void I2C_Peripheral_Control(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flag);

void I2C_Application_event_callback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);


#endif /* INC_STM32F411CEU6_I2C_DRIVER_H_ */
