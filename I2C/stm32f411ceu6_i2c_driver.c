/*
 * stm32f411ceu6_i2c_driver.c
 *
 *  Created on: May 25, 2025
 *      Author: bhoomika
 */
#include "stm32f411ceu6.h"
#include "stm32f411ceu6_i2c_driver.h"

static void I2C_GenrateStartCondition(I2C_RegDef_t *pI2Cx);


void I2C_PeriphCLKCtrl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI){
	if(ENorDI == ENABLE){
			if(pI2Cx == I2C1)          I2C1_PCLK_EN();
			else if(pI2Cx == I2C2)     I2C2_PCLK_EN();
			else if(pI2Cx == I2C3)     I2C3_PCLK_EN();
		}
		else{
			if(pI2Cx == I2C1)          I2C1_PCLK_DI();
	        else if(pI2Cx == I2C2)     I2C2_PCLK_DI();
	        else if(pI2Cx == I2C3)     I2C3_PCLK_DI();
		}

}
uint32_t RCC_GetPLLOutputClock(void){
	return;
}
uint8_t AHB_PreScalar[8]  = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScalar[4] = {2, 4, 8, 16};

uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1, sysclk;
	uint8_t clksrc, temp, ahbps, apb1ps;
	clksrc = ( (RCC->CFGR >> 2) & 0x3 );
	if(clksrc == 0) sysclk = 16000000;  //HSE
	else if(clksrc == 1) sysclk = 8000000;  //HSI
	else if(clksrc == 2) sysclk = RCC_GetPLLOutputClock();  //PLL

	temp = ( (RCC->CFGR >> 4) & 0xf );
	if(temp < 8) ahbps = 1;
	else{
		ahbps = AHB_PreScalar[temp - 8];
	}
	temp = ( (RCC->CFGR >> 10) & 0x7 );
	if(temp < 4) apb1ps = 1;
	else{
		apb1ps = APB1_PreScalar[temp - 4];
	}

	pclk1 = (sysclk / ahbps) / apb1ps;
	return pclk1;
}
void I2C_Init(I2C_Handle_t *pI2CHandle){
	//enabling the clock
	I2C_PeriphCLKCtrl(pI2CHandle->pI2Cx, ENABLE);
	uint32_t temp_reg = 0;
	//enable the ACKing
	temp_reg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;

	//configure the FREQ in CR2
	temp_reg = 0;
	temp_reg |= (RCC_GetPCLK1Value() / 1000000U);
	pI2CHandle->pI2Cx->I2C_CR2 = (temp_reg & 0x3F);

	//storing the device's own address
	temp_reg |= pI2CHandle->I2C_Config.I2C_DeviceAdress << 1;
	pI2CHandle->pI2Cx->I2C_OAR1 = temp_reg;

	//Bit 14 of I2C_OAR1 => Should always be kept at 1 by software
	pI2CHandle->pI2Cx->I2C_OAR1 = (1 << 14);

	//CCR Calculations
	uint16_t ccr_value = 0;
	temp_reg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//mode = SM
		ccr_value = ( RCC_GetPCLK1Value() /(2 * pI2CHandle->I2C_Config.I2C_SCLSpeed) );
		temp_reg |= (ccr_value & 0xFFF);

	}
	else{
		//mode = FM
		//bit15 = 1 for FM
		temp_reg |= (1 << 15);
		temp_reg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = ( RCC_GetPCLK1Value() /(3 * pI2CHandle->I2C_Config.I2C_SCLSpeed) );
		}
		else{
			ccr_value = ( RCC_GetPCLK1Value() /(25 * pI2CHandle->I2C_Config.I2C_SCLSpeed) );
		}
		temp_reg |= (ccr_value & 0xFFF);

	}
	pI2CHandle->pI2Cx->I2C_CCR = temp_reg;

	//TRISE calculation
	temp_reg = 0;;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		temp_reg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else{
		temp_reg = ( (RCC_GetPCLK1Value() * 300)/ 1000000000U ) + 1;
	}
	pI2CHandle->pI2Cx->I2C_TRISE = (temp_reg & 0x3F);

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1)      I2C1_RST();
	else if(pI2Cx == I2C2) I2C2_RST();
	else if(pI2Cx == I2C3) I2C3_RST();
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flag){
	if(pI2Cx->I2C_SR1 & flag) return FLAG_SET;
	return FLAG_RESET;
}
static void I2C_GenrateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr){
	//1. generate the START cond.n
	I2C_GenrateStartCondition(pI2CHandle->pI2Cx); //helper function
	//2. confirm that the start cond.n is completed by checking the SB flag in SR1
	// Until the SB is cleared SCL will be stretched
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, (1 << I2C_SR1_SB)) );

	//3. send the address of the slave with r/nw bit = 0
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2CHandle->pI2Cx->I2C_DR = SlaveAddr;

	//4. confirm that the address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, (1 << I2C_SR1_ADDR)) );
	//5. we need to clear the ADDR flag by first reading the SR1 and the SR2
	uint32_t dummyread = pI2CHandle->pI2Cx->I2C_SR1;
	dummyread = pI2CHandle->pI2Cx->I2C_SR2;
	(void)dummyread;

	//6. semd data until the length becomes 0
	while(len > 0){
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, (1 << I2C_SR1_TXE)) ); //waiting until TxE is set
		pI2CHandle->pI2Cx->I2C_DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}
	//7. when the len == 0, wait for the TxE = 1 and BTF = 1 before generating the stop cond.n
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, (1 << I2C_SR1_TXE)) ); //waiting until TxE is set
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, (1 << I2C_SR1_BTF)) ); //waiting until BTF is set

	//8. generating the STOP cond.n
	pI2CHandle->pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);

}

void I2C_Peripheral_Control(I2C_RegDef_t *pI2Cx, uint8_t ENorDI){
	if(ENorDI == ENABLE) pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
	else pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
}

//IRQ Configuration & ISR Handling
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t ENorDI){
	if(ENorDI == ENABLE){
		if(IRQNumber <= 31){
			//program ISER0
			*NVIC_ISER0 |= ( 1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 63){
			//program ISER1
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber > 63 && IRQNumber < 96){
			//ISER2
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
	else{
		if(IRQNumber <= 31){
			//program ISER0
			*NVIC_ICER0 |= ( 1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 63){
			//program ISER1
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber > 63 && IRQNumber < 96){
			//ISER2
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}


void I2C_IRQPriority(uint8_t IRQNumber, uint8_t IRQPriority){
	//1.IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_section = (IRQNumber % 4);
	uint8_t shift_parameter = (IPRx_section * 8) + ( 8 - NO_OF_PR_BITS_IMPLEMENTED);
	*( NVIC_PR_BASEADDR + (IPRx * 4) ) |= (IRQPriority << shift_parameter);
}
