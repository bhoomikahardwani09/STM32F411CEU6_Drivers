/*
 * stm32f411ceu6_i2c_driver.c
 *
 *  Created on: May 25, 2025
 *      Author: bhoomika
 */
#include "stm32f411ceu6.h"
#include "stm32f411ceu6_i2c_driver.h"

static void I2C_GenrateStartCondition(I2C_RegDef_t *pI2Cx);
//static void I2C_GenrateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_GenrateStartCondition(I2C_RegDef_t *pI2Cx){

	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);

}

void I2C_GenrateStopCondition(I2C_RegDef_t *pI2Cx){

	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);

}


static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				I2C_ManageACKing(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
				dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
				(void)dummy_read;
			}

		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
			dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
		dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
		(void)dummy_read;
	}


}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){
		//data reception
		if(pI2CHandle->Rxlen == 1){

			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
			pI2CHandle->Rxlen --;

		}
		if(pI2CHandle->Rxlen > 1){

			if(pI2CHandle->Rxlen == 2){
				I2C_ManageACKing(pI2CHandle->pI2Cx, DISABLE); //clear the ACK bit
			}

			// read the DR
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
			pI2CHandle->Rxlen --;
			pI2CHandle->pRxBuffer ++;

		}

		if(pI2CHandle->Rxlen == 0){

			if(pI2CHandle->Sr == 0){
				I2C_GenrateStopCondition(pI2CHandle->pI2Cx);
			}

			I2C_CloseReception(pI2CHandle);

			I2C_Application_event_callback(pI2CHandle, I2C_EV_RX_OVER);
		}

}


static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->Txlen > 0){

		pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->pTxBuffer ++;
		pI2CHandle->Txlen --;
	}
}



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



void I2C_ManageACKing(I2C_RegDef_t *pI2Cx, uint8_t ENorDI){
	if(ENorDI == ENABLE) pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
	else pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
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

void I2C_Peripheral_Control(I2C_RegDef_t *pI2Cx, uint8_t ENorDI){
	if(ENorDI == ENABLE) pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
	else pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flag){
	if(pI2Cx->I2C_SR1 & flag) return FLAG_SET;
	return FLAG_RESET;
}



void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t sr){
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
	I2C_ClearADDRFlag(pI2CHandle);
	//6. send data until the length becomes 0
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
	if(sr == 0){
		pI2CHandle->pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
	}

}


void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t sr){
	//generate the START cond.n
	I2C_GenrateStartCondition(pI2CHandle->pI2Cx); //helper function
	//2. confirm that the start cond.n is completed by checking the SB flag in SR1
	// Until the SB is cleared SCL will be stretched
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, (1 << I2C_SR1_SB)) );

	//3. send the address of the slave with r/nw = 1
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 0x01;
	pI2CHandle->pI2Cx->I2C_DR = SlaveAddr;
	//4. confirm that the address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, (1 << I2C_SR1_ADDR)) );

	//TO READ ONLY 1 BYTE OF DATA
	if(len == 1){
		//Disable the ACKing
//		I2C_ManageACKing(pI2CHandle->pI2Cx, DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
		//waithing until the RxNE is set
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, (1 << I2C_SR1_RXNE)) );

		//generate the STOP cond.n
		if(sr == 0){
			pI2CHandle->pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
		}


		//read the data from the DR
		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
	}
	else if(len > 1){

		I2C_ClearADDRFlag(pI2CHandle);

		//read data until len becomes 0
		for(uint32_t i = len; i > 0; i--){
			//wait until RxNE is set
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, (1 << I2C_SR1_RXNE)) );

			//read data from the buffer
			*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
			pRxBuffer++;
			if(i == 2){
				//Disable the ACKing
				I2C_ManageACKing(pI2CHandle->pI2Cx, DISABLE);
				//generate the STOP cond.n
				if(sr == 0){
					pI2CHandle->pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
				}
			}
		}

	}
	//re-enable the ACKing if
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_EN){
		I2C_ManageACKing(pI2CHandle->pI2Cx, ENABLE);
	}

}


void I2C_SlaveSendData(I2C_Handle_t *pI2CHandle, uint8_t data){

	pI2CHandle->pI2Cx->I2C_DR = data;

}

uint8_t I2C_SlaveReceiveData(I2C_Handle_t *pI2CHandle){

	return (uint8_t) pI2CHandle->pI2Cx->I2C_DR;
}


uint8_t I2C_MasterSendData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t sr){
	uint8_t busy_state = pI2CHandle->TxRxState;

	if( (busy_state != I2C_BUSY_IN_TX) && (busy_state != I2C_BUSY_IN_RX) ){
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->Txlen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DeviceAddr = SlaveAddr;
		pI2CHandle->Sr = sr;

		//generate the start condition
		I2C_GenrateStartCondition(pI2CHandle->pI2Cx);

		//enable the ITBUFEN control bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);
		//enable the ITEVFEN control bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);
		//enable the ITERREN control bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busy_state;
}


uint8_t I2C_MasterReceiveData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t sr){
	uint8_t busy_state = pI2CHandle->TxRxState;

	if( (busy_state != I2C_BUSY_IN_TX) && (busy_state != I2C_BUSY_IN_RX) ){
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->Rxlen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DeviceAddr = SlaveAddr;
		pI2CHandle->RxSize = len;//Rxsize is used in the ISR code to manage the data reception 
		pI2CHandle->Sr = sr;

		//generate the start condition
		I2C_GenrateStartCondition(pI2CHandle->pI2Cx);

		//enable the ITBUFEN control bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);
		//enable the ITEVFEN control bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);
		//enable the ITERREN control bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busy_state;

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


void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle){

	//Disable the ITBUFEN Control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	//Disable the ITEVFEN control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->Txlen = 0;

}

void I2C_CloseReception(I2C_Handle_t *pI2CHandle){

	//Disable the ITBUFEN Control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	//Disable the ITEVFEN control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->Rxlen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_EN){

		I2C_ManageACKing(pI2CHandle->pI2Cx, ENABLE);

	}


}


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	//interrupt handling for both master and slave devices

	uint8_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_SB);

	//////////////////1. handle for interrupt generated by SB event( SB flag is only applicable in master mode)////////////////////////
	if(temp1 && temp3){
		//SB flag is set

		//execute the next address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){

			pI2CHandle->DeviceAddr = pI2CHandle->DeviceAddr << 1;
			pI2CHandle->DeviceAddr &= ~(1);
			pI2CHandle->pI2Cx->I2C_DR = pI2CHandle->DeviceAddr;
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){

			pI2CHandle->DeviceAddr = pI2CHandle->DeviceAddr << 1;
			pI2CHandle->DeviceAddr |= 0x01;
			pI2CHandle->pI2Cx->I2C_DR = pI2CHandle->DeviceAddr;
		}
	}

	///////////////////////2. handle for interrupt generated by ADDR event////////////////////
	//in master mode :- address is sent
	//in slave mode  :- address matched with own address
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3){

		I2C_ClearADDRFlag(pI2CHandle);

	}

    ///////////////////////3. handle for interrupt generated by BTF event/////////////////////
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3){

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){

			if( pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TXE) ){

				//1.now BTF and TxE are set. So generate the STOP condition
				if(pI2CHandle->Txlen == 0){
					if(pI2CHandle->Sr == 0) I2C_GenrateStopCondition(pI2CHandle->pI2Cx);

					//2.reset all the member elements of the handle structure
					I2C_CloseTransmission(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_Application_event_callback(pI2CHandle, I2C_EV_TX_OVER);
				}

			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			;

		}

	}

	//4.handle for interrupt generated by STOPF event mode. In master
	//stop flag is only applicable for slave. For master it will never be set
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_STOPF);
	if(temp1 && temp3){// STOPF is set

		//Clearing it => read SR1 then writing to CR1
		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000;

		I2C_Application_event_callback(pI2CHandle, I2C_EV_STOP);

	}

	//5.handle for interrupt generated by TxE event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3){//TxE is set

		if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL) ){
			//MASTER MODE//

			//Transmit data only when its state is busy in transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){

				I2C_MasterHandleTXEInterrupt(pI2CHandle);

			}
		}
		else{
			//SLAVE MODE//
			if( pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA) ){

				I2C_Application_event_callback(pI2CHandle, I2C_EV_DATA_REQ);

			}

		}

	}


	//6.handle for interrupt generated by RxNE event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3){

		if( pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL) ){
			//MASTER MODE//

			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){

				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
		    }

		}
		else{
			//SLAVE MODE//
			if(!(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA)) ){

				I2C_Application_event_callback(pI2CHandle, I2C_EV_DATA_RCV);

			}
		}


	}

}


void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){

	uint8_t temp1, temp2;

	//check for the ITERREN
	temp2 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITERREN);

	////////1.check for the BUS error//////////
	temp1 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_BERR);
	if(temp1 && temp2){
		// bus error

		//clear the BUS error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_BERR);

		//notify about this error to the application
		I2C_Application_event_callback(pI2CHandle, I2C_ER_BERR);
	}


	////////2.check for the ARBITRATION error//////////
	temp1 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ARLO);
	if(temp1 && temp2){
		// arbitration error

		//clear the arbitration error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_ARLO);

		//notify about this error to the application
		I2C_Application_event_callback(pI2CHandle, I2C_ER_ARLO);
	}


	////////3.check for the ACK failure//////////
	temp1 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_AF);
	if(temp1 && temp2){
		//ACK failure error

		//clear the ACK failure error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_AF);

		//notify about this error to the application
		I2C_Application_event_callback(pI2CHandle, I2C_ER_AF);
	}


	////////4.check for the Overrun/underrun error//////////
	temp1 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_OVR);
	if(temp1 && temp2){
		//Overrun/underrun error

		//clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_OVR);

		//notify about this error to the application
		I2C_Application_event_callback(pI2CHandle, I2C_ER_OVR);
	}


	////////5.check for the  Time out error//////////
	temp1 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TIMEOUT);
	if(temp1 && temp2){
		//Overrun/underrun error

		//clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		//notify about this error to the application
		I2C_Application_event_callback(pI2CHandle, I2C_ER_TIMEOUT);
	}



}

void I2C_SLave_EnDi_callback_events(I2C_RegDef_t *pI2Cx, uint8_t ENorDI){

	if(ENorDI == ENABLE){

		//enable the ITBUFEN control bit
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);
		//enable the ITEVFEN control bit
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);
		//enable the ITERREN control bit
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);

	}
	else{

		//Disable the ITBUFEN Control bit
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		//Disable the ITEVFEN control bit
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		//Disable the ITERREN control bit
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITERREN);

	}
}

