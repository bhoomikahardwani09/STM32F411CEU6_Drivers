#include "stm32f411ceu6_spi_driver.h"

static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handler(SPI_Handle_t *pSPIHandle);
//enabling the peripheral clock
void SPI_PeriphCLKCtrl(SPI_RegDef_t *pSPIx, uint8_t ENorDI){
	if(ENorDI == ENABLE){
			if(pSPIx == SPI1)          SPI1_PCLK_EN();
			else if(pSPIx == SPI2)     SPI2_PCLK_EN();
			else if(pSPIx == SPI3)     SPI3_PCLK_EN();
		}
		else{
			if(pSPIx == SPI1)          SPI1_PCLK_DI();
	        else if(pSPIx == SPI2)     SPI2_PCLK_DI();
	        else if(pSPIx == SPI3)     SPI3_PCLK_DI();
		}
}

//Init & DeInit
void SPI_Init(SPI_Handle_t *pSPIHandle){
	uint32_t temp_reg = 0;
	//enabling the peripheral clock
	SPI_PeriphCLKCtrl(pSPIHandle->pSPIx, ENABLE);
	//1. configuring the device mode as master or slave
	temp_reg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;
	//2. configuring the bus configuration
	if(pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BusConfig_FD){
		//bidi mode should be cleared
		temp_reg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BusConfig_HD){
		//bidi mode should be set
		temp_reg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BusConfig_SimplexRX){
		//bidi mode should be cleared
		temp_reg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXonly bit must be set
		temp_reg |= (1 << SPI_CR1_RXONLY);
	}

	//3. configuring the serial clock speed
	temp_reg |= pSPIHandle -> SPIConfig.SPI_SCLKspeed << SPI_CR1_BR;
	//4. configuring the DFF
	temp_reg |= pSPIHandle -> SPIConfig.SPI_DFF << SPI_CR1_DFF;
	//5. configuring CPOL and CPHA
	temp_reg |= pSPIHandle -> SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	temp_reg |= pSPIHandle -> SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
	pSPIHandle -> pSPIx -> SPI_CR1 = temp_reg;
}
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1)      SPI1_RST();
	else if(pSPIx == SPI1) SPI1_RST();
	else if(pSPIx == SPI1) SPI1_RST();
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag){
	if(pSPIx->SPI_SR & flag) return FLAG_SET;
	return FLAG_RESET;
}
//Data send and receive
//known as blocking API as it waits for the whole data to transfer
//also known as polling based API function because here we are polling for the TXE flag to set
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len){
	if(len == 0) return;
	while(len > 0){
//		while(!(pSPIx -> SPI_SR & (1 << 1))); //wait until TXE is set = TX buffer is empty
		while(SPI_GetFlagStatus(pSPIx, SPI_TX_FLAG) == FLAG_RESET);

		// checking the DFF bit
		if(pSPIx -> SPI_CR1 & (1 << SPI_CR1_DFF)){//its a 16 bit DFF
			pSPIx -> SPI_DR = *((uint16_t*)pTxBuffer);
			len -= 2;
			(uint16_t*)pTxBuffer++;
		}
		else{// 8 bit DFF
			pSPIx -> SPI_DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len){
	if(len == 0) return;
	while(len > 0){
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_SET);//wait until the RX buffer is not empty

		//check DFF bit
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){ //its 16-bit data
			//loading data from DR into the Rx buffer
			*((uint16_t*)pRxBuffer) = pSPIx->SPI_DR;
			len -= 2;
			(uint16_t*)pRxBuffer++;
		}
		else{//its 8 bit data
			//loading data from DR into the Rx buffer
			*pRxBuffer = pSPIx->SPI_DR;
			len--;
			pRxBuffer++;
		}
	}
}

//IRQ Configuration & ISR Handling
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t ENorDI){
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


void SPI_IRQPriority(uint8_t IRQNumber, uint8_t IRQPriority){
	//1.IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_section = (IRQNumber % 4);
	uint8_t shift_parameter = (IPRx_section * 8) + ( 8 - NO_OF_PR_BITS_IMPLEMENTED);
	*( NVIC_PR_BASEADDR + (IPRx * 4) ) |= (IRQPriority << shift_parameter);
}


uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len){
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX){
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		//2. mark the SPI state busy so that no other can use it
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. enable the TXEIE control bit to get interrupt when TXE is set in SR(Tx is empty)
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << 7);

		//4. data transmission will be handled by the ISR code

	}
	return state;
}
uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len){
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX){
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//3. enable the RXEIE control bit to get interrupt when RXNE is set in SR(Rx not empty)
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << 6);
	}
	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pHandle){//INTERRUPT FOR A PARTICULAR PIN NUMBER WILL BE MANAGED
	uint8_t temp1, temp2;
	//.checking the reason of interrupt
	//1. checking TXE flag
	temp1 = pHandle->pSPIx->SPI_SR & (SPI_TX_FLAG);
	temp2 = pHandle->pSPIx->SPI_CR2 & (1 << 7);
	if(temp1 && temp2){
		//handle TXE : helper function
		spi_txe_interrupt_handler(pHandle);
	}

	//2. checking the RXNE flag
	temp1 = pHandle->pSPIx->SPI_SR & (SPI_RXNE_FLAG);
	temp2 = pHandle->pSPIx->SPI_CR2 & (1 << 6);
	if(temp1 && temp2){
		//handle TXE : helper function
		spi_rxne_interrupt_handler(pHandle);
	}

	//3. checking the overrun error
	temp1 = pHandle->pSPIx->SPI_SR & (1 << 6);
	temp2 = pHandle->pSPIx->SPI_CR2 & (1 << 5);
	if(temp1 && temp2){
		//handle TXE : helper function
		spi_ovr_interrupt_handler(pHandle);
	}


}

void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t ENorDI){
	if(ENorDI == ENABLE) pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
	else pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
}

void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDI){
	if(ENorDI == ENABLE) pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
	else pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
}

void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDI){
	if(ENorDI == ENABLE) pSPIx->SPI_CR2 |= (1 << 2);
	else pSPIx->SPI_CR2 &= ~(1 << 2);
}



static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2;
		(uint16_t*)pSPIHandle->pTxBuffer ++;

	}
	else{
		pSPIHandle->pSPIx->SPI_DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(! pSPIHandle->TxLen){
		//closing spi comm.n and inform the application that TX is over
		SPI_CloseTransmission(pSPIHandle);
		SPI_Application_event_callback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->RxLen -= 2;
		(uint16_t*)pSPIHandle->pRxBuffer ++;

	}
	else{
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(! pSPIHandle->RxLen){
		//closing spi comm.n and inform the application that TX is over
		SPI_CloseReception(pSPIHandle);
		SPI_Application_event_callback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}

}

static void spi_ovr_interrupt_handler(SPI_Handle_t *pSPIHandle){
	//1. clear the overrun flag = reading the DR and then read access to SR
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;
	}
	(void)temp; // as temp is not used so we are type casting it
	//2. inform the application
	SPI_Application_event_callback(pSPIHandle, SPI_EVENT_OVR_CMPLT);

}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void)temp;

}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << 7); //prevents interrupt from TXE flag
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << 6); //prevents interrupt from TXE flag
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

__attribute__((weak)) void SPI_Application_event_callback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent){
	//this is a weak implementation . the application may override this function.
}


