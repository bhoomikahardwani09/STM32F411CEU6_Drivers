#include "stm32f411ceu6_spi_driver.h"

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
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len){
	if(len == 0) return;
	else{
//		while(!(pSPIx -> SPI_SR & (1 << 1))); //wait until TXE is set
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

//IRQ Configuration & ISR Handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t ENorDI);
void GPIO_IRQPriority(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(SPI_Handle_t *pHandle);   //INTERRUPT FOR A PARTICULAR PIN NUMBER WILL BE MANAGED

void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t ENorDI){
	if(ENorDI == ENABLE) pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
	else pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
}
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDI){
	if(ENorDI == ENABLE) pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
	else pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
}







