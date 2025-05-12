/*
 * stm32f411ceu6.h
 *
 *  Created on: Apr 1, 2025
 *      Author: bhoomika
 */

#ifndef INC_STM32F411CEU6_H_
#define INC_STM32F411CEU6_H_
#include<stdint.h>
#define __vo volatile
////////////////DETAILS OF PROCESSOR///////////////////////////
#define NO_OF_PR_BITS_IMPLEMENTED   4
//ARM Cortex M4 processor NVIC ISERx register address
#define NVIC_ISER0     ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1     ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2     ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3     ( (__vo uint32_t*)0xE000E10C )

//ARM Cortex M4 processor NVIC ICERx register address
#define NVIC_ICER0     ( (__vo uint32_t*)0XE000E180 )
#define NVIC_ICER1     ( (__vo uint32_t*)0XE000E184 )
#define NVIC_ICER2     ( (__vo uint32_t*)0XE000E188 )
#define NVIC_ICER3     ( (__vo uint32_t*)0XE000E18C )
//ARM Cortex M4 processor NVIC Priority register address
#define NVIC_PR_BASEADDR  ( (__vo uint32_t*)0xE000E400 )


///////////////////////////////////////////////////////////////////
#define FLASH_BASE_ADDRESS           0x08000000U
#define SRAM_BASE_ADDRESS            0x20000000U
#define ROM_BASE_ADDRESS             0x1FFF0000U

#define PERIPHERAL_BASE_ADDRESS      0x40000000U
#define APB1_PERIPH_BASE_ADDRESS     PERIPHERAL_BASE_ADDRESS
#define APB2_PERIPH_BASE_ADDRESS     0x40010000U
#define AHB1_PERIPH_BASE_ADDRESS     0x40020000U
#define AHB2_PERIPH_BASE_ADDRESS     0x50000000U

//peripherals on AHB1 BUS
#define GPIOA_BASEADDR               (AHB1_PERIPH_BASE_ADDRESS + 0x0000)
#define GPIOB_BASEADDR               (AHB1_PERIPH_BASE_ADDRESS + 0x0400)
#define GPIOC_BASEADDR               (AHB1_PERIPH_BASE_ADDRESS + 0x0800)
#define GPIOD_BASEADDR               (AHB1_PERIPH_BASE_ADDRESS + 0x0C00)
#define GPIOE_BASEADDR               (AHB1_PERIPH_BASE_ADDRESS + 0x1000)
#define GPIOH_BASEADDR               (AHB1_PERIPH_BASE_ADDRESS + 0x1C00)
#define RCC_BASEADDR                 (AHB1_PERIPH_BASE_ADDRESS + 0x3800)

//peripherals on APB1 BUS
#define I2C1_BASEADDR                (APB1_PERIPH_BASE_ADDRESS + 0x5400)
#define I2C2_BASEADDR                (APB1_PERIPH_BASE_ADDRESS + 0x5800)
#define I2C3_BASEADDR                (APB1_PERIPH_BASE_ADDRESS + 0x5C00)
#define SPI2_BASEADDR                (APB1_PERIPH_BASE_ADDRESS + 0x3800U)
#define SPI3_BASEADDR                (APB1_PERIPH_BASE_ADDRESS + 0x3C00U)
#define USART2_BASEADDR              (APB1_PERIPH_BASE_ADDRESS + 0x4400)

//peripherals on APB2 BUS
#define SPI1_BASEADDR                (APB2_PERIPH_BASE_ADDRESS + 0x3000U)
#define SYSCFG_BASEADDR              (APB2_PERIPH_BASE_ADDRESS + 0x3800)
#define EXTI_BASEADDR                (APB2_PERIPH_BASE_ADDRESS + 0x3C00)
#define USART1_BASEADDR              (APB2_PERIPH_BASE_ADDRESS + 0x1000)
#define USART6_BASEADDR              (APB2_PERIPH_BASE_ADDRESS + 0x1400)


//Peripheral Register Definition Structure
typedef struct{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
}GPIO_RegDef_t;

//RCC Register Definition Structure
typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED2[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED6[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED7[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;

//EXTI Register Definition Structure
typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

//SYSCFG Register Definition Structure
typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

//Peripheral register definition structure for SPI
typedef struct{
	__vo uint32_t SPI_CR1;
	__vo uint32_t SPI_CR2;
	__vo uint32_t SPI_SR;
	__vo uint32_t SPI_DR;
	__vo uint32_t SPI_CRCPR;
	__vo uint32_t SPI_RXCRCR;
	__vo uint32_t SPI_TXCRCR;
	__vo uint32_t SPI_I2SCFGR;
	__vo uint32_t SPI_I2SPR;

}SPI_RegDef_t;

//peripheral definitions
#define GPIOA   ( (GPIO_RegDef_t*)GPIOA_BASEADDR )
#define GPIOB   ( (GPIO_RegDef_t*)GPIOB_BASEADDR )
#define GPIOC   ( (GPIO_RegDef_t*)GPIOC_BASEADDR )
#define GPIOD   ( (GPIO_RegDef_t*)GPIOD_BASEADDR )
#define GPIOE   ( (GPIO_RegDef_t*)GPIOE_BASEADDR )
#define GPIOH   ( (GPIO_RegDef_t*)GPIOH_BASEADDR )

#define RCC     ( (RCC_RegDef_t*)RCC_BASEADDR )

#define EXTI    ( (EXTI_RegDef_t*)EXTI_BASEADDR )

#define SYSCFG  ( (SYSCFG_RegDef_t*)SYSCFG_BASEADDR )

#define SPI1    ( (SPI_RegDef_t*)SPI1_BASEADDR )
#define SPI2    ( (SPI_RegDef_t*)SPI2_BASEADDR )
#define SPI3    ( (SPI_RegDef_t*)SPI3_BASEADDR )

//clock enable macros for GPIOx peripherals
#define GPIOA_PCLK_EN()   ( RCC -> AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()   ( RCC -> AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()   ( RCC -> AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()   ( RCC -> AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()   ( RCC -> AHB1ENR |= (1 << 4) )
#define GPIOH_PCLK_EN()   ( RCC -> AHB1ENR |= (1 << 7) )
//clock enable macros for I2Cx peripherals
#define I2C1_PCLK_EN()    ( RCC -> APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()    ( RCC -> APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()    ( RCC -> APB1ENR |= (1 << 23) )
//clock enable macros for SP1x peripherals
#define SPI1_PCLK_EN()    ( RCC -> APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()    ( RCC -> APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()    ( RCC -> APB1ENR |= (1 << 15) )
//clock enable macros for USARTx peripherals
#define USART1_PCLK_EN()  ( RCC -> APB2ENR |= (1 << 4)  )
#define USART2_PCLK_EN()  ( RCC -> APB1ENR |= (1 << 17) )
#define USART6_PCLK_EN()  ( RCC -> APB2ENR |= (1 << 5)  )
//clock enable macros for SYSCFGx peripherals
#define SYSCFG_PCLK_EN()  ( RCC -> APB2ENR |= (1 << 14) )


//clock disable macros for GPIOx peripherals
#define GPIOA_PCLK_DI()   ( RCC -> AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()   ( RCC -> AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()   ( RCC -> AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()   ( RCC -> AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()   ( RCC -> AHB1ENR &= ~(1 << 4) )
#define GPIOH_PCLK_DI()   ( RCC -> AHB1ENR &= ~(1 << 7) )
//clock disablE macros for I2Cx peripherals
#define I2C1_PCLK_DI()    ( RCC -> APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()    ( RCC -> APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()    ( RCC -> APB1ENR &= ~(1 << 23) )
//clock disable macros for SP1x peripherals
#define SPI1_PCLK_DI()    ( RCC -> APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()    ( RCC -> APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()    ( RCC -> APB1ENR &= ~(1 << 15) )
//clock disable macros for USARTx peripherals
#define USART1_PCLK_DI()  ( RCC -> APB2ENR &= ~(1 << 4)  )
#define USART2_PCLK_DI()  ( RCC -> APB1ENR &= ~(1 << 17) )
#define USART6_PCLK_DI()  ( RCC -> APB2ENR &= ~(1 << 5)  )
//clock disable macros for SYSCFGx peripherals
#define SYSCFG_PCLK_DI()  ( RCC -> APB2ENR &= ~(1 << 14) )


//REG_RESET
#define GPIOA_REG_RESET()   do{ RCC -> AHB1RSTR |= (1 << 0);  RCC -> AHB1RSTR &= ~(1 << 0); }while(0)
#define GPIOB_REG_RESET()   do{ RCC -> AHB1RSTR |= (1 << 1);  RCC -> AHB1RSTR &= ~(1 << 1); }while(0)
#define GPIOC_REG_RESET()   do{ RCC -> AHB1RSTR |= (1 << 2);  RCC -> AHB1RSTR &= ~(1 << 2); }while(0)
#define GPIOD_REG_RESET()   do{ RCC -> AHB1RSTR |= (1 << 3);  RCC -> AHB1RSTR &= ~(1 << 3); }while(0)
#define GPIOE_REG_RESET()   do{ RCC -> AHB1RSTR |= (1 << 4);  RCC -> AHB1RSTR &= ~(1 << 4); }while(0)
#define GPIOH_REG_RESET()   do{ RCC -> AHB1RSTR |= (1 << 7);  RCC -> AHB1RSTR &= ~(1 << 7); }while(0)

#define GPIO_BASEADDR_TO_CODE(x)    ( (x == GPIOA)?0:\
		                              (x == GPIOB)?1:\
		                              (x == GPIOC)?2:\
		                              (x == GPIOD)?3:\
		                              (x == GPIOE)?4:\
		                              (x == GPIOH)?7:0 )
#define SPI1_RST()        do{ RCC -> APB2RSTR |= (1 << 12); RCC -> APB2RSTR &= ~(1 << 12); }while(0)
#define SPI2_RST()        do{ RCC -> APB1RSTR |= (1 << 14); RCC -> APB1RSTR &= ~(1 << 14); }while(0)
#define SPI3_RST()        do{ RCC -> APB1RSTR |= (1 << 15); RCC -> APB1RSTR &= ~(1 << 15); }while(0)
//IRQ(Interrupt Request ) number
#define IRQ_NO_EXTI0        6
#define IRQ_NO_EXTI1        7
#define IRQ_NO_EXTI2        8
#define IRQ_NO_EXTI3        9
#define IRQ_NO_EXTI4       10
#define IRQ_NO_EXTI9_5     23
#define IRQ_NO_EXTI15_10   40

#define NVIC_IRQ_PR0        0
#define NVIC_IRQ_PR15       15

//////////////GENERIC MACROS//////////////////
#define ENABLE      1
#define DISABLE     0
#define SET         ENABLE
#define RESET       DISABLE
#define FLAG_SET    SET
#define FLAG_RESET  RESET

//bit position definition of SPI peripheral
#define SPI_CR1_CPHA      0
#define SPI_CR1_CPOL      1
#define SPI_CR1_MSTR      2
#define SPI_CR1_BR        3
#define SPI_CR1_SPE       6
#define SPI_CR1_LSBFIRST  7
#define SPI_CR1_SSI       8
#define SPI_CR1_SSM       9
#define SPI_CR1_RXONLY    10
#define SPI_CR1_DFF       11
#define SPI_CR1_CRCNEXT   12
#define SPI_CR1_CRCEN     13
#define SPI_CR1_BIDIOE    14
#define SPI_CR1_BIDIMODE  15

#include "stm32f411ceu6_gpio_driver.h"
#include "stm32f411ceu6_spi_driver.h"


#endif /* INC_STM32F411CEU6_H_ */
