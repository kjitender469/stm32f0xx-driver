/*
 * stm32f030xx.h
 *
 *  Created on: June 8, 2020
 *      Author: Jitender Kumar
 *      E-mail: kjitender469@gmail.com
 *
 *  Upated:
 */

#ifndef INC_STM32F030XX_H_
#define INC_STM32F030XX_H_

#include <stddef.h>
#include <stdint.h>

/*
 * Abbreviations used are declared here
 * */
#define __vo volatile
#define __weak __attribute__((weak))



/****************************************** Processor Specific Details ******************************************/
/*
 * ARM Cortex Mx Processor Interrupt Set Enable Register(ISER) Address
 * */
#define NVIC_ISER			( (__vo uint32_t*)0xE000E100 )		/* Address of ISER(Interrupt Set Enable Register) based on Cortex Mx Devices Generic User Guide */

/*
 * ARM Cortex Mx Processor Interrupt Clear Enable Register(ICER) Address
 * */
#define NVIC_ICER			( (__vo uint32_t*)0xE000E180 )		/* Address of ICER(Interrupt Clear Enable Register) based on Cortex Mx Devices Generic User Guide */

/*
 * ARM Cortex Mx Processor Interrupt Set Pending Register(ISPR) Address
 * */
#define NVIC_ISPR			( (__vo uint32_t*)0xE000E200 )		/* Address of ISPR(Interrupt Set Pending Register) based on Cortex Mx Devices Generic User Guide */

/*
 * ARM Cortex Mx Processor Interrupt Clear Enable Register(ICPR) Address
 * */
#define NVIC_ICPR			( (__vo uint32_t*)0xE000E280 )		/* Address of ISPR(Interrupt Clear Pending Register) based on Cortex Mx Devices Generic User Guide */


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 * */
#define NVIC_PR_BASE_ADDR			( (__vo uint32_t*)0xE000E400 )

#define NO_PR_BITS_IMPLEMENTED		2 		/* Number of Priority bits implemented. It is specific to MCU. In Cortex M0 Number of Priority bits implemented are 2 */

//#define NVIC_IPR0			( (__vo uint32_t*)0xE000E400 )		/* Address of IPR(Interrupt Priority Register) based on Cortex Mx Devices Generic User Guide */
//#define NVIC_IPR1			( (__vo uint32_t*)0xE000E404 )
//#define NVIC_IPR2			( (__vo uint32_t*)0xE000E408 )
//#define NVIC_IPR3			( (__vo uint32_t*)0xE000E40C )		/* Address of IPR(Interrupt Priority Register) based on Cortex Mx Devices Generic User Guide */
//#define NVIC_IPR4			( (__vo uint32_t*)0xE000E410 )
//#define NVIC_IPR5			( (__vo uint32_t*)0xE000E414 )
//#define NVIC_IPR6			( (__vo uint32_t*)0xE000E418 )
//#define NVIC_IPR7			( (__vo uint32_t*)0xE000E41C )



/*
 * Base addresses of Flash and SRAM memories
 *
 * 	SRAM 		8Kb
 * 	FLASH		64Kb
 * 	ROM
 * 	Timers		7
 * 		Basic					1 (16-bit)
 * 		General Purpose			5 (16-bit)
 *		Advanced Control		1 (16-bit)
 * 	Communication Interface		6
 * 		SPI						2
 * 		I2C						2
 * 		USART					2	(USART3 to USART6 are not present)
 *	GPIOs						55
 *	ADC							1	(16 external + 2 internal channel)
 *	CPU Frequency				48 MHz Max.
 *	Operating Voltage			2.4 to 3.6V
 *
 * */

// By default compiler treat these addresses as signed number thats why we use 'U' to
// tell compiler that these are unsigned integer numbers.
// Alternatively we can also be type cast these numbers using (uint32_t).
// Ex - (uint32_t)0x20000000

#define FLASH_BASE_ADDR				0x08000000U		/* Base Address of Flash Memory */
#define SRAM_1_BASE_ADDR			0x20000000U		/* SRAM address. Size 8Kb. */
#define SRAM SRAM_1_BASE_ADDR
#define ROM							0x1FFFEC00		/* ROM Address */

/*
 * 	AHBx and APBx Bus Peripheral Base Addresses
 * 	In data sheet there is not separate APB1 and APB2 buses are mentioned but when
 * 	you look at the addresses of APB bus then we have to split it out into APB1 and
 * 	APB2 to access all the peripheral's addresses correctly. Page 39/93 and 40/93 of
 * 	manufacturer data sheet.
 * */
#define	PERIPH_BASE					0x40000000U
#define	APB1_PERIPH_BASE			PERIPH_BASE
#define APB2_PERIPH_BASE			0x40010000U
#define AHB1_PERIPH_BASE			0x40020000U
#define AHB2_PERIPH_BASE			0x48000000U

/*
 * Base Addresses of all the peripherals which are hanging on AHB1 Bus
 * */
#define	DMA_BASE_ADDR				(AHB1_PERIPH_BASE + 0x0000)
#define RCC_BASE_ADDR				(AHB1_PERIPH_BASE + 0x1000)
#define	CRC_BASE_ADDR				(AHB1_PERIPH_BASE + 0x3000)
//#define FLASH_BASE_ADDR				(AHB1_PERIPH_BASE + 0x2000) // commented after warning says "FLASH_BASE_ADDR" redefined

/*
 * Base Addresses of all the peripherals which are hanging on AHB2 Bus
 * */
#define GPIOA_BASE_ADDR				(AHB2_PERIPH_BASE + 0x0000)
#define GPIOB_BASE_ADDR				(AHB2_PERIPH_BASE + 0x0400)
#define GPIOC_BASE_ADDR				(AHB2_PERIPH_BASE + 0x0800)
#define GPIOD_BASE_ADDR				(AHB2_PERIPH_BASE + 0x0C00)
#define GPIOF_BASE_ADDR				(AHB2_PERIPH_BASE + 0x1400)

/*
 * Base Addresses of all the peripherals which are hanging on APB1 Bus
 * */
#define I2C1_BASE_ADDR				(APB1_PERIPH_BASE + 0x5400)
#define I2C2_BASE_ADDR				(APB1_PERIPH_BASE + 0x5800)

#define SPI2_BASE_ADDR				(APB1_PERIPH_BASE + 0x3800)

#define USART2_BASE_ADDR			(APB1_PERIPH_BASE + 0x4400)

#define RTC_BASE_ADDR				(APB1_PERIPH_BASE + 0x2800)

#define TIMER3_BASE_ADDR			(APB1_PERIPH_BASE + 0x0400)
#define TIMER6_BASE_ADDR			(APB1_PERIPH_BASE + 0x1000)
#define TIMER14_BASE_ADDR			(APB1_PERIPH_BASE + 0x2000)

#define WWDG_BASE_ADDR				(APB1_PERIPH_BASE + 0x2C00)
#define IWDG_BASE_ADDR				(APB1_PERIPH_BASE + 0x3000)


/*
 * Base Addresses of all the peripherals which are hanging on APB2 Bus
 * */
#define SPI1_BASE_ADDR				(APB2_PERIPH_BASE + 0x3000)

#define	USART1_BASE_ADDR			(APB2_PERIPH_BASE + 0x3800)

#define	TIMER1_BASE_ADDR			(APB2_PERIPH_BASE + 0x2C00)
#define TIMER15_BASE_ADDR			(APB2_PERIPH_BASE + 0x4000)
#define TIMER16_BASE_ADDR			(APB2_PERIPH_BASE + 0x4400)
#define TIMER17_BASE_ADDR			(APB2_PERIPH_BASE + 0x4800)

#define ADC_BASE_ADDR				(APB2_PERIPH_BASE + 0x2400)

#define EXTI_BASE_ADDR				(APB2_PERIPH_BASE + 0x0400)

#define	SYSCFG_BASE_ADDR			(APB2_PERIPH_BASE + 0x0000)

#define DBGMCU_BASE_ADDR			(APB2_PERIPH_BASE + 0x5800)



/*********** Peripheral register definition structure ***********/

/*
 * 	Note : Registers of a peripheral are specific to MCU
 * 	e.g : Number of registers of SPI peripheral of STM32F0x family of MCUs may be
 * 	different(more or less) compared to number of registers of SPI peripheral of
 * 	STM32Lx or STM32F4x family of MCUs. Please check your Device Reference Manual.
 * */

typedef struct
{

	__vo uint32_t MODER;		/* GPIO port mode register						Address offset: 0x00 */
	__vo uint32_t OTYPER;		/* GPIO port output type register				Address offset: 0x04 */
	__vo uint32_t OSPEEDR;		/* GPIO port output speed register				Address offset: 0x08 */
	__vo uint32_t PUPDR;		/* GPIO port pull-up/pull-down register			Address offset: 0x0C */
	__vo uint32_t IDR;			/* GPIO port input data register				Address offset: 0x10 */
	__vo uint32_t ODR;			/* GPIO port output data register				Address offset: 0x14 */
	__vo uint32_t BSRR;			/* GPIO port bit set/reset register				Address offset: 0x18 */
	__vo uint32_t LCKR;			/* GPIO port configuration register				Address offset: 0x1C */
	__vo uint32_t AFR[2];		/* GPIO port alternate function register array. index 0 refer to
						   	   	   AFRL and index 1 refer to AFRH 				Address offset: 0x20-0x24 */
	__vo uint32_t BRR;			/* GPIO port bit reset register register		Address offset: 0x28 */

}GPIO_RegDef_t;



/*
 * Peripheral Register Definition Structure for SPI
 * */
typedef struct
{
	__vo uint32_t CR1;			/* SPI control register						Address offset: 0x00 */
	__vo uint32_t CR2;			/* SPI control register						Address offset: 0x04 */
	__vo uint32_t SR;			/* SPI status register						Address offset: 0x08 */
	__vo uint16_t DR;			/* SPI data register						Address offset: 0x0C */
	__vo uint32_t CRCPR;		/* SPI CRC polynomial register				Address offset: 0x10 */
	__vo uint32_t RXCRCR;		/* SPI Rx CRC register						Address offset: 0x14 */
	__vo uint32_t TXCRCR;		/* SPI Tx CRC register						Address offset: 0x18 */

}SPI_RegDef_t;



/*
 * Peripheral Register Definition Structure for I2C
 * */
typedef struct
{
	__vo uint32_t CR1;			/* I2C control register						Address offset: 0x00 */
	__vo uint32_t CR2;			/* I2C control register						Address offset: 0x04 */
	__vo uint32_t OAR1;			/* I2C own address register					Address offset: 0x08 */
	__vo uint32_t OAR2;			/* I2C own address register					Address offset: 0x0C */
	__vo uint32_t TIMINGR;		/* I2C timing register						Address offset: 0x10 */
	__vo uint32_t TIMEOUTR;		/* I2C timeout register						Address offset: 0x14 */
	__vo uint32_t ISR;			/* I2C Interrupt and status register		Address offset: 0x18 */
	__vo uint32_t ICR;			/* I2C Interrupt clear register				Address offset: 0x1C */
	__vo uint32_t PECR;			/* I2C Packet Error Checking register		Address offset: 0x20 */
	__vo uint32_t RXDR;			/* I2C Receive Data register				Address offset: 0x24 */
	__vo uint32_t TXDR;			/* I2C transmit Data register				Address offset: 0x28 */

}I2C_RegDef_t;



/*
 * Peripheral Register Definition Structure for USART
 * */
typedef struct
{
	__vo uint32_t CR1;			/* USART control register						Address offset: 0x00 */
	__vo uint32_t CR2;			/* USART control register						Address offset: 0x04 */
	__vo uint32_t CR3;			/* USART control register						Address offset: 0x08 */
	__vo uint32_t BRR;			/* USART Baud rate register 					Address offset: 0x0C */
	__vo uint32_t RTOR;			/* USART Receiver timeout register 				Address offset: 0x14 */
	__vo uint32_t RQR;			/* USART Request register 						Address offset: 0x18 */
	__vo uint32_t ISR;			/* USART Interrupt and status register 			Address offset: 0x1C */
	__vo uint32_t ICR;			/* USART Interrupt flag clear register 			Address offset: 0x20 */
	__vo uint32_t RDR;			/* USART Receive data register 					Address offset: 0x24 */
	__vo uint32_t TDR;			/* USART Transmit data register 				Address offset: 0x28 */

}USART_RegDef_t;



/*
 * Peripheral Register Definition Structure for EXTI
 * */
typedef struct
{
	__vo uint32_t IMR;			/* Interrupt mask register						Address offset: 0x00 */
	__vo uint32_t EMR;			/* Event mask register 							Address offset: 0x04 */
	__vo uint32_t RTSR;			/* Rising trigger selection register  			Address offset: 0x08 */
	__vo uint32_t FTSR;			/* Falling trigger selection register 			Address offset: 0x0C */
	__vo uint32_t SWIER;		/* Software interrupt event register 			Address offset: 0x10 */
	__vo uint32_t PR;			/* Pending register 							Address offset: 0x14 */
}EXTI_RegDef_t;


/*
 * peripheral register definition for SYSCFG
 * */
typedef struct
{
	__vo uint32_t CFGR1;			/* SYSCFG configuration register 1				Address offset: 0x00 */
	__vo uint32_t EXTICR[4];		/* SYSCFG external interrupt
										configuration register 1					Address offset: 0x04 - 0x14 */
	__vo uint32_t CFGR2;			/* SYSCFG configuration register 2  			Address offset: 0x18 */
}SYSCFG_RegDef_t;


typedef struct
{

	__vo uint32_t CR;			/* Clock control register						Address offset: 0x00 */
	__vo uint32_t CFGR;			/* Clock configuration register 				Address offset: 0x04 */
	__vo uint32_t CIR;			/* Clock interrupt register 					Address offset: 0x08 */
	__vo uint32_t APB2RSTR;		/* APB peripheral reset register 2 				Address offset: 0x0C */
	__vo uint32_t APB1RSTR;		/* APB peripheral reset register 1 				Address offset: 0x10 */
	__vo uint32_t AHBENR;		/* AHB peripheral clock enable register 		Address offset: 0x14 */
	__vo uint32_t APB2ENR;		/* APB peripheral clock enable register 2 		Address offset: 0x18 */
	__vo uint32_t APB1ENR;		/* APB peripheral clock enable register 1 		Address offset: 0x1C */
	__vo uint32_t BDCR;			/* RTC domain control register 					Address offset: 0x20 */
	__vo uint32_t CSR;			/* Control/status register 						Address offset: 0x24 */
	__vo uint32_t AHBRSTR;		/* AHB peripheral reset register 				Address offset: 0x28 */
	__vo uint32_t CFGR2;		/* Clock configuration register 2 				Address offset: 0x2C */
	__vo uint32_t CFGR3;		/* Clock configuration register 3 				Address offset: 0x30 */
	__vo uint32_t CR2;			/* Clock control register 2 					Address offset: 0x34 */

}RCC_RegDef_t;


/*
 * 	Peripheral definitions ( Peripheral base addresses type casted to xxx_RegDef_t)
 * */
#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASE_ADDR)

#define RCC			((RCC_RegDef_t*)RCC_BASE_ADDR)

#define EXTI		((EXTI_RegDef_t*)EXTI_BASE_ADDR)

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASE_ADDR)

#define I2C1		((I2C_RegDef_t*)I2C1_BASE_ADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASE_ADDR)

#define USART1		((USART_RegDef_t*)USART1_BASE_ADDR)
#define USART2		((USART_RegDef_t*)USART2_BASE_ADDR)



/*
 * 	Clock Enable Macros for GPIOx peripherals
 * */
#define GPIOA_PCLK_EN()				( RCC->AHBENR |= (1 << 17) )
#define GPIOB_PCLK_EN()				( RCC->AHBENR |= (1 << 18) )
#define GPIOC_PCLK_EN()				( RCC->AHBENR |= (1 << 19) )
#define GPIOD_PCLK_EN()				( RCC->AHBENR |= (1 << 20) )
#define GPIOF_PCLK_EN()				( RCC->AHBENR |= (1 << 22) )

/*
 * 	Clock Enable Macros for I2Cx peripherals
 * */
#define I2C1_PCLK_EN()				( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()				( RCC->APB1ENR |= (1 << 22) )


/*
 * 	Clock Enable Macros for SPIx peripherals
 * */
#define SPI1_PCLK_EN()				( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()				( RCC->APB1ENR |= (1 << 14) )

/*
 * 	Clock Enable Macros for USARTx peripherals
 * */
#define USART1_PCLK_EN()				( RCC->APB2ENR |= (1 << 14) )
#define USART2_PCLK_EN()				( RCC->APB1ENR |= (1 << 17) )

/*
 * 	Clock Enable Macros for SYSCFGx peripherals
 * */
#define SYSCFG_PCLK_EN()				( RCC->APB2ENR |= (1 << 0) )



/*
 * 	Clock Disable Macros for GPIOx peripherals
 * */
#define GPIOA_PCLK_DI()				( RCC->AHBENR &= ~(1 << 17) )
#define GPIOB_PCLK_DI()				( RCC->AHBENR &= ~(1 << 18) )
#define GPIOC_PCLK_DI()				( RCC->AHBENR &= ~(1 << 19) )
#define GPIOD_PCLK_DI()				( RCC->AHBENR &= ~(1 << 20) )
#define GPIOF_PCLK_DI()				( RCC->AHBENR &= ~(1 << 22) )

/*
 * 	Clock Disable Macros for I2Cx peripherals
 * */
#define I2C1_PCLK_DI()				( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()				( RCC->APB1ENR &= ~(1 << 22) )

/*
 * 	Clock Disable Macros for SPIx peripherals
 * */
#define SPI1_PCLK_DI()				( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()				( RCC->APB1ENR &= ~(1 << 14) )

/*
 * 	Clock Disable Macros for USARTx peripherals
 * */
#define USART1_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 14) )
#define USART2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 17) )

/*
 * 	Clock Disable Macros for SYSCFGx peripherals
 * */
#define SYSCFG_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 0) )


/*
 * Macros to reset GPIOx peripheral
 * */
#define GPIOA_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 17)); (RCC->AHBRSTR &= ~(1 << 17)); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 18)); (RCC->AHBRSTR &= ~(1 << 18)); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 19)); (RCC->AHBRSTR &= ~(1 << 19)); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 20)); (RCC->AHBRSTR &= ~(1 << 20)); }while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 22)); (RCC->AHBRSTR &= ~(1 << 22)); }while(0)


/*
 * Macros to reset SPIx peripheral
 * */
#define SPI1_REG_RESET()			do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()			do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)



/*
 * Macros to reset I2Cx peripheral
 * */
#define I2C1_REG_RESET()			do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()			do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)



/*
 * Macros to reset USARTx peripheral
 * */
#define USART1_REG_RESET()			do{ (RCC->APB2RSTR |= (1 << 14)); (RCC->APB2RSTR &= ~(1 << 14)); }while(0)
#define USART2_REG_RESET()			do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0)



/*
 * Return port code for GPIOx base address
 *
 * 4 is reserved that why value for GPIOF is 5.
 *
 * */
#define GPIO_BASE_ADDR_TO_CODE(x)		( (x == GPIOA) ? 0 : \
										  (x == GPIOB) ? 1 : \
										  (x == GPIOC) ? 2 : \
										  (x == GPIOD) ? 3 : \
										  (x == GPIOF) ? 5 : 0 )


/*
 * IRQ(Interrupt Request) numbers of STM32F030xx MCU
 * NOTE: These macros values are specific to MCU so updated accordingly.
 * These numbers 5,6,7 are basically the position of Interrupts(IRQ_Numbers)
 * */
#define IRQ_NO_EXTI0_1		5
#define IRQ_NO_EXTI2_3		6
#define IRQ_NO_EXTI4_15		7


/*
 * IRQ(Interrupt Request) numbers for SPI Peripheral
 * */
#define IRQ_NO_SPI1			32
#define IRQ_NO_SPI2			33


/*
 * IRQ(Interrupt Request) numbers for I2C Peripheral
 * */
#define IRQ_NO_I2C1			30
#define IRQ_NO_I2C2			31


/*
 * Macros for Priority Levels
 * */
#define NVIC_IRQ_PRIORITY_0		0
#define NVIC_IRQ_PRIORITY_1		1
#define NVIC_IRQ_PRIORITY_2		2
#define NVIC_IRQ_PRIORITY_3		3
#define NVIC_IRQ_PRIORITY_4		4
#define NVIC_IRQ_PRIORITY_5		5
#define NVIC_IRQ_PRIORITY_6		6
#define NVIC_IRQ_PRIORITY_7		7

/*
 * 	Some generic Macros
 * */
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET


/********************************************************************************************************
 * Bit Position definitions of SPI peripheral
 * *****************************************************************************************************/

/*
 * Bit Position definitions for SPI_CR1 Register
 * */
#define SPI_CR1_CPHA		0		/* Clock Phase 				*/
#define SPI_CR1_CPOL		1		/* Clock Polarity 			*/
#define SPI_CR1_MSTR		2		/* Master Selection 		*/
#define SPI_CR1_BR			3		/* Baud Rate		 		*/
#define SPI_CR1_SPE			6		/* SPI Peripheral Enable 	*/
#define SPI_CR1_LSBFIRST	7		/* Data Transmission Format */
#define SPI_CR1_SSI			8		/* Internal Slave Select 	*/
#define SPI_CR1_SSM			9		/* Software Slave Management*/
#define SPI_CR1_RXONLY		10		/* Receive Only Mode 		*/
#define SPI_CR1_CRCL		11		/* CRC length 				*/
#define SPI_CR1_CRCNEXT		12		/* Transmit CRC Next 		*/
#define SPI_CR1_CRCEN		13		/* Hardware CRC calculation enable 		*/
#define SPI_CR1_BIDIOE		14		/* Output enable in bidirectional mode 	*/
#define SPI_CR1_BIDIMODE	15		/* Bidirectional data mode enable 		*/

/*
 * Bit Position definitions for SPI_CR2 Register
 * */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_NSSP		3
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7
#define SPI_CR2_DS			8
#define SPI_CR2_FRXTH		12
#define SPI_CR2_LDMA_RX		13
#define SPI_CR2_LDMA_TX		14

/*
 * Bit Position definitions for SPI_SR Register
 * */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8
#define SPI_SR_FRLVL		9
#define SPI_SR_FTLEL		11


/********************************************************************************************************
 * Bit Position definitions of I2C peripheral
 * *****************************************************************************************************/

/*
 * Bit Position definitions for I2C_CR1 Register
 * */
#define I2C_CR1_PE			0
#define I2C_CR1_TXIE		1
#define I2C_CR1_RXIE		2
#define I2C_CR1_ADDRIE		3
#define I2C_CR1_NACKIE		4
#define I2C_CR1_STOPIE		5
#define I2C_CR1_TCIE		6
#define I2C_CR1_ERRIE		7
#define I2C_CR1_DNF			8
#define I2C_CR1_ANFOFF		12
#define I2C_CR1_TXDMAEN		14
#define I2C_CR1_RXDMAEN		15
#define I2C_CR1_SBC			16
#define I2C_CR1_NOSTRETCH	17
#define I2C_CR1_GCEN		19
#define I2C_CR1_SMBHEN		20
#define I2C_CR1_SMBDEN		21
#define I2C_CR1_ALERTEN		22
#define I2C_CR1_PECEN		23


/*
 * Bit Position definitions for I2C_CR2 Register
 * */
#define I2C_CR2_SADD		0
#define I2C_CR2_RD_WRN		10
#define I2C_CR2_ADD10		11
#define I2C_CR2_HEAD10R		12
#define I2C_CR2_START		13
#define I2C_CR2_STOP		14
#define I2C_CR2_NACK		15
#define I2C_CR2_NBYTES		16
#define I2C_CR2_RELOAD		24
#define I2C_CR2_AUTOEND		25
#define I2C_CR2_PECBYTE		26


/*
 * Bit Position definitions for I2C_ISR Register
 * */
#define I2C_ISR_TXE			0
#define I2C_ISR_TXIS		1
#define I2C_ISR_RXNE		2
#define I2C_ISR_ADDR		3
#define I2C_ISR_NACKF		4
#define I2C_ISR_STOPF		5
#define I2C_ISR_TC			6
#define I2C_ISR_TCR			7
#define I2C_ISR_BERR		8
#define I2C_ISR_ARLO		9
#define I2C_ISR_OVR			10
#define I2C_ISR_PECERR		11
#define I2C_ISR_TIMEOUT		12
#define I2C_ISR_ALERT		13
#define I2C_ISR_BUSY		15
#define I2C_ISR_DIR			16
#define I2C_ISR_ADDCODE		17



/*
 * Bit Position definitions for I2C_ICR Register
 * */
#define I2C_ICR_ADDRCF		3
#define I2C_ICR_NACKCF		4
#define I2C_ICR_STOPCF		5
#define I2C_ICR_BERRCF		8
#define I2C_ICR_ARLOCF		9
#define I2C_ICR_OVRCF		10
#define I2C_ICR_PECCF		11
#define I2C_ICR_TIMOUTCF	12
#define I2C_ICR_ALERTCF		13



/*
 * Bit Position definitions for USART_CR1 Register
 * */
#define USART_CR1_UE		0
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M0		12
#define USART_CR1_MME		13
#define USART_CR1_CMIE		14
#define USART_CR1_OVER8		15
#define USART_CR1_DEDT		16
#define USART_CR1_DEAT		21
#define USART_CR1_RTOIE		26
#define USART_CR1_M1		28



/*
 * Bit Position definitions for USART_CR2 Register
 * */
#define USART_CR2_ADDM7		4
#define USART_CR2_LBCL		8
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLKEN		11
#define USART_CR2_STOP		12
#define USART_CR2_SWAP		15
#define USART_CR2_RXINV		16
#define USART_CR2_TXINV		17
#define USART_CR2_DATAINV	18
#define USART_CR2_MSBFIRST	19
#define USART_CR2_ABREN		20
#define USART_CR2_ABRMOD	21
#define USART_CR2_RTOEN		23
#define USART_CR2_ADD1		24
#define USART_CR2_ADD2		28



/*
 * Bit Position definitions for USART_CR3 Register
 * */
#define USART_CR3_EIE		0
#define USART_CR3_HDSEL		3
#define USART_CR3_DMAR		6
#define USART_CR3_DMAT		7
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9
#define USART_CR3_CTSIE		10
#define USART_CR3_ONEBIT	11
#define USART_CR3_OVRDIS	12
#define USART_CR3_DDRE		13
#define USART_CR3_DEM		14
#define USART_CR3_DEP		15



/*
 * Bit Position definitions for USART_RTOR Register
 * */



/*
 * Bit Position definitions for USART_RQR Register
 * */
#define USART_RQR_ABRRQ		0
#define USART_RQR_SBKRQ		1
#define USART_RQR_MMRQ		2
#define USART_RQR_RXFRQ		3



/*
 * Bit Position definitions for USART_ISR Register
 * */
#define USART_ISR_PE		0
#define USART_ISR_FE		1
#define USART_ISR_NF		2
#define USART_ISR_ORE		3
#define USART_ISR_IDLE		4
#define USART_ISR_RXNE		5
#define USART_ISR_TC		6
#define USART_ISR_TXE		7
#define USART_ISR_CTSIF		9
#define USART_ISR_CTS		10
#define USART_ISR_RTOF		11
#define USART_ISR_ABRE		14
#define USART_ISR_ABRF		15
#define USART_ISR_BUSY		16
#define USART_ISR_CMF		17
#define USART_ISR_SBKF		18
#define USART_ISR_RWU		19



/*
 * Bit Position definitions for USART_ICR Register
 * */
#define USART_ICR_PECF		0
#define USART_ICR_FECF		1
#define USART_ICR_NCF		2
#define USART_ICR_ORECF		3
#define USART_ICR_IDLECF	4
#define USART_ICR_TCCF		6
#define USART_ICR_CTSCF		9
#define USART_ICR_RTOCF		11
#define USART_ICR_CMCF		17



#include "stm32f030xx_gpio_driver.h"
#include "stm32f030xx_spi_driver.h"
#include "stm32f030xx_i2c_driver.h"
#include "stm32f030xx_usart_driver.h"


#endif /* INC_STM32F030XX_H_ */

/****************************************** End of File ********************************/




