/*
 * stm32f030xx_spi_driver.h
 *
 *  Created on: Jun 12, 2020
 *      Author: Jitender Kumar
 *      E-mail: kjitender469@gmail.com
 */

#ifndef INC_STM32F030XX_SPI_DRIVER_H_
#define INC_STM32F030XX_SPI_DRIVER_H_


#include "stm32f030xx.h"



/*
 * 	This is a configuration structure for SPI pins
 * */
typedef struct
{
	uint8_t SPI_DeviceMode;			/* refer for possible device modes 						@SPI_DeviceMode */
	uint8_t SPI_BusConfig;			/* refer for possible bus config   						@SPI_BusConfig */
	uint8_t SPI_SClkSpeed;			/* System Clock Speed refer for possible clock speed   	@SPI_SClkSpeed */
	uint8_t SPI_DS;					/* Data Size refer for possible Data format				@SPI_DS */
	uint8_t SPI_CPOL;				/* Clock Polarity 					@SPI_CPOL */
	uint8_t SPI_CPHA;				/* Clock Phase  					@SPI_CPHA */
	uint8_t SPI_SSM;				/* Software Select Management		@SPI_SSM */

}SPI_Config_t;



/*
 * 	This is a Handle structure for SPI pin
 * */
typedef struct
{
	SPI_RegDef_t 	*pSPIx;				/* holds the base addresses of SPI */
	SPI_Config_t  	SPI_Config;
	uint8_t			*pTxBuffer;			/* To store the application Tx address */
	uint8_t			*pRxBuffer;			/* To store the application Rx address */
	uint32_t		TxLength;			/* To store Tx length */
	uint32_t		RxLength;			/* To store Rx length */
	uint8_t			TxState;			/* To store Tx state */
	uint8_t			RxState;			/* To store Rx state */
}SPI_Handle_t;



/*
 * SPI Application State
 * */
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2


/*
 * Possible SPI Application Events
 * */
#define SPI_EVENT_TX_COMPLETE		1
#define SPI_EVENT_RX_COMPLETE		2
#define SPI_EVENT_OVR_ERR			3
#define SPI_EVENT_CRC_ERR			4



/*
 * @SPI_DeviceMode
 * */
#define SPI_DEVICE_MODE_SLAVE		0
#define SPI_DEVICE_MODE_MASTER		1


/*
 * @SPI_BusConfig
 * */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3


/*
 * @SPI_SClkSpeed
 * */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7


/*
 * @SPI_DS
 * */
#define SPI_DS_4BITS		3
#define SPI_DS_5BITS		4
#define SPI_DS_6BITS		5
#define SPI_DS_7BITS		6
#define SPI_DS_8BITS		7
#define SPI_DS_9BITS		8
#define SPI_DS_10BITS		9
#define SPI_DS_11BITS		10
#define SPI_DS_12BITS		11
#define SPI_DS_13BITS		12
#define SPI_DS_14BITS		13
#define SPI_DS_15BITS		14
#define SPI_DS_16BITS		15

#define SPI_DS_8BITS_8Bit		0
#define SPI_DS_16BITS_16Bit		1


/*
 * @SPI_CPOL
 * */
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0


/*
 * @SPI_CPHA
 * */
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0


/*
 * SPI_SSM
 * */
#define SPI_SSM_DISABLE		0
#define SPI_SSM_ENABLE		1


/*
 * SPI related status flags definitions
 * */
#define SPI_RXNE_FLAG		( 1 << SPI_SR_RXNE )
#define SPI_TXE_FLAG		( 1 << SPI_SR_TXE )
#define SPI_CRCERR_FLAG		( 1 << SPI_SR_CRCERR )
#define SPI_MODF_FLAG		( 1 << SPI_SR_MODF )
#define SPI_OVR_FLAG		( 1 << SPI_SR_OVR )
#define SPI_BSY_FLAG		( 1 << SPI_SR_BSY )
#define SPI_FRE_FLAG		( 1 << SPI_SR_FRE )
#define SPI_FRLVL_FLAG		( 1 << SPI_SR_FRLVL )
#define SPI_FTLVL_FLAG		( 1 << SPI_FTLVL_TXE )



/**********************************************************************************************
 * 								APIs supported by this driver
 * 				For more information about the APIs check the function definitions
 * ********************************************************************************************/

/*
 * 	Peripheral Clock Setup
 * */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);



/*
 * 	Init and De-Init
 * */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_Init_DS(SPI_Handle_t *pSPIHandle);



/*
 * Data send and receive
 * */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length);



/*
 * 	IRQ Configuration and ISR handling
 * */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig( uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


/*
 * Other Peripheral Control APIs
 * */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);


/*
 * 	SSI(Internal Slave Select) Configuration
 * */
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);



/*
 * 	SSOE(Slave Select Output Enable) Configuration
 * */
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);


void SPI1_NSSP_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_TXDMAEN(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_FRXTH(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_LSBFIRST(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_CRCL(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);


/*
 * 	Read the status of flag specified
 * */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


/*
 * Application Callback
 * */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event);


#endif /* INC_STM32F030XX_SPI_DRIVER_H_ */
