/*
 * stm32f030xx_spi_driver.c
 *
 *  Created on: Jun 12, 2020
 *      Author: Jitender Kumar
 *      E-mail: kjitender469@gmail.com
 */


#include "stm32f030xx.h"
#include "stm32f030xx_spi_driver.h"

/*
 * @Note: static keyword is used to make these function private
 * */

static void SPI_TXE_InterruptHandle(SPI_Handle_t *pSPIHandle);		/*  */
static void SPI_RXNE_InterruptHandle(SPI_Handle_t *pSPIHandle);
static void SPI_OVRERR_InterruptHandle(SPI_Handle_t *pSPIHandle);


/*
 * 	Peripheral Clock Setup
 * */

/**********************************************************************************************************
 * @fn				- SPI_PeriClockControl
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
	}
}



/*
 * 	Init and De-Init
 * */

/**********************************************************************************************************
 * @fn				- SPI_Init
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// Configure the SPI_CR1 register
	uint32_t temp_reg = 0;

	// Configure Device Mode
	temp_reg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	// Configure the Bus
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDI(Bi-directional) mode should be cleared
		temp_reg &= ~(1 << 15);

	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDI(Bi-directional) mode should be set
		temp_reg |= 1 << 15;

	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI(Bi-directional) mode should be cleared
		temp_reg &= ~(1 << 15);
		// RXONLY bit should be set
		temp_reg |= 1 << 10;
	}

	// Configure SPI System Clock Speed
	temp_reg |= pSPIHandle->SPI_Config.SPI_SClkSpeed << SPI_CR1_BR;

	// COnfigure the DS(Data Size)
	SPI_Init_DS(pSPIHandle);
	//temp_reg |= pSPIHandle->SPI_Config.SPI_DS << SPI_CR1_CRCL;		// Specific to MCU. In Cortex M0 DS is in CR2 register while in Cortex M4 its name
														// is DFF and it reside in CR1 register.

	// Configure the CPOL
	temp_reg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	// Configure the CPHA
	temp_reg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	// Configure the SSM(Software Slave Management)
	temp_reg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = temp_reg;
}

// SPI_Init_DS() is used because DS bits are reside in CR2 register. Need to test and verify this.
void SPI_Init_DS(SPI_Handle_t *pSPIHandle)
{
	//uint32_t temp = 0;
	//temp |= pSPIHandle->SPI_Config.SPI_DS << SPI_CR2_DS;
	//pSPIHandle->pSPIx->CR2 |= temp;
	pSPIHandle->pSPIx->CR2 = pSPIHandle->SPI_Config.SPI_DS << SPI_CR2_DS;

	// make the FRXTH (FIFO reception threshold to 1/4 (8 bit) default is 1/2(16 bit))
//	uint32_t temp = 0;
//	temp |= 1 << 12;
//	pSPIHandle->pSPIx->CR2 |= temp;
}



/**********************************************************************************************************
 * @fn				- SPI_DeInit
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
}



uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
{
	if( pSPIx->SR & flagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}



/*
 * Data send and receive
 * */

/**********************************************************************************************************
 * @fn				- SPI_SendData
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			- This is a blocking call(Polling type call).
 *
 * *********************************************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length)
{
	while(length > 0)
	{
		// wait until TXE is set. If TXE is set that means Tx buffer is empty now and we can write new value in Tx buffer.
		//while( !(pSPIx->SR & (SPI_SR_TXE << 1)) );
		while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		// check the DFF bit in CR1. In our case(Cortex M0) Its not DFF but it is DS bits(3 bits) in CR2 register
		if(((pSPIx->CR2 & (15 << SPI_CR2_DS)) >> SPI_CR2_DS) == 15)		// Data Size is of 16 bits
		{
			// 16 bit DFF
			// Load the data in DR register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			length--;
			length--;
			(uint16_t*)pTxBuffer++;
		}else if(((pSPIx->CR2 & (15 << SPI_CR2_DS)) >> SPI_CR2_DS) == 7)	// Data Size is of 8 bits
		{
			// 8 bit DFF
			// Load the data in DR register
			//uint32_t xx = &(pSPIx->DR);
			pSPIx->DR = *pTxBuffer;
			length--;
			pTxBuffer++;
		}
		else{
			uint8_t cc = 12;
			cc++;
		}
	}
}



/**********************************************************************************************************
 * @fn				- SPI_ReceiveData
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length)
{
	while(length > 0)
	{
		// wait until RXNE is set. If RXNE is set that means Rx buffer is FUll now and we can read new value from Rx buffer.

		while( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );

		// check the DFF bit in CR1. In our case(Cortex M0) Its not DFF but it is DS bits(3 bits) in CR2 register
		if(((pSPIx->CR2 & (15 << SPI_CR2_DS)) >> SPI_CR2_DS) == 15)		// Data Size is of 16 bits
		{
			// 16 bit DFF
			// Load the data from DR register to Rx buffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			length--;
			length--;
			(uint16_t*)pRxBuffer++;
		}else if(((pSPIx->CR2 & (15 << SPI_CR2_DS)) >> SPI_CR2_DS) == 7)	// Data Size is of 8 bits
		{
			// 8 bit DFF
			// Load the data in DR register
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			length--;
			pRxBuffer++;
		}
		else{
			uint8_t cc = 12;
			cc++;
		}
	}
}



/**********************************************************************************************************
 * @fn				- SPI_SendDataIT
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		// Save the Tx buffer address and length information in some global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLength = length;

		// mark the SPI state as busy is SPI Transmission so that no other code can take over the same
		// SPI peripheral until the transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// Enable the TXEIE control bit to get an interrupt whenever TXE flag is set in SR.
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );
	}

	// Data Transmission will be handled by the ISR code

	return state;
}



/**********************************************************************************************************
 * @fn				- SPI_ReceiveDataIT
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		// Save the Rx buffer address and length information in some global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLength = length;

		// mark the SPI state as busy is SPI Reception so that no other code can take over the same
		// SPI peripheral until the Reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// Enable the RXNEIE control bit to get an interrupt whenever RXNE flag is set in SR.
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
	}

	return state;
}



/*
 * 	IRQ Configuration and ISR handling
 * */

/**********************************************************************************************************
 * @fn				- SPI_IRQInterruptConfig
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		// program ISER register
		*NVIC_ISER |= (1 << IRQNumber);

	}else
	{
		// program ISER register
		*NVIC_ICER |= (1 << IRQNumber);
	}
}



/**********************************************************************************************************
 * @fn				- SPI_IRQPriorityConfig
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void SPI_IRQPriorityConfig( uint8_t IRQNumber, uint32_t IRQPriority)
{
	// find out the IPR register
	uint8_t IPRx = IRQNumber / 4;				/* To identify which IPR register */
	uint8_t IPRx_section = IRQNumber % 4;		/* To identify which section of IPR register is to program. Each section is of 8 bits. */

	uint8_t shift_amount = (8 * IPRx_section) + (8 - NO_PR_BITS_IMPLEMENTED);	/* In Cortex M0 2 bits are implemented. Bit[7:6] bits [5:0] read as zero and ignore writes. */
	*(NVIC_PR_BASE_ADDR + IPRx) |= ( IRQPriority << shift_amount );
}



/**********************************************************************************************************
 * @fn				- SPI_IRQHandling
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	// Check for TXE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	// if temp1 and temp2 are 1 that means Tx Interrupt and Transmission ready(TXE) is enable and we can now
	// start transmission of data to SPI slave.
	if(temp1 && temp2)
	{
		// Handle TXE
		SPI_TXE_InterruptHandle(pHandle);
	}

	// Check for RXNE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		// Handle RXNE
		SPI_RXNE_InterruptHandle(pHandle);
	}

	// Check for OVR Error flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		// Handle OVR Error
		SPI_OVRERR_InterruptHandle(pHandle);
	}

	// Check for CRC Error flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_CRCERR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		// Handle CRCERR
		//SPI_CRCERR_InterruptHandle();
	}
}



/**********************************************************************************************************
 * @fn				- SPI_PeripheralControl
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}



/**********************************************************************************************************
 * @fn				- SPI_SSI_Config
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}



/**********************************************************************************************************
 * @fn				- SPI_SSOE_Config
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}



void SPI1_NSSP_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_NSSP);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_NSSP);
	}
}


void SPI_TXDMAEN(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_TXDMAEN);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_TXDMAEN);
	}
}


void SPI_FRXTH(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_FRXTH);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_FRXTH);
	}
}


void SPI_LSBFIRST(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR1_LSBFIRST);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR1_LSBFIRST);
	}
}


void SPI_CRCL(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR1_CRCL);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR1_CRCL);
	}
}


/*
 * Helper function implementation
 * */
static void SPI_TXE_InterruptHandle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1. In our case(Cortex M0) Its not DFF but it is DS bits(3 bits) in CR2 register
	if(((pSPIHandle->pSPIx->CR2 & (15 << SPI_CR2_DS)) >> SPI_CR2_DS) == 15)		// Data Size is of 16 bits
	{
		// 16 bit DFF
		// Load the data in DR register
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLength--;
		pSPIHandle->TxLength--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else if(((pSPIHandle->pSPIx->CR2 & (15 << SPI_CR2_DS)) >> SPI_CR2_DS) == 7)	// Data Size is of 8 bits
	{
		// 8 bit DFF
		// Load the data in DR register
		//uint32_t xx = &(pSPIx->DR);
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLength--;
		pSPIHandle->pTxBuffer++;
	}
	else{
		uint8_t cc = 12;
		cc++;
	}

	if( ! pSPIHandle->TxLength)
	{
		// TxLength is zero, close the SPI Transmission and inform the application that Tx is over.

		// This prevents Interrupts from setting up of TXE flag.
//		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
//		pSPIHandle->pTxBuffer = NULL;
//		pSPIHandle->TxLength = 0;
//		pSPIHandle->TxState = SPI_READY;

		SPI_CloseTransmission(pSPIHandle);

		// Inform the application that Tx is over via a callback
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);
	}
}


static void SPI_RXNE_InterruptHandle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1. In our case(Cortex M0) Its not DFF but it is DS bits(3 bits) in CR2 register
	if(((pSPIHandle->pSPIx->CR2 & (15 << SPI_CR2_DS)) >> SPI_CR2_DS) == 15)		// Data Size is of 16 bits
	{
		// 16 bit DFF
		// Load the data from DR register to Rx buffer address
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLength--;
		pSPIHandle->RxLength--;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;
	}else if(((pSPIHandle->pSPIx->CR2 & (15 << SPI_CR2_DS)) >> SPI_CR2_DS) == 7)	// Data Size is of 8 bits
	{
		// 8 bit DFF
		// Load the data from DR register to Rx buffer address
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLength--;
		pSPIHandle->pRxBuffer--;
	}
	else{
		uint8_t cc = 12;
		cc++;
	}

	if( ! pSPIHandle->RxLength)
	{
		// RxLength is zero, close the SPI Reception and inform the application that Rx is over.

		// This prevents Interrupts from setting up of RXNE flag.
//		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
//		pSPIHandle->pRxBuffer = NULL;
//		pSPIHandle->RxLength = 0;
//		pSPIHandle->RxState = SPI_READY;

		SPI_CloseReception(pSPIHandle);

		// Inform the application that Tx is over via a callback
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
	}
}


static void SPI_OVRERR_InterruptHandle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	// clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void) temp;
	// Inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void) temp;
}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLength = 0;
	pSPIHandle->TxState = SPI_READY;
}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLength = 0;
	pSPIHandle->RxState = SPI_READY;
}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event)
{
	// This is a weak implementation. The application may override this function.
}


/*********************************** End of File ***************************************************/
