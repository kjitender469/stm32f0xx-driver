/*
 * stm32f030xx_i2c_driver.c
 *
 *  Created on: 24-Jun-2020
 *      Author: Jitender Kumar
 */


#include "stm32f030xx.h"


uint16_t HCLK_PreScaler[9] = {2, 4, 8, 16, 32, 64, 128, 256, 512};
uint8_t PCLK_PreScaler[8] = {2, 4, 8, 16};


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slave_address);



/**********************************************************************************************************
 * @fn				- I2C_GenerateStartCondition
 *
 * @brief			- helper function for function I2C_masterSendData()
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR2 |= (1 << I2C_CR2_START);
}



/**********************************************************************************************************
 * @fn				- I2C_GenerateStartCondition
 *
 * @brief			- helper function for function I2C_masterSendData()
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR2 |= (1 << I2C_CR2_STOP);
}




/**********************************************************************************************************
 * @fn				- I2C_ExecuteAddressPhase
 *
 * @brief			- helper function for function I2C_masterSendData()
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slave_address)
{
	// left shift slave address by 1 to make space for r/w bit
	slave_address = slave_address << 1;
	slave_address &= ~(1);		// slave address is slave address + r/w bit = 0
	pI2Cx->TXDR = slave_address;
}



/*
 * 	Peripheral Clock Setup
 * */
/**********************************************************************************************************
 * @fn				- I2C_PeriClockControl
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
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
	}
}


uint8_t RCC_GetOutputClock(void)
{
	return 0;
}

/*
 * Function for calculating APB Peripheral clock
 * */
uint32_t RCC_GetPCLKValue(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, hpre, ppre;
	clksrc = (RCC->CFGR >> 2) & 0x3;

	if(clksrc < 0)
	{
		SystemClk = 8000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetOutputClock();
	}

	// Read HPRE Value here
	// HCLK Prescaler
	temp = (RCC->CFGR >> 4) & 0xF;

	if(temp < 8)
	{
		hpre = 1;
	}else
	{
		hpre = HCLK_PreScaler[temp - 8];
	}

	// PCLK Prescaler - PPRE
	temp = (RCC->CFGR >> 8) & 0x7;

	if(temp < 4)
	{
		ppre = 1;
	}else
	{
		ppre = PCLK_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / hpre) / ppre;

	return pclk1;
}



/*
 * 	Init and De-Init
 * */

/**********************************************************************************************************
 * @fn				- I2C_Init
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			- ** Initialization has to be done when I2C Peripheral is Disable.
 * 					- Configure the Mode(Standard or Fast or Fast+)
 * 					- Configure the Speed of the Serial Clock(SCLK)
 * 					- Configure the Device Address
 * 					- Enable the ACK
 * 					- Configure the Rise Time for I2C pins
 *
 * *********************************************************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	// enable the clock for I2Cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//tempreg =;

	// Configure the NACKIE. Not acknowledge received Interrupt enable. By default this is disabled.

	// Configure the device own address
	tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	pI2CHandle->pI2Cx->OAR1 |= tempreg;

	// Enable the OA1
	pI2CHandle->pI2Cx->OAR1 |= (1 << 15);

	// Configure the I2C timing here
	pI2CHandle->pI2Cx->TIMINGR = pI2CHandle->I2C_Config.I2C_SCLTiming;

	// Enable Interrupts

	// Error Interrupt
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ERRIE);

	// Transfer Complete Interrupt
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_TCIE);

	// Stop Detection Interrupt
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOPIE);

	// NACK received interrupt
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_NACKIE);

	// TX interrupt
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_TXIE);

	// Enable AUTOEND
	pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_AUTOEND);
}



/**********************************************************************************************************
 * @fn				- I2C_DeInit
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
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
}



/**********************************************************************************************************
 * @fn				- I2C_SlaveAddressConfig
 *
 * @brief			- Configure the I2C Slave address in I2C_OA1
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void I2C_SlaveAddressConfig(I2C_RegDef_t *pI2Cx, uint8_t slave_address)
{
	// Configure the OA1. OA1 can be configured either in 7-bit mode (by default) or in 10-bit addressing mode.
	//pI2Cx->OAR1 |= (  );

	// Enable the OA1
	pI2Cx->OAR1 |= ( 1 << 15 );

	// Setting the slave address
	pI2Cx->OAR1 |= ( slave_address << 1 );

	// Enable GCEN Bit
	I2C_GCEN_Control(I2C1, ENABLE);
}




/*
 * Data send and receive
 * */
/**********************************************************************************************************
 * @fn				- I2C_MasterSendData
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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slave_address)
{
	//Wait until I2C is busy.
	while(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BUSY) == SET);

	// Set the slave address
	pI2CHandle->pI2Cx->CR2 |= (slave_address << ( I2C_CR2_SADD + 1 ) );

	// Set the number of bytes to be transfered
	pI2CHandle->pI2Cx->CR2 |= (length << I2C_CR2_NBYTES);

	// Enable General Call GCEN
	//pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_GCEN );

//	// Enable Interrupts
//
//	// Error Interrupt
//	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ERRIE);
//
//	// Transfer Complete Interrupt
//	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_TCIE);
//
//	// Stop Detection Interrupt
//	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOPIE);
//
//	// NACK received interrupt
//	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_NACKIE);
//
//	// TX interrupt
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_TXIE);
//
	// Enable AUTOEND
//	pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_AUTOEND);

	// Enable RELOAD
	//pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_RELOAD);

	// Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// Set TC flag (alpha)


	// Confirm that start generation is completed by checking the SB flag.
	// Note: Until SB is cleared SCL will be stretched (pull to LOW)
	/*
	 * Not Applicable
	 * */

	// setting no of bytes transferred in NBYTE register
	//pI2CHandle->pI2Cx->CR2 |= (length << I2C_CR2_NBYTES);

	// Send the address of the slave with r/rw bit set to w(0) ( total 8 bits )
	//I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, slave_address);

	// Confirm that address phase is completed by checking the ADDR flag in the SR register

	// Stop generation
	//pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_STOP );

	// send data until length become zero
	while(length > 0)
	{
		/*
		 * This flag is set by hardware when the I2C_TXDR register is empty.
		 * */
		while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXIS) == RESET ); 		// wait till TXIS is RESET
		pI2CHandle->pI2Cx->TXDR = *pTxBuffer;
		pTxBuffer++;
		length--;


		// Generate start condition
		//I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) == SET ); 		// wait till TXE is reset

		// wait until stop condition is not detected i.e STOPF flag is SET
		//while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF) == SET );

		// Clear the STOP flag by setting STOPCF bit
		//pI2CHandle->pI2Cx->ICR = (1 << I2C_ICR_STOPCF);

		//while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TCR) == SET ); 		// wait till TCR is set



		//pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_STOP );
	}

	// when length becomes zero wait for TXE = 1 and BTF = 1
	// In stm32f0308 wait until TC set to 0
	//while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TC) );

	//I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

	// wait until stop condition is not detected i.e STOPF flag is SET
	//while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF) == SET );

	// Clear the STOP flag by setting STOPCF bit
	//pI2CHandle->pI2Cx->ICR = (1 << I2C_ICR_STOPCF);

	// Generate start condition
	//I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
}



/**********************************************************************************************************
 * @fn				- I2C_MasterReceiveData
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
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t slave_address)
{
	// Check if I2C bus is busy or not
	// Wait until I2C is busy.
	while(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BUSY) == SET);

	// Set the I2C mode (Whether Read or Write)
	pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_RD_WRN );

	// Send the address of the slave with r/w bit set to R(1) (total 8 bits)
	// Set the slave address
	pI2CHandle->pI2Cx->CR2 |= (slave_address << ( I2C_CR2_SADD + 1 ) );

	// Set the number of bytes to be received
	pI2CHandle->pI2Cx->CR2 |= (length << I2C_CR2_NBYTES);

	// Generate the Start Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// wait until the address phase is completed by checking the ADDR flag in the SR1

	// Read the data until length becomes zero
	for( uint32_t i = length; i > 0; i-- )
	{
		/*
		 * This flag is set by hardware when I2C_RXDR is ready to be read.
		 * */
		 while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) == RESET ); 		// wait till RXNE is RESET

		 // Read the data from data register into buffer
		 *pRxBuffer = pI2CHandle->pI2Cx->RXDR;
		 pRxBuffer++;
	}
}



/**********************************************************************************************************
 * @fn				- I2C_MasterSendData_IT
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
uint8_t I2C_MasterSendData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slave_address, uint8_t SR)
{
	uint8_t busy_state = pI2CHandle->TxRxState;

	if( (busy_state != I2C_BUSY_IN_TX) && (busy_state != I2C_BUSY_IN_RX) )
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = length;
		//pI2CHandle->pI2Cx->CR2 |= (length << I2C_CR2_NBYTES);
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DeviceAddress = slave_address;
		// Set the slave address
		//pI2CHandle->pI2Cx->CR2 |= (slave_address << ( I2C_CR2_SADD + 1 ) );
		pI2CHandle->SR = SR;

		// Generate start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable Transmit Interrupt
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_TXIE );

		// Enable Error Interrupt
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_ERRIE );
	}
	return busy_state;
}



/**********************************************************************************************************
 * @fn				- I2C_MasterReceiveData_IT
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
uint8_t I2C_MasterReceiveData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t slave_address, uint8_t SR)
{
	uint8_t busy_state = pI2CHandle->TxRxState;

	if( (busy_state != I2C_BUSY_IN_TX) && (busy_state != I2C_BUSY_IN_RX) )
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = length;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DeviceAddress = slave_address;
		pI2CHandle->SR = SR;

		// Generate start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Generate start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable Transmit Interrupt
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_TXIE );

		// Enable Error Interrupt
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_ERRIE );
	}
	return busy_state;
}



/**********************************************************************************************************
 * @fn				- I2C_SlaveSendData
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
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->TXDR = data;
}



/**********************************************************************************************************
 * @fn				- I2C_SlaveReceiveData
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
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t)pI2C->TXDR;
}



/*
 * 	Read the status of flag specified
 * */

/**********************************************************************************************************
 * @fn				- I2C_GetFlagStatus
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
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName)
{
	if( pI2Cx->ISR & flagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}



/*
 * Other Peripheral Control APIs
 * */

/**********************************************************************************************************
 * @fn				- I2C_PeripheralControl
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
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}



/**********************************************************************************************************
 * @fn				- I2C_GCEN_Control
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
void I2C_GCEN_Control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_GCEN);
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_GCEN);
	}
}



/**********************************************************************************************************
 * @fn				- I2C_SBC_Control
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
void I2C_SBC_Control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_SBC);
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_SBC);
	}
}



/**********************************************************************************************************
 * @fn				- I2C_NOSTRETCH_Control
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
void I2C_NOSTRETCH_Control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == DISABLE)
	{
		// Setting NOSTRETCH bit will disable NOSTRETCH
		pI2Cx->CR1 |= (1 << I2C_CR1_NOSTRETCH);
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_NOSTRETCH);
	}
}



/**********************************************************************************************************
 * @fn				- I2C_ManageAcking
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
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if( EnOrDi == I2C_ACK_ENABLE)
	{
		// Enable the ACK
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_NACK ); 		// This will disable the NACK means ACK is send.
	}else
	{
		// Disable the ACK
		pI2Cx->CR2 |= ( 1 << I2C_CR2_NACK ); 		// This will enable the NACK means ACK is not send.
	}
}



/**********************************************************************************************************
 * @fn				- I2C_IRQInterruptConfig
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
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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
 * @fn				- I2C_IRQPriorityConfig
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
void I2C_IRQPriorityConfig( uint8_t IRQNumber, uint32_t IRQPriority)
{
	// find out the IPR register
	uint8_t IPRx = IRQNumber / 4;				/* To identify which IPR register */
	uint8_t IPRx_section = IRQNumber % 4;		/* To identify which section of IPR register is to program. Each section is of 8 bits. */

	uint8_t shift_amount = (8 * IPRx_section) + (8 - NO_PR_BITS_IMPLEMENTED);	/* In Cortex M0 2 bits are implemented. Bit[7:6] bits [5:0] read as zero and ignore writes. */
	*(NVIC_PR_BASE_ADDR + IPRx) |= ( IRQPriority << shift_amount );
}



/**********************************************************************************************************
 * @fn				- I2C_EV_IRQHandling
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
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	// Interrupt handling for both master and slave device

	uint32_t temp1, temp1_status;
//	temp2, temp3, temp4, temp5, temp6, temp7;
//	uint32_t temp1_status, temp2_status, temp3_status, temp4_status, temp5_status, temp6_status;

	// check whether Interrupt is actually enable or not.
	temp1 = pI2CHandle->pI2Cx->CR1 & ( 1 << I2C_CR1_RXIE );
//	temp2 = pI2CHandle->pI2Cx->CR1 & ( 1 << I2C_CR1_TXIE );
//	temp3 = pI2CHandle->pI2Cx->CR1 & ( 1 << I2C_CR1_ADDRIE );
//	temp4 = pI2CHandle->pI2Cx->CR1 & ( 1 << I2C_CR1_NACKIE );
//	temp5 = pI2CHandle->pI2Cx->CR1 & ( 1 << I2C_CR1_STOPIE );
//	temp6 = pI2CHandle->pI2Cx->CR1 & ( 1 << I2C_CR1_TCIE );
//	temp7 = pI2CHandle->pI2Cx->CR1 & ( 1 << I2C_CR1_ERRIE );

	// get the corresponding flag in status register
	temp1_status = pI2CHandle->pI2Cx->ISR & ( 1 << I2C_ISR_RXNE );
//	temp2_status = pI2CHandle->pI2Cx->ISR & ( 1 << I2C_ISR_TXIS );
//	temp3_status = pI2CHandle->pI2Cx->ISR & ( 1 << I2C_ISR_ADDR );
//	temp4_status = pI2CHandle->pI2Cx->ISR & ( 1 << I2C_ISR_NACKF );
//	temp5_status = pI2CHandle->pI2Cx->ISR & ( 1 << I2C_ISR_STOPF);
//	temp6_status = pI2CHandle->pI2Cx->ISR & ( 1 << I2C_ISR_TC );
	//temp1_status = pI2CHandle->pI2Cx->ISR & ( 1 << I2C_ISR_ );

	// Handle for interrupt generated by RXNE event
	if( temp1 && temp1_status )
	{
		/*
		 * By default, it operates in slave mode. The interface automatically switches from slave to master
		 * when it generates a START condition, and from master to slave if an arbitration loss or a STOP
		 * generation occurs
		 * */
		if( pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_START ) )
		{
			// If start condition detected that means Device is in Master Mode
			// confirm that RXNE flag is set
			if(pI2CHandle->RxLen > 0)
			{
				if( pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
				{
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->RXDR;
					pI2CHandle->pRxBuffer++;
					pI2CHandle->RxLen--;
				}
			}

			// close the I2C data reception and notify the application

			// 1. Generate the stop condition
			if( pI2CHandle->SR == I2C_DISABLE_SR )
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			// 2. Close the I2C Rx
			I2C_CloseReceiveData(pI2CHandle);

			// 3. Notify the application
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);

		}else
		{
			// Slave Mode confirm
			if( pI2CHandle->pI2Cx->ISR & ( 1 << I2C_ISR_DIR ) )
			{
				// Device in transmitter mode confirmed

				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}

	}

	temp1 = pI2CHandle->pI2Cx->CR1 & ( 1 << I2C_CR1_TXIE );
	temp1_status = pI2CHandle->pI2Cx->ISR & ( 1 << I2C_ISR_TXIS );

	// Handle for interrupt generated by TXIS event
	if( temp1 && temp1_status )
	{
		/*
		 * By default, it operates in slave mode. The interface automatically switches from slave to master
		 * when it generates a START condition, and from master to slave if an arbitration loss or a STOP
		 * generation occurs
		 * */
		if( pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_START ) )
		{
			// If start condition detected that means Device is in Master Mode

			/*
			 * confirm that TXIS flag is set. This means the I2C_TXDR register is empty and the data to be
			 * transmitted must be written in the I2C_TXDR register. It is cleared when the next data to be
			 * sent is written in the I2C_TXDR register
			 * */
			if( pI2CHandle->TxRxState == I2C_BUSY_IN_TX )
			{
				if( pI2CHandle->TxLen > 0 )
				{
					// Load the data into DR
					pI2CHandle->pI2Cx->TXDR = *(pI2CHandle->pTxBuffer);

					// Decrement the TxLen
					pI2CHandle->TxLen--;

					// Increment the buffer address
					pI2CHandle->pTxBuffer++;
				}
			}

			// close the I2C data transmission and notify the application

			// 1. Generate the stop condition
			if( pI2CHandle->SR == I2C_DISABLE_SR )
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			// 2. Close the I2C Tx
			I2C_CloseSendData(pI2CHandle);

			// 3. Notify the application
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);

		}else{
			// Device is in Master Mode
			if( pI2CHandle->pI2Cx->ISR & ( 1 << I2C_ISR_DIR ) )
			{
				// Device in transmitter mode confirmed

				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	temp1 = pI2CHandle->pI2Cx->CR1 & ( 1 << I2C_CR1_ADDRIE );
	temp1_status = pI2CHandle->pI2Cx->ISR & ( 1 << I2C_ISR_ADDR );

	// Handle for interrupt generated by ADDR event
	if( temp1 && temp1_status )
	{
		// confirm that ADDR flag is set

		// Get the received Address
		uint8_t address = pI2CHandle->pI2Cx->ISR & 0x4F;

		pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_ADDRCF );
	}

	temp1 = pI2CHandle->pI2Cx->CR1 & ( 1 << I2C_CR1_NACKIE );
	temp1_status = pI2CHandle->pI2Cx->ISR & ( 1 << I2C_ISR_NACKF );

	// Handle for interrupt generated by NACKF event
	if( temp1 && temp1_status )
	{
		// confirm that NACKF flag is set
	}

	temp1 = pI2CHandle->pI2Cx->CR1 & ( 1 << I2C_CR1_STOPIE );
	temp1_status = pI2CHandle->pI2Cx->ISR & ( 1 << I2C_ISR_STOPF );

	// Handle for interrupt generated by STOPF event
	if( temp1 && temp1_status )
	{
		// confirm that STOPF flag is set
		pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_STOPCF );
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp1 = pI2CHandle->pI2Cx->CR1 & ( 1 << I2C_CR1_TCIE );
	temp1_status = pI2CHandle->pI2Cx->ISR & ( 1 << I2C_ISR_TC );

	// Handle for interrupt generated by TC event
	if( temp1 && temp1_status )
	{
		// confirm that TC flag is set
	}
}



/**********************************************************************************************************
 * @fn				- I2C_ER_IRQHandling
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
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

	// know the status of ERRIE control bit in the CR1
	temp2 = ( pI2CHandle->pI2Cx->CR1 ) & ( 1 << I2C_CR1_ERRIE );

	/*
	 * Check for bus Error
	 * */
	temp1 = ( pI2CHandle->pI2Cx->ISR ) & ( 1 << I2C_ISR_BERR );

	if( temp1 && temp2 )
	{
		// Bus error is confirmed

		// clear the bus error
		pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_BERRCF );

		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	/*
	 * Check for arbitration lost error
	 * */
	temp1 = ( pI2CHandle->pI2Cx->ISR ) & ( 1 << I2C_ISR_ARLO );

	if( temp1 && temp2 )
	{
		// Arbitration lost error is confirmed

		// clear the arbitration lost error
		pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_ARLOCF );

		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	/*
	 * Check for ACK failure error
	 * */
	temp1 = ( pI2CHandle->pI2Cx->ISR ) & ( 1 << I2C_ISR_NACKF );

	if( temp1 && temp2 )
	{
		// ACK failure error is confirmed

		// clear the ACK failure error
		pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_NACKCF );

		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_NACKF);
	}

	/*
	 * Check for overrun/underrun error
	 * */
	temp1 = ( pI2CHandle->pI2Cx->ISR ) & ( 1 << I2C_ISR_OVR );

	if( temp1 && temp2 )
	{
		// overrun/underrun error is confirmed

		// clear the overrun/underrun error
		pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_OVRCF );

		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	/*
	 * Check for timeout error
	 * */
	temp1 = ( pI2CHandle->pI2Cx->ISR ) & ( 1 << I2C_ISR_TIMEOUT );

	if( temp1 && temp2 )
	{
		// timeout error is confirmed

		// clear the timeout error
		pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_TIMOUTCF );

		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}



void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	// Disable the interrupt
	pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_RXIE );

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
}



void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// Disable the interrupt
	pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_TXIE );

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}



void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if( EnOrDi == ENABLE )
	{
		pI2Cx->CR1 |= ( 1 << I2C_CR1_TXIE );
		pI2Cx->CR1 |= ( 1 << I2C_CR1_RXIE );
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ADDRIE );
		pI2Cx->CR1 |= ( 1 << I2C_CR1_NACKIE );
		pI2Cx->CR1 |= ( 1 << I2C_CR1_TCIE );
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ERRIE );
		pI2Cx->CR1 |= ( 1 << I2C_CR1_STOPIE );
	}else
	{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_TXIE );
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_RXIE );
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ADDRIE );
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_NACKIE );
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_TCIE );
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ERRIE );
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_STOPIE );
	}
}





