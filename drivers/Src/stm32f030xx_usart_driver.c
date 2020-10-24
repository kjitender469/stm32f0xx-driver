/*
 * stm32f030xx_usart_driver.c
 *
 *  Created on: 15-Aug-2020
 *      Author: Jitender Kumar
 */


#include "stm32f030xx.h"



/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{

	//Temporary variable
	uint32_t tempreg=0;

	/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg |= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= (1 << USART_CR1_TE);

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TX_RX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		//tempreg |= ( (1 << USART_CR1_RE) | (1 << USART_CR1_TE);
		tempreg |= ( 1 << USART_CR1_RE );
		tempreg |= ( 1 << USART_CR1_TE );
	}

    //Implement the code to configure the Word length configuration item
	//tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1 ;
	// First Check if WordLength is 7, 8 or 9
	if ( pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_7BITS )
	{
		tempreg |= 1 << USART_CR1_M1 ;
		tempreg &= ~( 1 << USART_CR1_M0 ) ;
	}
	if ( (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS) | (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) )
	{
		tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M0 ;
	}


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enale the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_StopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= ( 1 << USART_CR3_RTSE );

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= ( 1 << USART_CR3_CTSE);
		tempreg |= ( 1 << USART_CR3_RTSE );
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_BaudRate);

}



/**********************************************************************************************************
 * @fn				- USART_DeInit
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
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}
}



/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Resolve all the TODOs

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	uint32_t clk = RCC_GetPCLKValue();

	if ( BaudRate == (uint32_t)USART_STD_BAUD_9600 )
	{
		// I am using direct value for BRR register
		//pUSARTx->BRR = 0x341;		// 2.4kbps
		//pUSARTx->BRR = 0x1A1;		// 2.4kbps
		//pUSARTx->BRR = 0x4E20;	// 2.4kbps
		//pUSARTx->BRR = 0x1388;		// 9.6kbps
		//pUSARTx->BRR = 0x9C4;		// 19.2kbps
		//pUSARTx->BRR = 0x4E2;		// 38.4kbps
		//pUSARTx->BRR = 0x5000;

		//pUSARTx->BRR = 0x341;

		//pUSARTx->BRR = ((uint16_t)(48000000/115200));
		pUSARTx->BRR = ((uint16_t)(8000000/9600));

	}else if ( BaudRate == (uint32_t)USART_STD_BAUD_115200 )
	{
		pUSARTx->BRR = 0x69;
	}else
	{

	}

/*
	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << TODO))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	   TODO
  }

  //Calculate the Mantissa part
  M_part = TODO/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (TODO * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * TODO)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * TODO)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->TODO = tempreg;
  */
}



/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t length)
{

	uint16_t *pdata;
   //Loop over until "length" number of bytes are transferred
	//pUSARTHandle->pUSARTx->ICR |= ( 1 << USART_ICR_TCCF );
	//uint32_t *temp = (uint32_t*)0x40004420U;
	//*temp |= ( 1 << USART_ICR_TCCF );

	for(uint32_t i = 0 ; i < length; i++)
	{
		//uint8_t h = USART_GetFlagStatus( pUSARTHandle->pUSARTx, USART_FLAG_TXE );

		//uint32_t *temp = (uint32_t*)0x40004420U;
		//*temp |= ( 1 << USART_ICR_TCCF );

		//Implement the code to wait until TXE flag is set in the ISR
		//uint32_t c = pUSARTHandle->pUSARTx->ISR;
		//uint8_t n = 1 << USART_ISR_TXE;
		while( ! USART_GetFlagStatus( pUSARTHandle->pUSARTx, USART_FLAG_TXE ) );

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->TDR = ( *pdata & (uint16_t)0x01FF );

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else if ( pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS )
		{
			//This is 8bit data transfer
			//uint32_t d = (*pTxBuffer  & (uint8_t)0xFF);

			uint32_t *temp = (uint32_t*)0x40013828;
			//uint32_t temp2 = *temp;

			//pUSARTHandle->pUSARTx->TDR = (*pTxBuffer  & (uint8_t)0xFF);
			*temp = (*pTxBuffer  & (uint8_t)0xFF);
			//*temp = *pTxBuffer;

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}else
		{
			// 7bit data transfer
			pUSARTHandle->pUSARTx->TDR = (*pTxBuffer  & (uint8_t)0x7F);
			pTxBuffer++;
		}


		//Implement the code to wait until TXE flag is set in the ISR
		//while( ! USART_GetFlagStatus( pUSARTHandle->pUSARTx, USART_FLAG_TXE ) );
	}

	//Implement the code to wait till TC flag is set in the ISR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t length)
{
   //Loop over until "length" number of bytes are transferred
	for(uint32_t i = 0 ; i < length; i++)
	{
		//Implement the code to wait until RXNE flag is set in the ISR
		while( ! USART_GetFlagStatus( pUSARTHandle->pUSARTx, USART_FLAG_RXNE ) );

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if( pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS )
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if( pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE )
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the RDR with 0x01FF
				*( (uint16_t*)pRxBuffer ) = ( pUSARTHandle->pUSARTx->RDR  & (uint16_t)0x01FF );

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = ( pUSARTHandle->pUSARTx->RDR  & (uint8_t)0xFF);

				//Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else if( pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS )
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_Parity control or not
			if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = ( pUSARTHandle->pUSARTx->RDR  & (uint8_t)0xFF);
				 //pRxBuffer++;
			}
			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = ( pUSARTHandle->pUSARTx->RDR & (uint8_t) 0x7F );
				 //pRxBuffer++;
			}

			//increment the pRxBuffer
			pRxBuffer++;
		}else
		{
			//We are going to receive 7bit data in a frame

			//check are we using USART_Parity control or not
			if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 7 bits from DR
				 *pRxBuffer = ( pUSARTHandle->pUSARTx->RDR  & (uint8_t)0x7F);
			}
			else
			{
				//Parity is used, so , 6 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X2F
				 *pRxBuffer = ( pUSARTHandle->pUSARTx->RDR & (uint8_t) 0x2F );
			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_SendData_IT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t length)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = length;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TXEIE );

		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TCIE );

	}

	return txstate;

}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_ReceiveData_IT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t length)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = length;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE );

	}

	return rxstate;

}


/***********************************************************************************************************************************************/



/*
 * 	Peripheral Clock Setup
 * */
/**********************************************************************************************************
 * @fn				- USART_PeriClockControl
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
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
	}
}




/*
 * 	Read the status of flag specified
 * */

/**********************************************************************************************************
 * @fn				- USART_GetFlagStatus
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
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t flagName)
{
	uint32_t x = pUSARTx->ISR;

	uint32_t *temp = 0x4001381cU;
	uint32_t temp2 = *temp;

	uint8_t y = flagName;

	if( temp2 & flagName )
	{
		return FLAG_SET;
	}
	return FLAG_RESET;

//
//	uint32_t *x = 0x4001381c;
//
//	if( x & flagName)
//	{
//		return FLAG_SET;
//	}
//	return FLAG_RESET;

}



/**********************************************************************************************************
 * @fn				- USART_GetFlagStatus
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
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
//	if( pUSARTx->ISR & flagName)
//	{
//		return FLAG_SET;
//	}
//	return FLAG_RESET;
}



/**********************************************************************************************************
 * @fn				- USART_IRQInterruptConfig
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
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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
 * @fn				- USART_IRQPriorityConfig
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
void USART_IRQPriorityConfig( uint8_t IRQNumber, uint32_t IRQPriority)
{
	// find out the IPR register
	uint8_t IPRx = IRQNumber / 4;				/* To identify which IPR register */
	uint8_t IPRx_section = IRQNumber % 4;		/* To identify which section of IPR register is to program. Each section is of 8 bits. */

	uint8_t shift_amount = (8 * IPRx_section) + (8 - NO_PR_BITS_IMPLEMENTED);	/* In Cortex M0 2 bits are implemented. Bit[7:6] bits [5:0] read as zero and ignore writes. */
	*( NVIC_PR_BASE_ADDR + IPRx) |= ( IRQPriority << shift_amount );
}



/*
 * Other Peripheral Control APIs
 * */

/**********************************************************************************************************
 * @fn				- USART_PeripheralControl
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
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}
