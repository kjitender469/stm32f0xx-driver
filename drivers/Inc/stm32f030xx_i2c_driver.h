/*
 * stm32f030xx_i2c_driver.h
 *
 *  Created on: 24-Jun-2020
 *      Author: Jitender Kumar
 */

#ifndef INC_STM32F030XX_I2C_DRIVER_H_
#define INC_STM32F030XX_I2C_DRIVER_H_


#include "stm32f030xx.h"


/*
 * Configuration structure for I2Cx peripheral
 * */
typedef struct
{
	//uint32_t I2C_SCLSpeed;			/* I2C Modes 			@I2C_SCLSpeed */
	uint32_t I2C_SCLTiming;				/* I2C Modes 							  */
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;			/* I2C NACK Control		 @I2C_ACKControl*/
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;


/*
 * Handle structure for I2Cx peripheral
 * */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t		 *pTxBuffer;
	uint8_t		 *pRxBuffer;
	uint32_t	 TxLen;
	uint32_t	 RxLen;
	uint8_t 	 TxRxState;			/* to store communication state */
	uint8_t 	 DeviceAddress;		/* to store slave/device address */
	uint32_t	 RxSize;
	uint8_t		 SR;				/* to store repeated start value */
}I2C_Handle_t;


/*
 * I2C application states
 * */
#define	I2C_READY			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2


/*
 * @I2C_SCLSpeed
 * */
#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM4K		400000



/*
 * @I2C_ACKControl
 * */
#define I2C_ACK_ENABLE				1
#define I2C_ACK_DISABLE				0

#define I2C_NACKIE_FLAG_ENABLE		1
#define I2C_NACKIE_FLAG_DISABLE		0



/*
 * @I2C_FMDutyCycle
 * */
#define I2C_FM_DUTY_



/*
 * I2C related status flags definitions
 * */
#define I2C_FLAG_TXE		( 1 << I2C_ISR_TXE )
#define I2C_FLAG_TXIS		( 1 << I2C_ISR_TXIS )
#define I2C_FLAG_RXNE		( 1 << I2C_ISR_RXNE )
#define I2C_FLAG_ADDR		( 1 << I2C_ISR_ADDR )
#define I2C_FLAG_NACKF		( 1 << I2C_ISR_NACKF )
#define I2C_FLAG_STOPF		( 1 << I2C_ISR_STOPF )
#define I2C_FLAG_TC			( 1 << I2C_ISR_TC )
#define I2C_FLAG_TCR		( 1 << I2C_ISR_TCR )
#define I2C_FLAG_BERR		( 1 << I2C_ISR_BERR )
#define I2C_FLAG_ARLO		( 1 << I2C_ISR_ARLO )
#define I2C_FLAG_OVR		( 1 << I2C_ISR_OVR )
#define I2C_FLAG_TIMEOUT	( 1 << I2C_ISR_TIMEOUT )
#define I2C_FLAG_BUSY		( 1 << I2C_ISR_BUSY )
#define I2C_FLAG_DIR		( 1 << I2C_ISR_DIR )


#define I2C_DISABLE_SR		RESET
#define I2C_ENABLE_SR		SET


/*
 * I2C application events macros
 * */
#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2
//#define

/*
 * I2C Error events macros
 * */
#define I2C_ERROR_BERR  	3
#define I2C_ERROR_ARLO  	4
#define I2C_ERROR_NACKF    	5
#define I2C_ERROR_OVR   	6
#define I2C_ERROR_TIMEOUT 	7
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV		9



/**********************************************************************************************
 * 								APIs supported by this driver
 * 				For more information about the APIs check the function definitions
 * ********************************************************************************************/

/*
 * Peripheral Clock Setup
 * */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);


/*
 * General Call Address For Slave Mode
 * */
void I2C_GCEN_Control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);


/*
 * Slave Byte Control Mode
 * */
void I2C_SBC_Control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);


/*
 * Clock Stretch Control
 * */
void I2C_NOSTRETCH_Control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);


/*
 * 	Init and De-Init
 * */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_SlaveAddressConfig(I2C_RegDef_t *pI2Cx, uint8_t slave_address);



/*
 * Data send and receive
 * */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slave_address);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slave_address);

uint8_t I2C_MasterSendData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slave_address, uint8_t SR);
uint8_t I2C_MasterReceiveData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slave_address, uint8_t SR);


void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);


void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);



/*
 *	IRQ Configuration and ISR handling
 * */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig( uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);


/*
 * Other Peripheral Control APIs
 * */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);



/*
 * Read the status of flag specified
 * */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName);



/*
 * Application Callback
 * */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event);


#endif /* INC_STM32F030XX_I2C_DRIVER_H_ */




