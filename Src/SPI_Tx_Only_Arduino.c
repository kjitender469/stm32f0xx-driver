/*
 * SPI_Tx_Only_Arduino.c
 *
 *  Created on: 15-Jun-2020
 *      Author: Jitender Kumar
 */


/*
 * PA6 			--> MISO
 * PA7 			--> MOSI
 * PA5 			--> SCLK
 * PA4 			--> NSS
 * ALT fn mode 	--> AF0
 * */


#include "stm32f030xx.h"
//#include "stm32f030xx_spi_driver.h"
//#include "stm32f030xx_gpio_driver.h"
#include <string.h>


void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

/*
 * SPI1 GPIO pins configuration
 * */
void SPI2_GPIO_Init(void)
{
	GPIO_Handle_t SPI_Pins;
	SPI_Pins.pGPIOx = GPIOB;
	SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 0;			// Configure in AF0
	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

	//

	// SCLK
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPI_Pins);

	// MOSI
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPI_Pins);

	// MISO
//	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
//	GPIO_Init(&SPI_Pins);

	// NSS
	//SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPI_Pins);
}

/*
 * SPI1 peripheral configuration
 * */
void SPI2_Init(void)
{
	SPI_Handle_t SPI1_handle;
	SPI1_handle.pSPIx = SPI2;
	SPI1_handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1_handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1_handle.SPI_Config.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV2;	// generate clock of 4MHz
	SPI1_handle.SPI_Config.SPI_DS = SPI_DS_8BITS;
	SPI1_handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI1_handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI1_handle.SPI_Config.SPI_SSM = SPI_SSM_DISABLE;	// Hardware slave management is enable for NSS pin.

	SPI_Init(&SPI1_handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIO_Btn;
	// For GPIO Button
	GPIO_Btn.pGPIOx = GPIOA;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIO_Btn);
}

int main(void)
{
	char user_data[] = "Hello World How are you";

	GPIO_ButtonInit();

	// this function is used to initialize the GPIO pins to behave as SPI1 pins
	SPI2_GPIO_Init();

	SPI2_Init();

	//SPI1_NSSP_Config(SPI1, ENABLE);

	//SPI_TXDMAEN(SPI1, ENABLE);

	//SPI_FRXTH(SPI1, ENABLE);

	//SPI_LSBFIRST(SPI1, ENABLE);

	//SPI_CRCL(SPI2, ENABLE);


	// this makes NSS signal internally high and avoid MODF(Mode Fault) error
	// SPI_SSI_Config(SPI1, ENABLE);

	// Enable the SSOE(Slave Select Output Enable)
	/*
	 * Making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware.
	 * i.e when SPE=1, NSS will be pulled to LOW.
	 * when SPE=0, NSS will be pulled to HIGH.
	 * */
	SPI_SSOE_Config(SPI2, ENABLE);

	while(1)
	{
		// wait while button is pressed
		while( ! GPIO_ReadFromInuptPin(GPIOA, GPIO_PIN_NO_0) );

		delay();
//		delay();
//		delay();
//		delay();

		// enable SPI1 Peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// first send length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		// enable SPI1 Peripheral
		//SPI_PeripheralControl(SPI2, ENABLE);

		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		// check SPI is not busy
		while( SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) );

		// disable SPI1 Peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
