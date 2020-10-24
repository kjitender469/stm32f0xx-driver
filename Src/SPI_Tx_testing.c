/*
 * SPI_Tx_testing.c
 *
 *  Created on: 14-Jun-2020
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

/*
 * SPI1 GPIO pins configuration
 * */
void SPI1_GPIO_Init(void)
{
	GPIO_Handle_t SPI_Pins;
	SPI_Pins.pGPIOx = GPIOA;
	SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 0;			// Configure in AF0
	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//

	// SCLK
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPI_Pins);

	// MOSI
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPI_Pins);

	// MISO
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPI_Pins);

	// NSS
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPI_Pins);
}

/*
 * SPI1 peripheral configuration
 * */
void SPI1_Init(void)
{
	SPI_Handle_t SPI1_handle;
	SPI1_handle.pSPIx = SPI1;
	SPI1_handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1_handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1_handle.SPI_Config.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV2;	// generate clock of 8MHz
	SPI1_handle.SPI_Config.SPI_DS = SPI_DS_8BITS;
	SPI1_handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI1_handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI1_handle.SPI_Config.SPI_SSM = SPI_SSM_ENABLE;	// Software slave management is enable for NSS pin.

	SPI_Init(&SPI1_handle);
}

int main(void)
{
	char user_data[] = "HIK";

	// this function is used to initialize the GPIO pins to behave as SPI1 pins
	SPI1_GPIO_Init();

	SPI1_Init();

	// this makes NSS signal internally high and avoid MODF(Mode Fault) error
	SPI_SSI_Config(SPI1, ENABLE);

	//SPI_FRXTH(SPI1, ENABLE);

	SPI_LSBFIRST(SPI1, ENABLE);

	// enable SPI1 Peripheral
	SPI_PeripheralControl(SPI1, ENABLE);

	SPI_SendData(SPI1, (uint8_t*)user_data, strlen(user_data));

	while( SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG) );

	// disable SPI1 Peripheral
	SPI_PeripheralControl(SPI1, DISABLE);

	while(1);

	return 0;
}
