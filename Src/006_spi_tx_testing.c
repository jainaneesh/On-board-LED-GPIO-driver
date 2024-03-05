/*
 * 006_spi_tx_testing.c
 *
 *  Created on: Sep 26, 2023
 *      Author: jaina
 */
#include "stm32f4xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include <string.h>

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);
//
//	//NSS
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//	GPIO_Init(&SPIPins);
}
void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_HIGH;
	SPI2handle.SPIConfig.SPI_SMM = SPI_SMM_EN;   //software slave management enabled for NSS
	SPI_Init(&SPI2handle);
}
int main(void)
{
	char user_data[]="Hello World";
	SPI2_GPIOInits();
	SPI2_Inits();

	//This makes NSS signal internally high
	SPI_SSIConfig(SPI2,ENABLE);
	SPI_PeripheralControl(SPI2,ENABLE);
	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));
	while(1);
	return 0;
}

