/*
 * 008spi_cmd_handling.c
 *
 *  Created on: Feb 7, 2024
 *      Author: jaina
 */
#include "stm32f4xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include <string.h>

/*
 * Command codes
 */
#define COMMAND_LED_CTRL			0x50
#define COMMAND_SENSOR_READ			0x51
#define COMMAND_LED_READ			0x52
#define COMMAND_PRINT				0x53
#define COMMAND_ID_READ				0x54
#define LED_PIN						9
#define LED_ON						1


void GPIO_ButtonInit();
void delay();
void SPI2_GPIOInits();
void SPI2_Inits();
uint8_t SPI_VerifyResponse(uint8_t ackbyte);


int main(void)
{
//	char user_data[]="Hello World";
	uint8_t dummy_write = 0xff;
	uint8_t analogread;
	uint8_t dummy_read;
	uint8_t ackbyte;
	uint8_t args[2];

	SPI2_GPIOInits();
	GPIO_ButtonInit();
	SPI2_Inits();
	//Lets make SSOE bit 1 before enabling the peripheral
	/*
	 * making SSOE bit 1 does NSS output enable
	 * The NSS pin is automatically managed by the hardware.
	 * i.e. when SPE=1, NSS will be pulled low and NSS pin
	 * will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2,ENABLE);
	while(1)
	{
		//Wait until the button is pressed
		while(  GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));//When the button is pressed the function evaluates to true and the while loop breaks
		delay();//To avoid debouncing

		SPI_PeripheralControl(SPI2,ENABLE);

		//Lets send the very first command
		//1. COMMAND_LED_CTRL  <pin no> <value>
		uint8_t commandcode = COMMAND_LED_CTRL;
		SPI_SendData(SPI2,&commandcode,1);

		//Read the garbage data to clear the RXNE flag
		SPI_ReadData(SPI2,&dummy_read,1);

		//Send some dummy bits (1 byte) to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);
		SPI_ReadData(SPI2,&ackbyte,1);

		//Check if we received ACK or NACK
		if( SPI_VerifyResponse(ackbyte) )
		{
			//send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;

			//Send data
			SPI_SendData(SPI2,args,2);

			//SPI receive garbage data
			SPI_ReadData(SPI2,&dummy_read,1);

			//Send garbage data
			SPI_SendData(SPI2,&dummy_write,1);

			//Receive the data
			SPI_ReadData(SPI2,&analogread,1);


		}

		//First lets send length information
//		uint8_t dataLen = strlen(user_data);
		//Send this length data to the slave before sending the actual data
//		SPI_SendData(SPI2,&dataLen,1);
//		SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));
//
//		/*
//		 * Its imperative to check if the data transmission is active.
//		 * If the program moves to SPI Disable whilst data is being transmitted
//		 * it may lead to data corruption.
//		 */
//		//Lets confirm SPI is not busy
//		while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );
//		SPI_PeripheralControl(SPI2,DISABLE);
	}

//	while(1);
	return 0;
}

void GPIO_ButtonInit()
{
	GPIO_Handle_t Gpiobtn;
	Gpiobtn.pGPIOx = GPIOA;
	Gpiobtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	Gpiobtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpiobtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//	Gpiobtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpiobtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//	GPIO_PeriClockControl(GPIOA, ENABLE); //This code is in the GPIO driver
	GPIO_Init(&Gpiobtn);
}

void delay(void)
{
	for (uint32_t i=0;i<=500000/2;i++);
}
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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);
//
//	NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}
void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SMM = SPI_SMM_DI;   //Hardware slave management enabled for NSS
	SPI_Init(&SPI2handle);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		//ack
		return 1;
	}
	return 0;
}

