/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jun 26, 2023
 *      Author: Aneesh Jain
 */
#include "stm32f407xx_spi_driver.h"

/*
 * SPI  Enabling Peripheral Clock
 */
/******************************************************************************
 *  @fn						: SPI_PeriClockControl
 *
 *  @brief					: This function enables or disables peripheral clock for the given SPI port
 *
 *  @param[in]				: Base address of the SPI Peripheral
 *  @param[in]				: ENABLE or DISABLE macros
 *
 *  @return					: none
 *  @Note					: none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
		}
		else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}
		}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if( pSPIx->SR & FlagName )
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*
 * SPI Init and De-Init
 */
/******************************************************************************
 *  @fn						: SPI_Init
 *
 *  @brief					: This function initializes the given SPI port and
 *  						  the given GPIO pin. This function configures its
 *  						  mode, speed, output type, pull-up pull-down resister
 *  						  configuration and, alternate functionality
 *
 *  @param[in]				: Base address of the GPIO Peripheral
 *
 *  @return					: none
 *  @Note					: none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//First lets configure the SPI_CR1 register
	uint32_t tempreg = 0;
	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

	//1. Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_MASTER;

	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI Mode should be cleared
		tempreg &= ~(1 << SPI_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI Mode should be enabled
		tempreg |= (1 << SPI_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI Mode should be cleared
		tempreg &= ~(1 << SPI_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_RX_ONLY);
	}
	//3. Setting the SPI Serial Clock Speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_BR;

	//4. Setting the SPI Data Frame Format
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_DFF_BIT;

	//5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CPOL_BIT;

	//6. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CPHA_BIT;

	//Assigning tempreg value to CR1 register
	pSPIHandle->pSPIx->CR1 = tempreg;


}


void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. Wait until TXE is set
		while( SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET );

		//2. Check the DFF bit in CR1
		if ( pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			//16 bit DFF
			//1. Load the data into the data register DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			//1. Load the data into the data register DR
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. Wait until RXNE is set
		while( SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET );

		//2. Check the DFF bit in CR1
		if ( pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			//16 bit DFF
			//1. Load the data from DR into the RX buffer
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else
		{
			//8 bit DFF
			//1. Load the data from DR into the RX buffer
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}

	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
