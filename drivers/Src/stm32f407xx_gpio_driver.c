/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Apr 13, 2023
 *      Author: jaina
 */
#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral Clock Setup
 */
/******************************************************************************
 *  @fn						: GPIO_PeriClockControl
 *
 *  @brief					: This function enables or disables peripheral clock for the given GPIO port
 *
 *  @param[in]				: Base address of the GPIO Peripheral
 *  @param[in]				: ENABLE or DISABLE macros
 *
 *  @return					: none
 *  @Note					: none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

/*
 * GPIO Init and De-Init
 */
/******************************************************************************
 *  @fn						: GPIO_Init
 *
 *  @brief					: This function initializes the given GPIO port and
 *  						  the given GPIO pin. This function configures its
 *  						  mode, speed, output type, pull-up pull-down resister
 *  						  configuration and, alternate functionality
 *
 *  @param[in]				: Base address of the GPIO Peripheral
 *
 *  @return					: none
 *  @Note					: none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);
	//1 Configure the mode of the GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;

	}else
	{
		//this part will code later
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure the falling edge trigger selection register FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2. Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure the rising edge trigger selection register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2. Clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. Configure both falling edge and rising edge trigger selection register
			// Configure the RTSR register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Configure the FTSR register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. Configure the GPIO port selection in SYSCFG_EXTICR (sys config exti control register)
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = ( portcode << temp2 * 4);

		//3. Enable the EXTI interrupt delivery using IMR (Intrrupt Mask Register)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;
	//2 Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//3 Configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;
	//4 Configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	//5 Configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure this alt function
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}
}
/******************************************************************************
 *  @fn						: GPIO_DeInit
 *
 *  @brief					: This function de-initializes or resets the given GPIO port
 *
 *  @param[in]				: Base address of the GPIO Peripheral
 *
 *  @return					: none
 *  @Note					: none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}else if (pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}else if (pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}else if (pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}else if (pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}else if (pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}else if (pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET();
			}else if (pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}else if (pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET();
			}
}

/*
 * Data read and write
 */
/******************************************************************************
 *  @fn						: GPIO_ReadFromInputPin
 *
 *  @brief					: This function is used to read from the given Pin
 *
 *  @param[in]				: Base address of the GPIO Peripheral
 *  @param[in]				: Pin number
 *
 *  @return					: Boolean value - either 0 or 1
 *  @Note					: none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
/******************************************************************************
 *  @fn						: GPIO_ReadFromInputPort
 *
 *  @brief					: This function is used to read from the given port
 *
 *  @param[in]				: Base address of the GPIO Peripheral
 *
 *  @return					: 16 bits information from the GPIO's register.
 *  @Note					: none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;

}
/******************************************************************************
 *  @fn						: GPIO_WriteToOutputPin
 *
 *  @brief					: This function is used to write to the given pin
 *
 *  @param[in]				: Base address of the GPIO Peripheral
 *  @param[in]				: Pin number
 *  @param[in]				: value will either be 0 or 1
 *
 *  @return					: 16 bits information from the GPIO's register.
 *  @Note					: none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}
/******************************************************************************
 *  @fn						: GPIO_WriteToOuputPort
 *
 *  @brief					: This function is used to write to the given port
 *
 *  @param[in]				: Base address of the GPIO Peripheral
 *  @param[in]				: value will either be 0 or 1
 *
 *  @return					: none
 *  @Note					: none
 */
void GPIO_WriteToOuputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value)
{
	pGPIOx->ODR = Value;
}
/******************************************************************************
 *  @fn						: GPIO_ToggleOutputPin
 *
 *  @brief					: This function is used to toggle the pin ON or OFF
 *
 *  @param[in]				: Base address of the GPIO Peripheral
 *  @param[in]				: pin number
 *
 *  @return					: none
 *  @Note					: none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if (IRQNumber > 31 && IRQNumber < 64 )
		{
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));

		}
	}else
	{
		if (IRQNumber <= 31)
		{
			//Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if (IRQNumber > 31 && IRQNumber < 64 )
		{
			//Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}

}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. First find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the EXTI PR register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber))
	{
		//Clear the PR register bit
		EXTI->PR |= (1 << PinNumber);
	}

}
