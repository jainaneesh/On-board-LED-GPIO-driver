/*
 * 001LEDToggle.c
 *
 *  Created on: Apr 15, 2023
 *      Author: jaina
 */
#include "stm32f4xx.h"
#include "stm32f407xx_gpio_driver.h"

#define HIGH 1
#define BTN_PRESSED HIGH
void delay(void)
{
	for (uint32_t i=0;i<=500000/2;i++);
}
int main(void)
{
	GPIO_Handle_t GpioLed, GPIOBtn;
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GPIOBtn);

	//IRQ priorities
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);
	//IRQ configurations
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);

	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_12);
}
