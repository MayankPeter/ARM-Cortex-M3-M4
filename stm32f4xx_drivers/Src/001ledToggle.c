/*
 * 001ledToggle.c
 *
 *  Created on: Aug 8, 2021
 *      Author: root
 */

#include "stm32f429xx.h"

void delay()
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main()
{
	GPIO_Handle_t GpioLedToggle1;
	GPIO_Handle_t GpioLedToggle2;

	GpioLedToggle1.pGPIOx = GPIOG;
	GpioLedToggle1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLedToggle1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLedToggle1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//GpioLedToggle1.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	GpioLedToggle1.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_OD;
	GpioLedToggle1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GpioLedToggle2 = GpioLedToggle1;
	GpioLedToggle2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;

    GPIO_PeriClockControl(GPIOG, ENABLE);
    GPIO_Init(&GpioLedToggle1);
    GPIO_Init(&GpioLedToggle2);
    while(1)
    {
    	GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_NO_13);
    	GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_NO_14);
    	delay();
    }

    return 0;
}
