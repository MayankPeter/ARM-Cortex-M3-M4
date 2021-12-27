/*
 * 002ledButtonPress.c
 *
 *  Created on: Aug 8, 2021
 *      Author: root
 */


#include "stm32f429xx.h"

void delay(uint32_t value)
{
	for(uint32_t i = 0; i < value; i++);
}

int main()
{
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioBlueButton;

	GpioLed.pGPIOx = GPIOG;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GpioBlueButton.pGPIOx = GPIOA;
	GpioBlueButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioBlueButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBlueButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBlueButton.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	GpioBlueButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOG, ENABLE);
    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&GpioLed);
    GPIO_Init(&GpioBlueButton);
    while(1)
    {
    	if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED)
    	{
    		//delay(50000);
    		//GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_NO_13);
    		GPIO_WriteToOutputPin(GPIOG, GPIO_PIN_NO_13, ENABLE);
    	}
    	else
    	{
    		GPIO_WriteToOutputPin(GPIOG, GPIO_PIN_NO_13, DISABLE);
    	}
    }

    return 0;
}
