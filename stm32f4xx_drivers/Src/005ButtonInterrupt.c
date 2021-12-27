/*
 * main.c
 *
 *  Created on: Aug 15, 2021
 *      Author: root
 */


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
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GpioBlueButton.pGPIOx = GPIOA;
	GpioBlueButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBlueButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBlueButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBlueButton.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	GpioBlueButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOG, ENABLE);
    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&GpioLed);
    GPIO_Init(&GpioBlueButton);

    GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, 15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

    while(1)
    {

    }

    return 0;
}

void EXTI0_IRQHandler(void)
{
	//handle the GPIO interrupt
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_NO_13);
}
