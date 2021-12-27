/*
 * stm32f429xx_gpio_driver.h
 *
 *  Created on: Aug 1, 2021
 *      Author: root
 */

#ifndef INC_STM32F429XX_GPIO_DRIVER_H_
#define INC_STM32F429XX_GPIO_DRIVER_H_

#include "stm32f429xx.h"

/*
 * This is Configuration structure for a GPIO pin
 */

typedef struct
{
	uint32_t GPIO_PinNumber;
	uint32_t GPIO_PinMode;
	uint32_t GPIO_PinSpeed;
	uint32_t GPIO_PinPuPdControl;
	uint32_t GPIO_PinOType;
	uint32_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

enum GPIO_MODE
{
	GPIO_MODE_INPUT = 0,
	GPIO_MODE_OUT =  1,
	GPIO_MODE_ALTFN = 2,
	GPIO_MODE_ANALOG = 3,
	GPIO_MODE_IT_FT = 4,
	GPIO_MODE_IT_RT = 5,
	GPIO_MODE_IT_RFT = 6
};

enum GPIO_OP_TYPE
{
  	GPIO_OP_TYPE_PP = 0,
	GPIO_OP_TYPE_OD = 1
};

enum GPIO_SPEED
{
	GPIO_SPEED_LOW = 0,
	GPIO_SPEED_MEDIUM = 1,
	GPIO_SPEED_FAST = 2,
	GPIO_SPEED_HIGH = 3
};

enum GPIO_PULLUP_DOWN
{
	GPIO_NO_PUPD = 0,
	GPIO_PIN_PU = 1,
	GPIO_PIN_PD = 2
};

enum GPIO_PIN_NO
{
	GPIO_PIN_NO_0 = 0,
	GPIO_PIN_NO_1 = 1,
	GPIO_PIN_NO_2 = 2,
	GPIO_PIN_NO_3 = 3,
	GPIO_PIN_NO_4 = 4,
	GPIO_PIN_NO_5 = 5,
	GPIO_PIN_NO_6 = 6,
	GPIO_PIN_NO_7 = 7,
	GPIO_PIN_NO_8 = 8,
	GPIO_PIN_NO_9 = 9,
	GPIO_PIN_NO_10 = 10,
	GPIO_PIN_NO_11 = 11,
	GPIO_PIN_NO_12 = 12,
	GPIO_PIN_NO_13 = 13,
	GPIO_PIN_NO_14 = 14,
	GPIO_PIN_NO_15 = 15
};

/******************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 ******************************************************************************/


/*
 * Peripheral Clock Setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);



/*
 * Peripheral Init & De-Init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);



/*
 * Peripheral Data Read and Write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritetoOutputport(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);



/*
 * Peripheral IRQ Config and ISR Handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F429XX_GPIO_DRIVER_H_ */
