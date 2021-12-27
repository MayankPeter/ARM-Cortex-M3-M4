/*
 * stm32f429xx.h
 *
 *  Created on: Jul 28, 2021
 *      Author: root
 */

#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_

#include<stdint.h>

#define __vo volatile

/*********************************START: Processor Specific Details*******************************/
/*
 *
 * Processor Peripherals
 *
 */

/*
 * NVIC Register Addresses
 */
#define NVIC_ISER0		( (__vo uint32_t *)0xE000E100 )
#define NVIC_ISER1		( (__vo uint32_t *)0xE000E104 )
#define NVIC_ISER2		( (__vo uint32_t *)0xE000E108 )
#define NVIC_ISER3		( (__vo uint32_t *)0xE000E10C )

#define NVIC_ICER0		( (__vo uint32_t *)0xE000E180 )
#define NVIC_ICER1		( (__vo uint32_t *)0xE000E184 )
#define NVIC_ICER2		( (__vo uint32_t *)0xE000E188 )
#define NVIC_ICER3		( (__vo uint32_t *)0xE000E18C )

#define NVIC_PR_BASE_ADDR		( (__vo uint32_t *)0xE000E400 )
#define NO_PR_BITS_IMPLEMENTED  4

/*********************************END: Processor Specific Details*******************************/



/*#############################################################################################*/



/*********************************START: Controller Specific Details*******************************/

/*
 *
 * base address of Flash and SRAM memories
 *
 */

#define FLASH_BASEADDR   			 0x08000000U
#define SRAM1_BASEADDR				 0x20000000U //112KB
#define SRAM2_BASEADDR				 0x2001C000U //16KB
#define SRAM3_BASEADDR				 0x20020000U //64KB
#define ROM							 0x1FFF0000U
#define SRAM						 SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base address
 */

#define PERIPH_BASEADDR				 0x40000000U
#define APB1PERIPH_BASEADDR			 PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			 0x40010000U
#define AHB1PERIPH_BASEADDR			 0x40020000U
#define AHB2PERIPH_BASEADDR			 0x50000000U

/*
 * Base address of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2800)

#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base address of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)
#define UART7_BASEADDR				(APB1PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR				(APB1PERIPH_BASEADDR + 0x7C00)

/*
 * Base address of peripherals which are hanging on APB2 bus
 */


#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR				(APB2PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR				(APB2PERIPH_BASEADDR + 0x5400)

#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)

#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)

#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)

/*********************peripheral register definition structures ********************/
/*
 * Note: Registers of a peripheral are specific to MCU
 * e.g. : Number of registers of SPI peripheral of STM32F4xx family of MCUs may be different(more or less)
 * Compared to number of Registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check youjr device TRM
 */

/*
 * GPIO registers
 */

typedef struct GPIO_RegDef
{
  __vo uint32_t	MODER;   /* GPIO port mode register */
  __vo uint32_t	OTYPER;  /* GPIO port output type register */
  __vo uint32_t	OSPEEDR; /* GPIO port output speed register */
  __vo uint32_t	PUPDR;   /* GPIO port pull-up/pull-down register */
  __vo uint32_t	IDR;	 /* GPIO port input data register */
  __vo uint32_t	ODR;	 /* GPIO port output data register */
  __vo uint32_t	BSRR;    /* GPIO port bit set/reset register */
  __vo uint32_t	LCKR;    /* GPIO port configuration lock register */
  __vo uint32_t	AFR[2];  /* AFR[0]-GPIO alternate function low register,AFR[1]-GPIO alternate function low register */
}GPIO_RegDef_t;


/*
 * SPI registers
 */

typedef struct SPI_RegDef
{
  __vo uint32_t	CR1;   /* SPI Control register 1 */
  __vo uint32_t	CR2;  /* SPI Control register 2  */
  __vo uint32_t	SR; /* SPI Status register */
  __vo uint32_t	DR;   /* SPI Data register */
  __vo uint32_t	CRCPR;	 /* SPI CRC Polynomial register */
  __vo uint32_t	RXCRCR;
  __vo uint32_t	TXCRCR;
  __vo uint32_t	I2SCFGR;
  __vo uint32_t	I2SPR;
}SPI_RegDef_t;


/*
 * RCC registers
 */

typedef struct RCC_RegDef
{
  __vo uint32_t CR;
  __vo uint32_t PLLCFGR;
  __vo uint32_t CFGR;
  __vo uint32_t CIR;
  __vo uint32_t AHB1RSTR;
  __vo uint32_t AHB2RSTR;
  __vo uint32_t AHB3RSTR;
  __vo uint32_t RESERVED0;
  __vo uint32_t APB1RSTR;
  __vo uint32_t APB2RSTR;
  __vo uint32_t RESERVED1[2];
  __vo uint32_t AHB1ENR;
  __vo uint32_t AHB2ENR;
  __vo uint32_t AHB3ENR;
  __vo uint32_t RESERVED2;
  __vo uint32_t APB1ENR;
  __vo uint32_t APB2ENR;
  __vo uint32_t RESERVED3[2];
  __vo uint32_t AHB1LPENR;
  __vo uint32_t AHB2LPENR;
  __vo uint32_t AHB3LPENR;
  __vo uint32_t RESERVED4;
  __vo uint32_t APB1LPENR;
  __vo uint32_t APB2LPENR;
  __vo uint32_t RESERVED5[2];
  __vo uint32_t BDCR;
  __vo uint32_t CSR;
  __vo uint32_t RESERVED6[2];
  __vo uint32_t SSCGR;
  __vo uint32_t PLLI2SCFGR;
  __vo uint32_t PLLSAICFGR;
  __vo uint32_t DCKCFGR;
}RCC_RegDef_t;

/*
 * SYS_CFG registers
 */

typedef struct SYSCFG_RegDef
{
	__vo uint32_t MEMRMP;				//SYSCFG memory remap register									Address offset: 0x00
	__vo uint32_t PMC;					//SYSCFG peripheral mode configuration register					Address offset: 0x04
	__vo uint32_t EXTICR[4];			//SYSCFG external interrupt configuration register 1,2,3,4		Address offset: 0x08-0x14
	uint32_t RESERVED1[2];				//RESERVED														Address offset:	0x18-0X1C
	__vo uint32_t CMPCR;				//Compensation cell control register							Address offset: 0x20
	uint32_t RESERVED[2];				//RESERVED														Address offset: 0x24-0x28
	__vo uint32_t CFGR;
}SYSCFG_RegDef_t;


/*
 * EXTI registers
 */

typedef struct EXTI_RegDef
{
  __vo uint32_t	IMR;  	 /* Interrupt Mask register */
  __vo uint32_t	EMR;  	 /* Event Mask register */
  __vo uint32_t	RTSR;    /* Rising Trigger selection register */
  __vo uint32_t	FTSR;  	 /* Falling Trigger selection register */
  __vo uint32_t	SWIER;	 /* Software interrupt event register */
  __vo uint32_t	PR;	 	 /* Pending register */
}EXTI_RegDef_t;

/*
 * CLOCK ENABLE MACROS
 */

/*
 * peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ			((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK			((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define RCC				((RCC_RegDef_t *)RCC_BASEADDR)
#define EXTI			((EXTI_RegDef_t *)EXTI_BASEADDR)
#define SYSCFG          ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4			((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5			((SPI_RegDef_t*)SPI5_BASEADDR)
#define SPI6			((SPI_RegDef_t*)SPI6_BASEADDR)

/*
 *
 * Macro functions
 *
 */


/*
 * Reset Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_RESET()	do{ ( RCC->AHB1RSTR |= (1 << 0) );	( RCC->AHB1RSTR &= ~(1 << 0) );  }while(0)
#define GPIOB_PCLK_RESET()	do{ ( RCC->AHB1RSTR |= (1 << 1) );  ( RCC->AHB1RSTR &= ~(1 << 1) );  }while(0)
#define GPIOC_PCLK_RESET()	do{ ( RCC->AHB1RSTR |= (1 << 2) );  ( RCC->AHB1RSTR &= ~(1 << 2) );  }while(0)
#define GPIOD_PCLK_RESET()	do{ ( RCC->AHB1RSTR |= (1 << 3) );  ( RCC->AHB1RSTR &= ~(1 << 3) );  }while(0)
#define GPIOE_PCLK_RESET()	do{ ( RCC->AHB1RSTR |= (1 << 4) );  ( RCC->AHB1RSTR &= ~(1 << 4) );  }while(0)
#define GPIOF_PCLK_RESET()	do{ ( RCC->AHB1RSTR |= (1 << 5) );  ( RCC->AHB1RSTR &= ~(1 << 5) );  }while(0)
#define GPIOG_PCLK_RESET()	do{ ( RCC->AHB1RSTR |= (1 << 6) );  ( RCC->AHB1RSTR &= ~(1 << 6) );  }while(0)
#define GPIOH_PCLK_RESET()	do{ ( RCC->AHB1RSTR |= (1 << 7) );  ( RCC->AHB1RSTR &= ~(1 << 7) );  }while(0)
#define GPIOI_PCLK_RESET()	do{ ( RCC->AHB1RSTR |= (1 << 8) );  ( RCC->AHB1RSTR &= ~(1 << 8) );  }while(0)
//#define GPIOJ_PCLK_RESET()	do{ ( RCC->AHB1RSTR |= (1 << 9) );  ( RCC->AHB1RSTR &= ~(1 << 9) );  }while(0)
//#define GPIOK_PCLK_RESET()	do{ ( RCC->AHB1RSTR |= (1 << 10));  ( RCC->AHB1RSTR &= ~(1 << 10));  }while(0)



/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()	( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()	( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()	( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()	( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()	( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()	( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()	( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()	( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()	( RCC->AHB1ENR |= (1 << 8) )
#define GPIOJ_PCLK_EN()	( RCC->AHB1ENR |= (1 << 9) )
#define GPIOK_PCLK_EN()	( RCC->AHB1ENR |= (1 << 10))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()	( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()	( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()	( RCC->APB1ENR |= (1 << 23) )

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()	( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()	( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()	( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN()	( RCC->APB2ENR |= (1 << 13) )
#define SPI5_PCLK_EN()	( RCC->APB2ENR |= (1 << 20) )
#define SPI6_PCLK_EN()	( RCC->APB2ENR |= (1 << 21) )

/*
 * Clock Enable Macros for UARTx peripherals
 */

#define USART1_PCLK_EN()	( RCC->APB2ENR |= (1 << 4)  )
#define USART2_PCLK_EN()	( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()	( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN()		( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()		( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN()	( RCC->APB2ENR |= (1 << 5)  )
#define UART7_PCLK_EN()		( RCC->APB1ENR |= (1 << 30) )
#define UART8_PCLK_EN()		( RCC->APB1ENR |= (1 << 31) )

/*
 * Clock Enable Macros for System Config peripherals
 */

#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= (1 << 14) )

#define GPIO_BASEADDR_TO_CODE(x)    ( (x == GPIOA)?0:\
							  (x == GPIOB)?1:\
							  (x == GPIOC)?2:\
							  (x == GPIOD)?3:\
		  	  	  	  	  	  (x == GPIOE)?4:\
		  	  	  	  	  	  (x == GPIOF)?5:\
		  	  	  	  	  	  (x == GPIOG)?6:\
							  (x == GPIOH)?7:\
		  	  	  	  	  	  (x == GPIOI)?8:\
							  (x == GPIOJ)?9:\
		  	  	  	  	  	  (x == GPIOK)?10:0 )

/*
 * CLOCK DISABLE MACROS
 */

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 8) )
#define GPIOJ_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 9) )
#define GPIOK_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 10))

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 23) )

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 13) )
#define SPI5_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 20) )
#define SPI6_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 21) )

/*
 * Clock Disable Macros for UARTx peripherals
 */

#define USART1_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 4)  )
#define USART2_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 18) )
#define UART4_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 5)  )
#define UART7_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 30) )
#define UART8_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 31) )

/*
 * Clock Disable Macros for System Config peripherals
 */

#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 14) )


/*
 *
 * Some Mapping
 *
 */

/*
 * EXTI NVIC_IRQ Mapping
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5	    23
#define IRQ_NO_EXTI15_10	40

/*
 *
 * Generic Macros
 *
 */

#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define BTN_PRESSED		SET

/*********************************END: Controller Specific Details*******************************/

#include "stm32f429xx_gpio_driver.h"

#endif /* INC_STM32F429XX_H_ */
