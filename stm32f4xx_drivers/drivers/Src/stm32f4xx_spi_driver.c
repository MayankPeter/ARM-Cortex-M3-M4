/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Sep 12, 2021
 *      Author: root
 */

#include "stm32f429xx_spi_driver.h"

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if	   (pSPIx == SPI1){ SPI1_PCLK_EN(); }
			else if(pSPIx == SPI2){ SPI2_PCLK_EN(); }
			else if(pSPIx == SPI3){ SPI3_PCLK_EN(); }
			else if(pSPIx == SPI4){ SPI4_PCLK_EN(); }
			else if(pSPIx == SPI5){ SPI5_PCLK_EN(); }
			else if(pSPIx == SPI6){ SPI6_PCLK_EN(); }

		}
		else if(EnorDi == DISABLE)
		{
			if	   (pSPIx == SPI1){ SPI1_PCLK_DI(); }
			else if(pSPIx == SPI2){ SPI2_PCLK_DI(); }
			else if(pSPIx == SPI3){ SPI3_PCLK_DI(); }
			else if(pSPIx == SPI4){ SPI4_PCLK_DI(); }
			else if(pSPIx == SPI5){ SPI5_PCLK_DI(); }
			else if(pSPIx == SPI6){ SPI6_PCLK_DI(); }

		}
}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{

}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{

}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}
