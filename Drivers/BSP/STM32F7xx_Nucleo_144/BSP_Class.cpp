/*
 * BSP_Class.cpp
 *
 *  Created on: Mar 5, 2022
 *      Author: jenni
 */

#include <stm32_includes.h>
#include "BSP_Class.h"
#include <cstdint>

#include HAL_SPI_H



/**
  * @brief STM32F7xx NUCLEO BSP Driver version number V1.0.0
  */
#ifndef __STM32F7xx_NUCLEO_BSP_VERSION
#define __STM32F7xx_NUCLEO_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __STM32F7xx_NUCLEO_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __STM32F7xx_NUCLEO_BSP_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __STM32F7xx_NUCLEO_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32F7xx_NUCLEO_BSP_VERSION        ((__STM32F7xx_NUCLEO_BSP_VERSION_MAIN << 24)\
                                             |(__STM32F7xx_NUCLEO_BSP_VERSION_SUB1 << 16)\
                                             |(__STM32F7xx_NUCLEO_BSP_VERSION_SUB2 << 8 )\
                                             |(__STM32F7xx_NUCLEO_BSP_VERSION_RC))
#endif
/**
  * @brief LINK SD Card
  */
#define SD_DUMMY_BYTE            0xFF
#define SD_NO_RESPONSE_EXPECTED  0x80


/* Defined in another module */
#ifndef __STM32F7XX_NUCLEO_H
uint32_t BSP_CLASS::GetVersion(void)
{
 /* return (__STM32F7xx_NUCLEO_VERSION);*/
	return (__STM32F7xx_NUCLEO_BSP_VERSION);
}
#else

uint32_t BSP_CLASS::GetVersion(void)
{
	 return (1);
}
#endif

void BSP_CLASS::LED_Init(Led_TypeDef Led)
{
	GetVersion();
}

void BSP_CLASS::LED_DeInit(Led_TypeDef Led)
{
	GetVersion();
}

void BSP_CLASS::LED_On(Led_TypeDef Led)
{
	GetVersion();
}

void BSP_CLASS::LED_Off(Led_TypeDef Led)
{
	GetVersion();
}

void BSP_CLASS::LED_Toggle(Led_TypeDef Led)
{
	GetVersion();
}

void BSP_CLASS::PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
	GetVersion();
}

void BSP_CLASS::PB_DeInit(Button_TypeDef Button)
{
	GetVersion();
}

uint32_t BSP_CLASS::PB_GetState(Button_TypeDef Button)
{
	return (0);
}

void BSP_CLASS::digitalSet(uint16_t pin)
{
	GetVersion();
}

void BSP_CLASS::PinSet(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GetVersion();
}

void BSP_CLASS::digitalReset(uint16_t pin)
{
	GetVersion();
}

void BSP_CLASS::PinReset(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GetVersion();
}

GPIO_PinState BSP_CLASS::digitalRead(uint16_t pin)
{
	return (GPIO_PinState::GPIO_PIN_RESET);
}

GPIO_PinState BSP_CLASS::PinRead(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	return (GPIO_PinState::GPIO_PIN_RESET);
}

void BSP_CLASS::digitalWrite(uint16_t pin, GPIO_PinState PinState)
{
	GetVersion();
}

void BSP_CLASS::PinWrite(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	GetVersion();
}

void BSP_CLASS::digitalToggle(uint16_t pin)
{
	GetVersion();
}

void BSP_CLASS::TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GetVersion();
}

void BSP_CLASS::SPIx_MspInit(SPI_HandleTypeDef *hspi)
{
	GetVersion();
}

void BSP_CLASS::SPIx_Init(void)
{
	GetVersion();
}

void BSP_CLASS::SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth)
{
	GetVersion();
}

void BSP_CLASS::SPIx_Write(uint8_t Value)
{
	GetVersion();
}

void BSP_CLASS::SPIx_Error(void)
{
	GetVersion();
}

void BSP_CLASS::SD_IO_Init(void)
{
	GetVersion();
}

void BSP_CLASS::SD_IO_CSState(uint8_t val)
{
	GetVersion();
}

void BSP_CLASS::SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
	GetVersion();
}

uint8_t BSP_CLASS::SD_IO_WriteByte(uint8_t Data)
{
	return (0);
}

#ifdef HAL_ADC_MODULE_ENABLED
void BSP_CLASS::ADCx_MspInit(ADC_HandleTypeDef *hadc)
{
	GetVersion();
}

void BSP_CLASS::ADCx_MspDeInit(ADC_HandleTypeDef *hadc)
{
	GetVersion();
}

void BSP_CLASS::ADCx_Init(void)
{
	GetVersion();
}

void BSP_CLASS::ADCx_DeInit(void)
{
	GetVersion();
}

uint8_t BSP_CLASS::JOY_Init(void)
{
	return (0);
}

void BSP_CLASS::JOY_DeInit(void)
{
	GetVersion();
}

JOYState_TypeDef BSP_CLASS::BSP_JOY_GetState(void)
{
	return (JOYState_TypeDef::JOY_NONE);
}
#endif
