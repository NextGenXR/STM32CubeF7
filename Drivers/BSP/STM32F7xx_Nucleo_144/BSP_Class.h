/*
 * BSP_Class.h
 *
 *  Created on: Mar 5, 2022
 *      Author: jenni
 */

#ifndef BSP_STM32F7XX_NUCLEO_144_BSP_CLASS_H_
#define BSP_STM32F7XX_NUCLEO_144_BSP_CLASS_H_

#include <cstdint>

#include "BSP_Types.h"

#ifdef __cplusplus

class BSP_CLASS
{
public:
	virtual ~BSP_Class();

	virtual uint32_t GetVersion(void) = 0;
	virtual void LED_Init(Led_TypeDef Led) = 0;
	virtual void LED_DeInit(Led_TypeDef Led) = 0;
	virtual void LED_On(Led_TypeDef Led) = 0;
	virtual void LED_Off(Led_TypeDef Led) = 0;
	virtual void LED_Toggle(Led_TypeDef Led) = 0;
	virtual void PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode) = 0;
	virtual void PB_DeInit(Button_TypeDef Button) = 0;
	virtual uint32_t PB_GetState(Button_TypeDef Button) = 0;
	virtual void digitalSet(uint16_t pin) = 0;
	virtual void PinSet(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) = 0;
	virtual void digitalReset(uint16_t pin) = 0;
	virtual void PinReset(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) = 0;
	virtual GPIO_PinState digitalRead(uint16_t pin) = 0;
	virtual GPIO_PinState PinRead(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) = 0;
	virtual void digitalWrite(uint16_t pin, GPIO_PinState PinState) = 0;
	virtual void PinWrite(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) = 0;
	virtual void digitalToggle(uint16_t pin) = 0;
	virtual void TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) = 0;

	virtual void SPIx_MspInit(SPI_HandleTypeDef *hspi) = 0;
	virtual void SPIx_Init(void) = 0;
	virtual void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth) = 0;
	virtual void SPIx_Write(uint8_t Value) = 0;
	virtual void SPIx_Error (void) = 0;

	virtual void SD_IO_Init(void) = 0;
	virtual void SD_IO_CSState(uint8_t val) = 0;
	virtual void SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength) = 0;
	virtual uint8_t SD_IO_WriteByte(uint8_t Data) = 0;

#ifdef HAL_ADC_MODULE_ENABLED
	virtual void ADCx_MspInit(ADC_HandleTypeDef *hadc) = 0;
	virtual void ADCx_MspDeInit(ADC_HandleTypeDef *hadc) = 0;
	virtual void ADCx_Init(void) = 0;
	virtual void ADCx_DeInit(void) = 0;
	virtual uint8_t JOY_Init(void) = 0;
	virtual void JOY_DeInit(void) = 0;
	virtual JOYState_TypeDef BSP_JOY_GetState(void) = 0;

#endif

protected:

private:


};


#endif /* cpp */

#endif /* BSP_STM32F7XX_NUCLEO_144_BSP_CLASS_H_ */
