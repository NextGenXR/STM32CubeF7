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


class BSP_CLASS
{
public:
	virtual uint32_t GetVersion(void);
	virtual void LED_Init(Led_TypeDef Led);
	virtual void LED_DeInit(Led_TypeDef Led);
	virtual void LED_On(Led_TypeDef Led);
	virtual void LED_Off(Led_TypeDef Led);
	virtual void LED_Toggle(Led_TypeDef Led);
	virtual void PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
	virtual void PB_DeInit(Button_TypeDef Button);
	virtual uint32_t PB_GetState(Button_TypeDef Button);
	virtual void digitalSet(uint16_t pin);
	virtual void PinSet(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
	virtual void digitalReset(uint16_t pin);
	virtual void PinReset(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
	virtual GPIO_PinState digitalRead(uint16_t pin);
	virtual GPIO_PinState PinRead(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
	virtual void digitalWrite(uint16_t pin, GPIO_PinState PinState);
	virtual void PinWrite(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
	virtual void digitalToggle(uint16_t pin);
	virtual void TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

	virtual void SPIx_MspInit(SPI_HandleTypeDef *hspi);
	virtual void SPIx_Init(void);
	virtual void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth);
	virtual void SPIx_Write(uint8_t Value);
	virtual void SPIx_Error (void);

	virtual void SD_IO_Init(void);
	virtual void SD_IO_CSState(uint8_t val);
	virtual void SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);
	virtual uint8_t SD_IO_WriteByte(uint8_t Data);

#ifdef HAL_ADC_MODULE_ENABLED
	virtual static void ADCx_MspInit(ADC_HandleTypeDef *hadc);
	virtual static void ADCx_MspDeInit(ADC_HandleTypeDef *hadc);
	virtual static void ADCx_Init(void);
	virtual static void ADCx_DeInit(void);
	virtual uint8_t JOY_Init(void);
	virtual void JOY_DeInit(void);
	virtual JOYState_TypeDef BSP_JOY_GetState(void);


#endif

protected:

private:


};



#endif /* BSP_STM32F7XX_NUCLEO_144_BSP_CLASS_H_ */
