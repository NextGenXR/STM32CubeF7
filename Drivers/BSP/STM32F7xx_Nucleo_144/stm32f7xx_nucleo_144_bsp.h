/*
 * stm32f7xx_nucleo_144_bsp.h
 *
 *  Created on: Mar 2, 2022
 *      Author: jenni
 */

#ifndef BSP_STM32F7XX_NUCLEO_144_STM32F7XX_NUCLEO_144_BSP_H_
#define BSP_STM32F7XX_NUCLEO_144_STM32F7XX_NUCLEO_144_BSP_H_

class stm32f7xx_nucleo_144_bsp {
public:
	stm32f7xx_nucleo_144_bsp();
	virtual ~stm32f7xx_nucleo_144_bsp();

	/** @defgroup STM32F7XX_NUCLEO_144_LOW_LEVEL_Exported_Functions
	  * @{
	  */

	#ifndef __STM32F7XX_NUCLEO_H
		uint32_t	GetVersion(void);
	#endif

		void        LED_Init(Led_TypeDef Led);
		void        LED_DeInit(Led_TypeDef Led);
		void        LED_On(Led_TypeDef Led);
		void        LED_Off(Led_TypeDef Led);
		void       	LED_Toggle(Led_TypeDef Led);
		void        PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
		void        PB_DeInit(Button_TypeDef Button);
		uint32_t    PB_GetState(Button_TypeDef Button);

		void 		Pin_Init(uint8_t pin, bool type);

		void 		digitalSet(uint16_t GPIO_Pin);
		void 		digitalReset(uint16_t GPIO_Pin);
		GPIO_PinState digitalRead(uint16_t GPIO_Pin);
		void 		digitalWrite(uint16_t GPIO_Pin, GPIO_PinState PinState);
		void 		digitalToggle(uint16_t GPIO_Pin);

		void 		PinSet(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
		void 		PinReset(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
		GPIO_PinState PinRead(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
		void 		PinWrite(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
		void 		TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);


	#ifdef HAL_SPI_MODULE_ENABLED

	#define SPI_PORT hspi1	/* Arduino SPI Port */

		/* extern SPI_HandleTypeDef hspi1;*/
		#define hnucleo_Spi SPI_PORT
		#define SpixTimeout 0x100

		void SP_SPIx_Init();
		void SD_IO_Init(void);
		void SD_IO_CSState(uint8_t val);
		void SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);
		uint8_t SD_IO_WriteByte(uint8_t Data);

		void SPIx_Error (void);
		void SPIx_Write(uint8_t Value);
		void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth);
		void SPIx_Init(void);
		void SPIx_MspInit(SPI_HandleTypeDef *SPI_PORT);
	#endif

	#ifdef JOY_ENABLED
		uint8_t          JOY_Init(void);
		JOYState_TypeDef JOY_GetState(void);
		void             JOY_DeInit(void);
	#endif /* HAL_ADC_MODULE_ENABLED */



};

#endif /* BSP_STM32F7XX_NUCLEO_144_STM32F7XX_NUCLEO_144_BSP_H_ */
