/*
 * stm32f7xxnucleo144.h
 *
 *  Created on: Jan 8, 2022
 *      Author: jenni
 */

#ifndef BSP_STM32F7XX_NUCLEO_144_STM32F7XX_NUCLEO_144_HPP_
#define BSP_STM32F7XX_NUCLEO_144_STM32F7XX_NUCLEO_144_HPP_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stm32_includes.h>
#include HAL_SPI_H
#include "spi.h"
#include CONSTANTS_H
#include PROCESSOR_H

#include "BSP_Types.h"

/* #include <cstdbool>*/

/* To be defined only if the board is provided with the related shield */
/* https://www.adafruit.com/products/802 */
#ifndef ADAFRUIT_TFT_JOY_SD_ID802
//#define ADAFRUIT_TFT_JOY_SD_ID802
#endif

#ifdef HAL_ADC_MODULE_ENABLED
#undef HAL_ADC_MODULE_ENABLED
#endif

/** @addtogroup BSP
  * @{
  */


/** @addtogroup STM32F7XX_NUCLEO_144
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F7XX_NUCLEO_144_LOW_LEVEL_Exported_Constants
  * @{
  */

/**
  * @brief Define for STM32F7XX_NUCLEO_144 board
  */
#if !defined (USE_STM32F7XX_NUCLEO_144)
 #define USE_STM32F7XX_NUCLEO_144
#endif

/** @addtogroup STM32F7XX_NUCLEO_144_LOW_LEVEL_LED
  * @{
  */
#define LEDn                                    3

#define LED1_PIN                                GPIO_PIN_0
#define LED1_GPIO_PORT                          GPIOB
#define LED1_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()

#define LED2_PIN                                GPIO_PIN_7
#define LED2_GPIO_PORT                          GPIOB
#define LED2_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()

#define LED3_PIN                                GPIO_PIN_14
#define LED3_GPIO_PORT                          GPIOB
#define LED3_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   do { if((__INDEX__) == 0) {__HAL_RCC_GPIOB_CLK_ENABLE();} else\
                                                                    {__HAL_RCC_GPIOB_CLK_ENABLE();   }} while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  do { if((__INDEX__) == 0) {__HAL_RCC_GPIOB_CLK_DISABLE();} else\
                                                                    {__HAL_RCC_GPIOB_CLK_DISABLE();   }} while(0)
/**
  * @}
  */

/** @addtogroup STM32F7XX_NUCLEO_144_LOW_LEVEL_BUTTON
  * @{
  */
#define BUTTONn 1

/**
 * @brief Key push-button
 */
#define USER_BUTTON_PIN                          GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT                    GPIOC
#define USER_BUTTON_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define USER_BUTTON_EXTI_LINE                    GPIO_PIN_13
#define USER_BUTTON_EXTI_IRQn                    EXTI15_10_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)      USER_BUTTON_GPIO_CLK_ENABLE()
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)     USER_BUTTON_GPIO_CLK_DISABLE()

/* Aliases */
#define KEY_BUTTON_PIN                       USER_BUTTON_PIN
#define KEY_BUTTON_GPIO_PORT                 USER_BUTTON_GPIO_PORT
#define KEY_BUTTON_GPIO_CLK_ENABLE()         USER_BUTTON_GPIO_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE()        USER_BUTTON_GPIO_CLK_DISABLE()
#define KEY_BUTTON_EXTI_LINE                 USER_BUTTON_EXTI_LINE
#define KEY_BUTTON_EXTI_IRQn                 USER_BUTTON_EXTI_IRQn


/**
  * @brief USB Pins definition
  */


#define OTG_FS1_OVER_CURRENT_PIN                  GPIO_PIN_7
#define OTG_FS1_OVER_CURRENT_PORT                 GPIOG
#define OTG_FS1_OVER_CURRENT_PORT_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()

#define OTG_FS1_POWER_SWITCH_PIN                  GPIO_PIN_6
#define OTG_FS1_POWER_SWITCH_PORT                 GPIOG
#define OTG_FS1_POWER_SWITCH_PORT_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()


/**
  * @}
  */

/** @addtogroup STM32F7XX_NUCLEO_144_LOW_LEVEL_BUS
  * @{
  */
/*############################### SPI_A #######################################*/
#ifdef HAL_SPI_MODULE_ENABLED

#define NUCLEO_SPIx                                     SPI1
#define NUCLEO_SPIx_CLK_ENABLE()                        __HAL_RCC_SPI1_CLK_ENABLE()

#define NUCLEO_SPIx_SCK_AF                              GPIO_AF5_SPI1
#define NUCLEO_SPIx_SCK_GPIO_PORT                       SPI_A_SCK_GPIO_Port
#define NUCLEO_SPIx_SCK_PIN                             SPI_A_SCK_Pin
#define NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
#define NUCLEO_SPIx_SCK_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOA_CLK_DISABLE()

#define NUCLEO_SPIx_MISO_MOSI_AF                        GPIO_AF5_SPI1
#define NUCLEO_SPIx_MISO_MOSI_GPIO_PORT                 SPI_A_MOSI_GPIO_Port
#define NUCLEO_SPIx_MISO_MISO_GPIO_PORT					SPI_A_MISO_GPIO_Port
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()
#define NUCLEO_SPIx_MOSI_MOSI_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define NUCLEO_SPIx_MOSI_MOSI_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOB_CLK_DISABLE()
#define NUCLEO_SPIx_MISO_PIN                            SPI_A_MISO_Pin
#define NUCLEO_SPIx_MOSI_PIN                            SPI_A_MOSI_Pin
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define NUCLEO_SPIx_TIMEOUT_MAX                   1000

#define NUCLEO_SPIx_CS_GPIO_PORT                        GPIOD
#define NUCLEO_SPIx_CS_PIN                              GPIO_PIN_14
#define NUCLEO_SPIx_CS_GPIO_CLK_ENABLE()                __HAL_RCC_GPIOD_CLK_ENABLE()
#define NUCLEO_SPIx_CS_GPIO_CLK_DISABLE()               __HAL_RCC_GPIOD_CLK_DISABLE()

#define SPIx__CS_LOW()       HAL_GPIO_WritePin(NUCLEO_SPIx_CS_GPIO_PORT, NUCLEO_SPIx_CS_PIN, GPIO_PIN_RESET)
#define SPIx__CS_HIGH()      HAL_GPIO_WritePin(NUCLEO_SPIx_CS_GPIO_PORT, NUCLEO_SPIx_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  SD Control Lines management
  */
#define SD_CS_LOW()       HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()      HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  LCD Control Lines management
  */
#define LCD_CS_LOW()      HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH()     HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)
#define LCD_DC_LOW()      HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_HIGH()     HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET)

/**
  * @brief  SD Control Interface pins (shield D4)
  */
#define SD_CS_PIN                                 GPIO_PIN_14
#define SD_CS_GPIO_PORT                           GPIOF
#define SD_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOF_CLK_ENABLE()
#define SD_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOF_CLK_DISABLE()

/**
  * @brief  LCD Control Interface pins (shield D10)
  */
#define LCD_CS_PIN                                 GPIO_PIN_14
#define LCD_CS_GPIO_PORT                           GPIOD
#define LCD_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOD_CLK_ENABLE()
#define LCD_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOD_CLK_DISABLE()

/**
  * @brief  LCD Data/Command Interface pins (shield D8)
  */
#define LCD_DC_PIN                                 GPIO_PIN_12
#define LCD_DC_GPIO_PORT                           GPIOF
#define LCD_DC_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOF_CLK_ENABLE()
#define LCD_DC_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOF_CLK_DISABLE()

#endif /* HAL_SPI_MODULE_ENABLED */

/*################################ ADC3 ######################################*/
/**
  * @brief  ADC Interface pins
  *         used to detect motion of Joystick available on Adafruit 1.8" TFT shield
  */

#ifdef HAL_ADC_MODULE_ENABLED

#define NUCLEO_ADCx                        ADC3
#define NUCLEO_ADCx_CLK_ENABLE()         __HAL_RCC_ADC3_CLK_ENABLE()
#define NUCLEO_ADCx_CLK_DISABLE()        __HAL_RCC_ADC3_CLK_DISABLE()

#define NUCLEO_ADCx_CHANNEL                ADC_CHANNEL_9

#define NUCLEO_ADCx_GPIO_PORT              GPIOF
#define NUCLEO_ADCx_GPIO_PIN               GPIO_PIN_3
#define NUCLEO_ADCx_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()
#define NUCLEO_ADCx_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOF_CLK_DISABLE()

#endif /* HAL_ADC_MODULE_ENABLED */


#endif /* BSP_STM32F7XX_NUCLEO_144_STM32F7XX_NUCLEO_144_HPP_ */
