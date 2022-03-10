/*
 * BSP_Types.h
 *
 *  Created on: Mar 5, 2022
 *      Author: jenni
 */

#ifndef BSP_STM32F7XX_NUCLEO_144_BSP_TYPES_H_
#define BSP_STM32F7XX_NUCLEO_144_BSP_TYPES_H_


/** @defgroup STM32F7XX_NUCLEO_144_LOW_LEVEL_Exported_Types
  * @{
  */
typedef enum
{
  LED1 = 0,
  LED_GREEN = LED1,
  LED2 = 1,
  LED_BLUE = LED2,
  LED3 = 2,
  LED_RED = LED3
}Led_TypeDef;

typedef enum
{
  BUTTON_USER = 0,
  /* Alias */
  BUTTON_KEY = BUTTON_USER
}Button_TypeDef;

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;

typedef enum
{
  JOY_NONE  = 0,
  JOY_SEL   = 1,
  JOY_DOWN  = 2,
  JOY_LEFT  = 3,
  JOY_RIGHT = 4,
  JOY_UP    = 5
}JOYState_TypeDef;





#endif /* BSP_STM32F7XX_NUCLEO_144_BSP_TYPES_H_ */
