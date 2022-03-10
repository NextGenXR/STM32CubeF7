/*
 * Constants.h
 *
 *  Created on: Jan 4, 2022
 *  Author: NextGen XR, Inc.
 */

#ifndef _STM32F7xx_CONSTANTS_H_
#define _STM32F7xx_CONSTANTS_H_

#include "main.h"
#include "stm32_includes.h"
#include "CM3KConstants.h"
#include VARIANT_H
#include <stm32f7xx_nucleo_144.hpp>


#ifdef __cplusplus
extern "C"
{
#endif

typedef struct GPIO_Pin
{
	uint8_t	PortPin;
	uint16_t Port;
	uint8_t Pin;
}GPIO_Pin;

typedef GPIO_Pin GPIO_Pin;

/* GPIO_Pin arduino_pins[16]; */

//#define D0			PG9
#define D0_PIN		GPIO_PIN_9
#define D0_PORT		GPIOG

//#define D1			PG14
#define D1_PIN		GPIO_PIN_14
#define D1_PORT		GPIOG

//#define D2			PF15
#define D2_PIN		GPIO_PIN_15
#define D2_PORT		GPIOF

//#define D3			PE13
#define D3_PIN		GPIO_PIN_13
#define D3_PORT		GPIOE

//#define D4			PF14
#define D4_PIN		GPIO_PIN_14
#define D4_PORT		GPIOF

//#define D5			PE11
#define D5_PIN		GPIO_PIN_11
#define D5_PORT		GPIOE

//#define D6			PE9
#define D6_PIN		GPIO_PIN_9
#define D6_PORT		GPIOE

//#define D7			PF13
#define D7_PIN		GPIO_PIN_13
#define D7_PORT		GPIOF

//#define D8			PF12
#define D8_PIN		GPIO_PIN_12
#define D8_PORT		GPIOF

//#define D9			PD15
#define D9_PIN		GPIO_PIN_15
#define D9_PORT		GPIOD

//#define D10			PD14
#define D10_PIN		GPIO_PIN_14
#define D10_PORT	GPIOD

//#define D11 		PB5
#define D11_PIN		GPIO_PIN_5
#define D11_PORT	GPIOB

//#define D12 		PA6
#define D12_PIN		GPIO_PIN_6
#define D12_PORT	GPIOA

//#define D13 		PA5
#define D13_PIN		GPIO_PIN_5
#define D13_PORT	GPIOA

//#define D14			PB9
#define D14_PIN		GPIO_PIN_9
#define D14_PORT	GPIOB

//#define D15			PB8
#define D15_PIN		GPIO_PIN_8
#define D15_PORT	GPIOB


#define I2C_SCL_PIN		D15_PIN
#define I2C_SCL_PORT	D15_PORT
#define I2C_SDA_PIN		D14_PIN
#define I2C_SDA_PORT	D14_PORT

#define SPI_SCK_PIN 	D13_PIN
#define SPI_SCK_PORT 	D13_PORT

#define SPI_MOSI_PIN	D12_PIN
#define SPI_MOSI_PORT	D12_PORT

#define SPI_MISO_PIN	D11_PIN
#define SPI_MISO_PORT	D11_PORT

#define SPI_CS_PIN		D10_PIN
#define SPI_CS_PORT		D10_PORT

#define SPI_CS1_PIN		D9_PIN
#define SPI_CS1_PORT	D9_PORT

#define SPI3_SCK_PIN 	GPIO_PIN_10
#define SPI3_SCK_PORT 	GPIOC

#define SPI3_MOSI_PIN	GPIO_PIN_2
#define SPI3_MOSI_PORT	GPIOB

#define SPI3_MISO_PIN	GPIO_PIN_11
#define SPI3_MISO_PORT	GPIOC
//#define SPI3_CS_PIN		GPIO_PIN_
//#define SPI3_CS_PORT	GPIOC

#define SPI5_SCK_PIN 	GPIO_PIN_7
#define SPI5_SCK_PORT 	GPIOF

#define SPI5_MOSI_PIN	GPIO_PIN_9
#define SPI5_MOSI_PORT	GPIOF

#define SPI5_MISO_PIN	GPIO_PIN_8
#define SPI5_MISO_PORT	GPIOF
//#define SPI3_CS_PIN		GPIO_PIN_
//#define SPI3_CS_PORT	GPIOC



/** @addtogroup X_NUCLEO_PLC01A1_IO_Public_Constan

 */

#define X_NUCLEO_SPI_EXPBD_CLK_ENABLE() NUCLEO_SPIx_CLK_ENABLE()
// __SPI1_CLK_ENABLE()


#ifdef STM32F030x8
#define X_NUCLEO_SPI_MISO_ALTERNATE     GPIO_AF0_SPI1
#elif (STM32F334x8|STM32F401xE)
#define X_NUCLEO_SPI_MISO_ALTERNATE     GPIO_AF5_SPI1
#endif

// Validated for Nucleo-144
#define X_NUCLEO_SPI_EXPBD_MISO_PIN     NUCLEO_SPIx_MISO_PIN
#define X_NUCLEO_SPI_EXPBD_MISO_PORT    NUCLEO_SPIx_MISO_MOSI_GPIO_PORT

#define X_NUCLEO_SPI_EXPBD_MOSI_PIN     NUCLEO_SPIx_MOSI_PIN
#define X_NUCLEO_SPI_EXPBD_MOSI_PORT    NUCLEO_SPIx_MISO_MOSI_GPIO_PORT

#ifdef STM32F030x8
#define X_NUCLEO_SPI_MOSI_ALTERNATE     GPIO_AF0_SPI1
#elif (STM32F334x8|STM32F401xE)
#define X_NUCLEO_SPI_MOSI_ALTERNATE     GPIO_AF5_SPI1
#endif



#if defined (STM32F030x8) | defined (STM32F334x8) | defined (STM32F401xE)
#define X_NUCLEO_SPI_EXPBD_SCK_PIN      GPIO_PIN_3
#elif   STM32F103xB
#define X_NUCLEO_SPI_EXPBD_SCK_PIN      GPIO_PIN_5
#endif

#ifdef STM32F030x8
#define X_NUCLEO_SPI_SCK_ALTERNATE     GPIO_AF0_SPI1
#elif (STM32F334x8|STM32F401xE)
#define X_NUCLEO_SPI_SCK_ALTERNATE     GPIO_AF5_SPI1
#endif

#if defined (STM32F030x8) | defined (STM32F334x8) | defined (STM32F401xE)
#define X_NUCLEO_SPI_EXPBD_SCK_PORT    GPIOB
#elif   STM32F103xB
#define X_NUCLEO_SPI_EXPBD_SCK_PORT    GPIOA
#endif

// TODO: Should this calculated based on timebase freq?
#define X_NUCLEO_TIM_PRESCALER          749

// TODO: Verify PLC IO Connections
// TODO: Verify other uControllers for this configuration
#if defined (STM32F767xx) // Nucleo-144 Board

/*

#define SPI_A_SCK_Pin GPIO_PIN_5
#define SPI_A_SCK_GPIO_Port GPIOA

#define SPI_A_MISO_Pin GPIO_PIN_6
#define SPI_A_MISO_GPIO_Port GPIOA

#define SPI_A_MOSI_Pin GPIO_PIN_5
#define SPI_A_MOSI_GPIO_Port GPIOB

#define PLC_OUT_EN_Pin GPIO_PIN_9
#define PLC_OUT_EN_GPIO_Port GPIOE

#define PLC_SPI_SCK2_Pin GPIO_PIN_13
#define PLC_SPI_SCK2_GPIO_Port GPIOE

#define PLC_INPUT_CS2_Pin GPIO_PIN_14
#define PLC_INPUT_CS2_GPIO_Port GPIOD

#define PLC_RELAY_CS1_Pin GPIO_PIN_15
#define PLC_RELAY_CS1_GPIO_Port GPIOD

 */

// Confirmed Pins: D13 PA5 for SPI Clock.
#define X_NUCLEO_SPI_EXPBD_SCK_PIN     	SPI_A_SCK_Pin
#define NUCLEO_SPIx_SCK_PIN				SPI_A_SCK_Pin
#define X_NUCLEO_SPI_EXPBD_SCK_PORT    	NUCLEO_SPIx_SCK_GPIO_PORT

/* INPUT == CS1, RELAYS == CS2 */

#define SPI1_INPUT_CS1_LOW()       HAL_GPIO_WritePin(PLC_INPUT_CS1_GPIO_Port, PLC_INPUT_CS1_Pin, GPIO_PIN_RESET)
#define SPI1_INPUT_CS1_HIGH()      HAL_GPIO_WritePin(PLC_INPUT_CS1_GPIO_Port, PLC_INPUT_CS1_Pin, GPIO_PIN_SET)
#define SPI1_RELAY_CS2_LOW()       HAL_GPIO_WritePin(PLC_RELAY_CS2_GPIO_Port, PLC_RELAY_CS2_Pin, GPIO_PIN_RESET)
#define SPI1_RELAY_CS2_HIGH()      HAL_GPIO_WritePin(PLC_RELAY_CS2_GPIO_Port, PLC_RELAY_CS2_Pin, GPIO_PIN_SET)

// VNI8200XP_CS = D9 = PD15
// VNI8200XP_OUT_EN = PE9 = D6
// VNI8200XP_RESET = VNI8200XP_OUT_EN
// VNI8200XP_ENABLE = VNI8200XP_OUT_EN

// CLT01_38S_CS = D10 = PD14

// SPI1
// SPI_A_SCK = D13 = PA5
// SPI_A_MISO = D12 = PA6
// SPI_A_MOSI = D11 = PB5
//const uint16_t CLT_CS = 10; //= PD14 = CS2 = CTL
//const uint16_t VNI_CS = 9; //D9 = PD15

#define SPI_INTERFACE 1
//#define SPI_INTERFACE 3
//#define SPI_INTERFACE 5

#define RELAY_CS 10
#define INPUT_CS 9
#define RELAY_OUT_EN 6
#define RELAY_RESET 6

#if (SPI_INTERFACE == 1)
#define SPI_DRIVER hspi1
#define SPI_SCK SPI_A_SCK
#define SPI_MOSI SPI_A_MISO
#define SPI_MISO SPI_A_MOSI
#define SPI_CS1 PD15	// RELAYS D10
#define SPI_CS2 PD14	// INPUTS D9
#endif

#if (SPI_INTERFACE == 3)
#define SPI_HANDLE hspi3
#define SPI_SCK SPI3_SCK
#define SPI_MOSI SPI3_MISO
#define SPI_MISO SPI3_MOSI
#define SPI_CS1 PD14
#define SPI_CS2 PD15
#endif


#if (SPI_INTERFACE == 5)
#define SPI_HANDLE hspi5
#define SPI_SCK SPI5_SCK
#define SPI_MOSI SPI5_MISO
#define SPI_MISO SPI5_MOSI
#define SPI_CS1 PD14
#define SPI_CS2 PD15
#endif

/**
 * \brief From Internet for f767
 */

/* CS2 == Relays == D10 == PD14
 * CS1 == Inputs == D09 == PD15 */

#define VNI_CS_CLK_ENABLE()       __GPIOD_CLK_ENABLE();
#define VNI_CS_PIN                D10_PIN
#define VNI_CS_PORT               D10_PORT

#define VNI_OUT_EN_CLK_ENABLE()   __GPIOE_CLK_ENABLE();
#define VNI_OUT_EN_PIN            D6_PIN
#define VNI_OUT_EN_PORT           D6_PORT

#define VNI_RESET_PORT            D6_PORT
#define VNI_RESET_PIN             D6_PIN
#define VNI_ENABLE_PORT           VNI_OUT_EN_PORT
#define VNI_ENABLE_PIN            VNI_OUT_EN_PIN

#define CLT_CS_CLK_ENABLE()       __GPIOD_CLK_ENABLE();
#define CLT_CS_PIN                D9_PIN
#define CLT_CS_PORT               D9_PORT

#define VNI_OUT_EN_LOW()       		SPI1_RELAY_CS2_LOW()
#define VNI_OUT_EN_HIGH()      		SPI1_RELAY_CS2_HIGH()
#define VNI_ENABLE_LOW()       		HAL_GPIO_WritePin(VNI_ENABLE_PORT, VNI_ENABLE_PIN, GPIO_PIN_RESET)
#define VNI_ENABLE_HIGH()      		HAL_GPIO_WritePin(VNI_ENABLE_PORT, VNI_ENABLE_PIN, GPIO_PIN_SET)
#define VNI_RESET_HIGH()			HAL_GPIO_WritePin(VNI_RESET_PORT, VNI_RESET_PIN, GPIO_PIN_RESET)
#define VNI_RESET_LOW() 			HAL_GPIO_WritePin(VNI_RESET_PORT, VNI_RESET_PIN, GPIO_PIN_RESET)

#define VNI_CS_LOW()				SPI1_RELAY_CS2_LOW()
#define VNI_CS_HIGH()				SPI1_RELAY_CS2_HIGH()

#define CLT01_CS_LOW()				SPI1_INPUT_CS1_LOW()
#define CLT01_CS_HIGH()				SPI1_INPUT_CS1_HIGH()

#ifdef SKIP_THIS
// Relay Output Chip Defines
#define VNI8200XP_CS_CLK_ENABLE()       __GPIOD_CLK_ENABLE();
#define VNI8200XP_CS_PIN                PLC_RELAY_CS1_Pin
#define VNI8200XP_CS_PORT               PLC_RELAY_CS1_GPIO_Port

#define VNI8200XP_OUT_EN_CLK_ENABLE()   __GPIOE_CLK_ENABLE();
#define VNI8200XP_OUT_EN_PIN            PLC_OUT_EN_Pin
#define VNI8200XP_OUT_EN_PORT           PLC_OUT_EN_GPIO_Port


// TODO: Check Reset. Doesn't appear in schematics. Same as CS1 for Relay. Hold low to reset?
#define VNI8200XP_RESET_PORT            PLC_OUT_EN_GPIO_Port
#define VNI8200XP_RESET_PIN             PLC_OUT_EN_Pin
#define VNI8200XP_RESET_LOW()       	VNI8200XP_OUT_EN_LOW()
#define VNI8200XP_RESET_HIGH()      	VNI8200XP_OUT_EN_HIGH()

// PLC_OUT_EN
#define VNI8200XP_ENABLE_PORT           PLC_OUT_EN_GPIO_Port
#define VNI8200XP_ENABLE_PIN

// Input Chip Defines
//#define CLT01_38S_CS 					PD14
#define CLT01_38S_CS_CLK_ENABLE()       __GPIOD_CLK_ENABLE();
#define CLT01_38S_CS_PIN                PLC_INPUT_CS2_Pin
#define CLT01_38S_CS_PORT               PLC_INPUT_CS2_GPIO_Port          PLC_OUT_EN_Pin

#endif

#define FB_OK_STATUS 0x80
#define TEMP_WARNING_STATUS 0x40
#define PC_FAIL_STATUS 0x20
#define POWER_GOOD_STATUS 0x10


#endif

// Defaults for non STM32F767 boards
#ifndef STM32F767xx
#define VNI8200XP_CS_CLK_ENABLE()       __GPIOC_CLK_ENABLE();
#define VNI8200XP_CS_PIN                GPIO_PIN_7
#define VNI8200XP_CS_PORT               GPIOC

#define VNI8200XP_OUT_EN_CLK_ENABLE()   __GPIOB_CLK_ENABLE();
#define VNI8200XP_OUT_EN_PIN            GPIO_PIN_10
#define VNI8200XP_OUT_EN_PORT           GPIOB

#define CLT01_38S_CS_CLK_ENABLE()       __GPIOB_CLK_ENABLE();
#define CLT01_38S_CS_PIN                GPIO_PIN_6
#define CLT01_38S_CS_PORT               GPIOB

#define VNI8200XP_RESET_PORT            GPIOC
#define VNI8200XP_RESET_PIN             GPIO_PIN_7

#define VNI8200XP_ENABLE_PORT           GPIOB
#define VNI8200XP_ENABLE_PIN            GPIO_PIN_10
#endif


#ifdef __cplusplus
}
#endif

#endif
