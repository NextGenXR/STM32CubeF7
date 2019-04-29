/**
  ******************************************************************************
  * This file is part of the TouchGFX 4.10.0 distribution.
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
  


#include "STM32756GTouchController.hpp"

extern "C" {
#include "stm32756g_eval_ts.h"

uint32_t LCD_GetXSize();
uint32_t LCD_GetYSize();
}

using namespace touchgfx;

void STM32756GTouchController::init()
{
    BSP_TS_Init(LCD_GetXSize(), LCD_GetYSize());
}

static uint32_t prev_timestamp;
static int32_t prev_x, prev_y;

bool STM32756GTouchController::sampleTouch(int32_t& x, int32_t& y)
{
    TS_StateTypeDef state;

    /* sample the controller state */
    BSP_TS_GetState(&state);

    if (state.TouchDetected)
    {
        x = state.x;
        y = state.y;

        prev_x = x;
        prev_y = y;
        prev_timestamp = HAL_GetTick();

        return true;
    }

    uint32_t time = HAL_GetTick();

    /* EXC7200: filter out transient events within a 80ms windows */
    if ((prev_timestamp + 80) > time) {
        x = prev_x;
        y = prev_y;
        return true;
    }

    return false;
}