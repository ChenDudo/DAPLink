/**
 * @file    flash.c
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2019, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define _BEEP_C_

#include "mm32_device.h"
#include "hal_gpio.h"
#include "hal_tim.h"
#include "hal_rcc.h"

#include "beep.h"

// TIM4 CH3
#define BEEP_TIMER      TIM4
#define ARR_VALUE       999


/******************************************************************************/
void InitBeep(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef       TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM4, ENABLE);
	TIM_TimeBaseStructure.TIM_Period        = ARR_VALUE;
	TIM_TimeBaseStructure.TIM_Prescaler     = 95;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(BEEP_TIMER, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode          = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState     = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse           = ARR_VALUE >> 1;
	TIM_OCInitStructure.TIM_OCPolarity      = TIM_OCPolarity_High;
	TIM_OC3Init(BEEP_TIMER, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(BEEP_TIMER, TIM_OCPreload_Enable);

	TIM_CtrlPWMOutputs(BEEP_TIMER, ENABLE);
	TIM_ARRPreloadConfig(BEEP_TIMER, ENABLE);
	TIM_Cmd(BEEP_TIMER, ENABLE);
	//BEEP_off();
}

/******************************************************************************/
void BEEP_on(void)
{
	TIM_Cmd(BEEP_TIMER, ENABLE);
}

/******************************************************************************/
void BEEP_off(void)
{
	TIM_Cmd(BEEP_TIMER, DISABLE);
}
