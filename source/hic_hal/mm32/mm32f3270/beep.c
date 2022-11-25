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

#include <string.h>
#include "mm32_device.h"
#include "hal_gpio.h"
#include "hal_tim.h"
#include "hal_rcc.h"
#include "hal_gpio.h"
#include "hal_adc.h"

#include "settings.h"
#include "IO_Config.h"
#include "beep.h"

/******************************************************************************/
void InitGPIOBEEP(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
    // configure Beep
    GPIO_PinAFConfig(BEEP_PORT, BEEP_PIN_Bit, GPIO_AF_2);
    GPIO_ResetBits(BEEP_PORT, BEEP_PIN);
    GPIO_InitStructure.GPIO_Pin = BEEP_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(BEEP_PORT, &GPIO_InitStructure);
}
	
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
	
	InitGPIOBEEP();
	
	BEEP_off();
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

/******************************************************************************/
void BEEP_Hz(int pulse)
{
	TIM_SetCompare3(BEEP_TIMER, pulse);
}

//GLOBAL bool			beepEn;
//GLOBAL uint8_t		beepCount;
//GLOBAL embeepMode	beepMode;

/******************************************************************************/
void Beep_Tick(void)
{
    static uint16_t beepicnt = 0;
    
    if (beepicnt++){
        beepicnt = 0;
        beepEn = config_get_beep_en() ? true : false;
        
	if (beepEn){
		if (beepCount){
			((uint8_t)beepMode >> (5-beepCount)) & 0x01 ? BEEP_on() : BEEP_off();
            beepCount--;
		}
		else
			BEEP_off();
    }
	else
		BEEP_off();
}
}

/******************************************************************************/
void setBeepMode(embeepMode mode)
{
    beepMode = mode; beepCount = 5;
}

/******************************************************************************/
void initADC(void)
{
#if defined(DET_TVDD_PORT)
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// enable clock to ports
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= DET_TVDD_PIN;
    GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AIN;
    GPIO_Init(DET_TVDD_PORT , &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= DET_TVCC_PIN;
    GPIO_Init(DET_TVCC_PORT , &GPIO_InitStructure);
	
	ADC_InitTypeDef  ADC_InitStruct;
    ADC_StructInit(&ADC_InitStruct);

    ADC_InitStruct.ADC_Resolution	= ADC_Resolution_12b;
    ADC_InitStruct.ADC_PRESCARE		= ADC_PCLK2_PRESCARE_16;                        //ADC prescale factor
    ADC_InitStruct.ADC_Mode			= ADC_Mode_Continue;                                //ADC continue scan convert mode
    ADC_InitStruct.ADC_DataAlign	= ADC_DataAlign_Right;                         //AD data right-justified
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_Init(ADC1, &ADC_InitStruct);
	
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2,  1, ADC_Samctl_240_5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3,  2, ADC_Samctl_240_5);
    
    ADC_Cmd(ADC1, ENABLE);
#endif
}

/******************************************************************************/
void adcTick()
{
#if defined(DET_TVDD_PORT)
	uint16_t adctempValue[2];

	if (ADC_GetFlagStatus(ADC1, ADC_IT_EOC)) {
		adctempValue[0] = ADC1->CH2DR & 0xFFF;
		adctempValue[1] = ADC1->CH3DR & 0xFFF;
		adcValue[0] = (u16)((float)(adcValue[0] * 5 + adctempValue[0] * 5) / 10);
		adcValue[1] = (u16)((float)(adcValue[1] * 5 + adctempValue[1] * 5) / 10);
        
        targetVDD = 3300 * adcValue[0] / 0x0FFF * 3;
        targetVCC = 3300 * adcValue[1] / 0x0FFF * 3;
	}
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
#endif    
}

/******************************************************************************/
int8_t detectTarget(void)
{
#if defined(DET_TVDD_PORT)
	static bool tValid = false;
	static bool newUp = false;
    bool runBeep;
	
    // Cal output current
	if (targetVDD >= 3300)
        targetCurrent = (uint16_t)((5000 - targetVDD) / 10);

    // Deal with Target Power Up 
	tValid = (targetVDD > 1800) ? true : false;
    runBeep = tValid && !newUp;
	if (tValid){
        if(!newUp)  newUp = true;
    }
	else    newUp = false;
	if (runBeep)    setBeepMode(mode_bibi);
		
	return 0;
#endif
}
