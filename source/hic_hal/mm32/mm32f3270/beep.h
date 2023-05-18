////////////////////////////////////////////////////////////////////////////////
/// @file     BSP_BEEP.H
/// @author   Y Xu
/// @version  v1.0.0
/// @date     2021-09-28
/// @brief    THIS FILE CONTAINS ALL THE FUNCTIONS PROTOTYPES FOR THE BEEP
///           BSP LAYER.
////////////////////////////////////////////////////////////////////////////////
/// @attention
///
/// THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
/// CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
/// TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
/// CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
/// HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
/// CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
///
/// <H2><CENTER>&COPY; COPYRIGHT 2018-2019 MINDMOTION </CENTER></H2>
////////////////////////////////////////////////////////////////////////////////

// Define to prevent recursive inclusion  --------------------------------------
#ifndef __BEEP_H
#define __BEEP_H


// TIM4 CH3
#define BEEP_TIMER      TIM4
#define ARR_VALUE       416 //208//104 //416

typedef enum {
	mode_none   = 0x00,
    mode_bi     = 0x01,
    mode_bibi   = 0x05,
    mode_bi_bi  = 0x09,
    mode_bi__bi = 0x11,
    mode_bibibi = 0x15,
    mode_bii    = 0x03,
    mode_biii   = 0x07,
    mode_biiii  = 0x0F,
    mode_biiiii = 0x1F
} embeepMode;

////////////////////////////////////////////////////////////////////////////////
/// @defgroup BEEP_Exported_Variables
/// @{
#ifdef _BEEP_C_

#define GLOBAL
#else
#define GLOBAL extern
#endif

GLOBAL bool     beepEn;
GLOBAL uint8_t  beepCount;
GLOBAL embeepMode beepMode;
GLOBAL uint16_t	adcValue[2];
GLOBAL float	targetVDD;
GLOBAL float	targetVCC;
GLOBAL uint16_t	targetCurrent;
GLOBAL bool     gResetNeedflag;
GLOBAL bool     gProgrammer_TrueFlag;
GLOBAL bool     gF0010_TrueFlag;
GLOBAL bool     gPoweronFlag;
GLOBAL bool     gPoweroffFlag;
GLOBAL uint16_t gProgrammer_timeoutcnt;
#undef GLOBAL

/// @}

////////////////////////////////////////////////////////////////////////////////
/// @defgroup BEEP_Exported_Functions
/// @{
void BEEP_on(void);
void BEEP_off(void);
void InitBeep(void);
void BEEP_Hz(int pulse);
void Beep_Tick(void);
void setBeepMode(embeepMode mode);
void initADC(void);
void adcTick(void);

int8_t detectTarget(void);	//chendo new add 0804

/// @}

/// @}

/// @}

////////////////////////////////////////////////////////////////////////////////
#endif /*__BSP_BEEP_H */
////////////////////////////////////////////////////////////////////////////////
