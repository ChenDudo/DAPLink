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
#define ARR_VALUE       416

////////////////////////////////////////////////////////////////////////////////
/// @defgroup BEEP_Exported_Variables
/// @{
#ifdef _BEEP_C_

#define GLOBAL
#else
#define GLOBAL extern
#endif

#undef GLOBAL

/// @}

////////////////////////////////////////////////////////////////////////////////
/// @defgroup BEEP_Exported_Functions
/// @{
void BEEP_on(void);
void BEEP_off(void);
void InitBeep(void);
void BEEP_Hz(int pulse);

/// @}

/// @}

/// @}

////////////////////////////////////////////////////////////////////////////////
#endif /*__BSP_BEEP_H */
////////////////////////////////////////////////////////////////////////////////
