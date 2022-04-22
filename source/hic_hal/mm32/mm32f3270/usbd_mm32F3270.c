/*
 * Copyright (c) 2004-2016 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*----------------------------------------------------------------------------
*      RL-ARM - USB
*----------------------------------------------------------------------------
*      Name:    usbd_MM32F3270.c
*      Purpose: Hardware Layer module for ST STM32F103
*      Rev.:    V4.70
*---------------------------------------------------------------------------*/

/* Double Buffering is not supported                                         */

#include <rl_usb.h>
#include "mm32_device.h"
#include "hal_gpio.h"
#include "hal_rcc.h"

#include "usbreg.h"
#include "IO_Config.h"
#include "cortex_m.h"
#include "string.h"

#define __NO_USB_LIB_C
#include "usb_config.c"



/*
 *  USB Device Interrupt enable
 *   Called by USBD_Init to enable the USB Interrupt
 *    Return Value:    None
 */
#ifdef __RTX
void __svc(1) USBD_IntrEna(void);
void __SVC_1(void) {
#else
void          USBD_IntrEna(void) {
#endif
    //NVIC_SetPriority(USB_FS_IRQn, 3);
    NVIC_EnableIRQ(USB_FS_IRQn);
}

/*
 *  USB Device Initialize Function
 *   Called by the User to initialize USB Device
 *    Return Value:    None
 */
void USBD_Init(void)
{
    /* Select USBCLK source */
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div2);

    /* Enable USB clock */
    RCC->AHB2ENR |= 0x1 << 7;
	
    USBD_Reset();

    /* Enable USB Interrupts */
    USBD_IntrEna();
}

/*
 *  USB Device Connect Function
 *   Called by the User to Connect/Disconnect USB Device
 *    Parameters:      con:   Connect/Disconnect
 *    Return Value:    None
 */
void USBD_Connect(BOOL con)
{
    if (con) {

    }
    else {

    }
}

/*
 *  USB Device Reset Function
 *   Called automatically on USB Device Reset
 *    Return Value:    None
 */
void USBD_Reset(void)
{
    NVIC_DisableIRQ(USB_FS_IRQn);

    /* Clear Interrupt Status */
    //ISTR = 0;                            
    //CNTR = CNTR_CTRM | CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM |
#ifdef __RTX
           ((USBD_RTX_DevTask   != 0) ? CNTR_ERRM    : 0) |
           ((USBD_RTX_DevTask   != 0) ? CNTR_PMAOVRM : 0) |
           ((USBD_RTX_DevTask   != 0) ? CNTR_SOFM    : 0) |
           ((USBD_RTX_DevTask   != 0) ? CNTR_ESOFM   : 0);
#else
           //((USBD_P_Error_Event != 0) ? CNTR_ERRM    : 0) |
           //((USBD_P_Error_Event != 0) ? CNTR_PMAOVRM : 0) |
           //((USBD_P_SOF_Event   != 0) ? CNTR_SOFM    : 0) |
           //((USBD_P_SOF_Event   != 0) ? CNTR_ESOFM   : 0);
#endif

    /* set BTABLE Address */
    //FreeBufAddr = EP_BUF_ADDR;
    //BTABLE = 0x00;      

    /* Setup Control Endpoint 0 */
    //pBUF_DSCR->ADDR_TX = FreeBufAddr;
    //FreeBufAddr += USBD_MAX_PACKET0;
    //pBUF_DSCR->ADDR_RX = FreeBufAddr;
    //FreeBufAddr += USBD_MAX_PACKET0;
	//
    //if (USBD_MAX_PACKET0 > 62) {
    //    pBUF_DSCR->COUNT_RX = ((USBD_MAX_PACKET0 << 5) - 1) | 0x8000;
    //} else {
    //    pBUF_DSCR->COUNT_RX =   USBD_MAX_PACKET0 << 9;
    //}

    /* Enable USB Default Address */
    //EPxREG(0) = EP_CONTROL | EP_RX_VALID;
    //DADDR = DADDR_EF | 0;

    NVIC_EnableIRQ(USB_FS_IRQn);
}

/*
 *  USB Device Suspend Function
 *   Called automatically on USB Device Suspend
 *    Return Value:    None
 */
void USBD_Suspend(void)
{
    CNTR |= CNTR_FSUSP;                   /* Force Suspend                      */
    CNTR |= CNTR_LPMODE;                  /* Low Power Mode                     */
}

/*
 *  USB Device Resume Function
 *   Called automatically on USB Device Resume
 *    Return Value:    None
 */
void USBD_Resume(void)
{
    
}

/*
 *  USB Device Remote Wakeup Function
 *   Called automatically on USB Device Remote Wakeup
 *    Return Value:    None
 */
void USBD_WakeUp(void)
{
    /* Clear Suspend */
    USB_OTG_FS->CTL &= ~CNTR_FSUSP;
}

/*
 *  USB Device Remote Wakeup Configuration Function
 *    Parameters:      cfg:   Device Enable/Disable
 *    Return Value:    None
 */
void USBD_WakeUpCfg(BOOL cfg)
{
    //NULL
}

/*
 *  USB Device Set Address Function
 *    Parameters:      adr:   USB Device Address
 *                     setup: Called in setup stage (!=0), else after status stage
 *    Return Value:    None
 */
void USBD_SetAddress(U32 adr, U32 setup)
{
    if (setup) {    //????chend
        return;
    }
    USB_OTG_FS->ADDR = adr;
}

/*
 *  USB Device Configure Function
 *    Parameters:      cfg:   Device Configure/Deconfigure
 *    Return Value:    None
 */
void USBD_Configure(BOOL cfg)
{
    if (cfg == __FALSE) {
        /* reset Buffer address*/
        //g_u32FreeBufAddr = CEP_BUF_BASE + CEP_BUF_LEN;

    }
}

/******************************************************************************/
void USBD_ConfigEP(USB_ENDPOINT_DESCRIPTOR *pEPD)
{
    
}

/******************************************************************************/
void USBD_DirCtrlEP(U32 dir)
{
    
}

/******************************************************************************/
void USBD_EnableEP(U32 EPNum)
{
    
}

/******************************************************************************/
void USBD_DisableEP(U32 EPNum)
{
    
}

/******************************************************************************/
void USBD_ResetEP(U32 EPNum)
{
    
}

/******************************************************************************/
void USBD_SetStallEP(U32 EPNum)
{
    
}

/******************************************************************************/
void USBD_ClrStallEP(U32 EPNum)
{
    
}

/******************************************************************************/
void USBD_ClearEPBuf(U32 EPNum)
{
    
}

/******************************************************************************/
U32 USBD_ReadEP(U32 EPNum, U8 *pData, U32 cnt)
{
    return 0;
}

/******************************************************************************/
U32 USBD_WriteEP(U32 EPNum, U8 *pData, U32 cnt)
{
    return 0;
}

/******************************************************************************/
U32 USBD_GetFrame(void)
{
    return 0;
}

/******************************************************************************/
U32 USBD_GetError(void)
{
    return 0;
}

/******************************************************************************/
//void USBD_SignalHandler(void)
//{
//    
//}

/******************************************************************************/
void USBD_Handler(void)
{
    
}

void OTG_FS_IRQHandler(void)
{
    USBD_SignalHandler();
}
