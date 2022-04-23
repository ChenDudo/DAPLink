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
#include "hal_misc.h"
//#include "hal_crs.h"
//#include "usbreg.h"
#include "reg_usb_otg_fs.h"
#include "IO_Config.h"
#include "cortex_m.h"
#include "string.h"

#define __NO_USB_LIB_C
#include "usb_config.c"


// configure BDT table
#define OTG_BUFFER_BASE     0x50000000

/* The maximum value of ISO maximum packet size for FS in USB specification 2.0 */
#define USB_DEVICE_MAX_FS_ISO_MAX_PACKET_SIZE (1023U)

/* The maximum value of non-ISO maximum packet size for FS in USB specification 2.0 */
#define USB_DEVICE_MAX_FS_NONE_ISO_MAX_PACKET_SIZE (64U)

/* Set BDT buffer address */
#define BDT_SET_ADDRESS(bdt_base, ep, direction, odd, address)                          \
    *((volatile uint32_t *)((bdt_base & 0xfffffe00U) | (((uint32_t)ep & 0x0fU) << 5U) |          \
                            (((uint32_t)direction & 1U) << 4U) | (((uint32_t)odd & 1U) << 3U)) + \
      1U) = address

/* Set BDT control fields*/
#define BDT_SET_CONTROL(bdt_base, ep, direction, odd, control)                \
    *(volatile uint32_t *)((bdt_base & 0xfffffe00U) | (((uint32_t)ep & 0x0fU) << 5U) | \
                           (((uint32_t)direction & 1U) << 4U) | (((uint32_t)odd & 1U) << 3U)) = control

/* Get BDT buffer address*/
#define BDT_GET_ADDRESS(bdt_base, ep, direction, odd)                                    \
    (*((volatile uint32_t *)((bdt_base & 0xfffffe00U) | (((uint32_t)ep & 0x0fU) << 5U) |          \
                             (((uint32_t)direction & 1U) << 4U) | (((uint32_t)odd & 1U) << 3U)) + \
       1U))

/* Get BDT control fields*/
#define BDT_GET_CONTROL(bdt_base, ep, direction, odd)                          \
    (*(volatile uint32_t *)((bdt_base & 0xfffffe00U) | (((uint32_t)ep & 0x0fU) << 5U) | \
                            (((uint32_t)direction & 1U) << 4U) | (((uint32_t)odd & 1U) << 3U)))


////////////////////////////////////////////////////////////////////////////////
void USB_NVIC_Config(void)
{
	//???chend: no use
    NVIC_InitTypeDef NVIC_InitStruct;

    NVIC_InitStruct.NVIC_IRQChannel = USB_FS_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

////////////////////////////////////////////////////////////////////////////////
void USB_Clock_Config(void)
{
	/* configure CRS */
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_CRS, ENABLE);
    CRS->CFGR &= ~CRS_CFGR_RELOAD;
    CRS->CFGR |= 0xBB7F;	// RELOAD=47999
    CRS->CFGR &= ~CRS_CFGR_SRC;
    CRS->CFGR |= CRS_CFGR_SRC_USBSOF;
    CRS->CR |= CRS_CR_AUTOTRIMEN;
    CRS->CR |= CRS_CR_CNTEN;
	
    /**/
    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIO, ENABLE);

	/* Select USBCLK source */
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div2);

	/* Enable USB clock */
	RCC_AHB2PeriphClockCmd(RCC_AHB2ENR_USBFS, ENABLE);
}

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
	USB_Clock_Config();
    USBD_Reset();
    USBD_IntrEna();
    USB_OTG_FS->ADDR = 0; // USB default address = 0
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
        USB_OTG_FS->CTL |= OTG_FS_CTL_USB_EN_SOF_EN;
    }
    else {
        USB_OTG_FS->CTL &= ~OTG_FS_CTL_USB_EN_SOF_EN;
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
    USB_OTG_FS->OTG_ISTAT |= 0xED;

    USB_OTG_FS->CTL &= OTG_FS_CTL_HOST_MODE_EN;

    /* set BTABLE Address */


    /* Setup Control Endpoint 0 */


    /* Enable USB Default Address */


    NVIC_EnableIRQ(USB_FS_IRQn);
}

/*
 *  USB Device Suspend Function
 *   Called automatically on USB Device Suspend
 *    Return Value:    None
 */
void USBD_Suspend(void)
{
    /* Performed by Hardware */
}

/*
 *  USB Device Resume Function
 *   Called automatically on USB Device Resume
 *    Return Value:    None
 */
void USBD_Resume(void)
{
    /* Performed by Hardware */
}

/*
 *  USB Device Remote Wakeup Function
 *   Called automatically on USB Device Remote Wakeup
 *    Return Value:    None
 */
void USBD_WakeUp(void)
{
    // no neeed
}

/*
 *  USB Device Remote Wakeup Configuration Function
 *    Parameters:      cfg:   Device Enable/Disable
 *    Return Value:    None
 */
void USBD_WakeUpCfg(BOOL cfg)
{
    // no need
}

/*
 *  USB Device Set Address Function
 *    Parameters:      adr:   USB Device Address
 *                     setup: Called in setup stage (!=0), else after status stage
 *    Return Value:    None
 */
void USBD_SetAddress(U32 adr, U32 setup)
{
    if (!setup) {
        USB_OTG_FS->ADDR = adr;
    }
}

/*
 *  USB Device Configure Function
 *    Parameters:      cfg:   Device Configure/Deconfigure
 *    Return Value:    None
 */
void USBD_Configure(BOOL cfg)
{
    if (!cfg) {
        /* reset endpoint Buffer*/
        

    }
}


/* USB Standard Endpoint Descriptor */
// typedef __PACKED_STRUCT _USB_ENDPOINT_DESCRIPTOR {
//     U8  bLength;
//     U8  bDescriptorType;
//     U8  bEndpointAddress;
//     U8  bmAttributes;
//     U16 wMaxPacketSize;
//     U8  bInterval;
// } USB_ENDPOINT_DESCRIPTOR;

#define USB_IN_MASK         (0x80)

/******************************************************************************/
void USBD_ConfigEP(USB_ENDPOINT_DESCRIPTOR *pEPD)
{
    uint32_t num, val;
    
    U32 num, val;
    num = pEPD->bEndpointAddress & 0x0F;
    val = pEPD->wMaxPacketSize;
    type = pEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK;

    /* IN EPs */
    if (num & USB_IN_MASK) {

    }
    /* OUT EPs */
    else {
        EPBufInfo[EP_OUT_IDX(num)].buf_len  = val;
        EPBufInfo[EP_OUT_IDX(num)].buf_ptr  = s_next_ep_buf_addr;
        ptr  = GetEpCmdStatPtr(num);
        *ptr = N_BYTES(EPBufInfo[EP_OUT_IDX(num)].buf_len) |
               BUF_ADDR(EPBufInfo[EP_OUT_IDX(num)].buf_ptr) |
               EP_DISABLED;

        if (type == USB_ENDPOINT_TYPE_INTERRUPT) {
            *ptr |= EP_TYPE | EP_RF_TV;
        }
        else if (type == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
            *ptr |= EP_TYPE;
        }

        s_next_ep_buf_addr += ROUND_UP(val, MIN_BUF_SIZE);     /* calc new free buffer address */
    }
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
void USBD_Handler(void)
{
    //TODO: handle

    // end
    NVIC_EnableIRQ(USB_FS_IRQn);
}

void OTG_FS_IRQHandler(void)
{
    NVIC_DisableIRQ(USB_FS_IRQn);
    USBD_SignalHandler();
}
