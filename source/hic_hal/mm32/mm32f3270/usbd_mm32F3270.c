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



////////////////////////////////////////////////////////////////////////////////
//! Define Buffer Descripter Table
////////////////////////////////////////////////////////////////////////////////
// #if defined(__ICCARM__)
// #define USB_WEAK_VAR                    __attribute__((weak))
// #define USB_WEAK_FUN                    __attribute__((weak))
// _Pragma("diag_suppress=Pm120")
// #define USB_ALIGN_PRAGMA(x)             _Pragma(#x)
// _Pragma("diag_default=Pm120")
// #define USB_RAM_ADDRESS_ALIGNMENT(n)    USB_ALIGN_PRAGMA(data_alignment = n)
// _Pragma("diag_suppress=Pm120")
// #define USB_LINK_SECTION_PART(str)      _Pragma(#str)
// #define USB_LINK_DMA_INIT_DATA(sec)     USB_LINK_SECTION_PART(location = #sec)
// #define USB_LINK_USB_GLOBAL             _Pragma("location = \"m_usb_global\"")
// #define USB_LINK_USB_BDT                _Pragma("location = \"m_usb_bdt\"")
// #define USB_LINK_USB_GLOBAL_BSS         _Pragma("location = \".bss.m_usb_global\"")
// #define USB_LINK_USB_BDT_BSS            _Pragma("location = \".bss.m_usb_bdt\"")
// _Pragma("diag_default=Pm120")
// #define USB_LINK_DMA_NONINIT_DATA       _Pragma("location = \"m_usb_dma_noninit_data\"")
// #define USB_LINK_NONCACHE_NONINIT_DATA  _Pragma("location = \"NonCacheable\"")

// #elif defined(__CC_ARM)
// #define USB_WEAK_VAR                    __attribute__((weak))
// #define USB_WEAK_FUN                    __weak
// #define USB_RAM_ADDRESS_ALIGNMENT(n)    __attribute__((aligned(n)))
// #define USB_LINK_DMA_INIT_DATA(sec)     __attribute__((section(#sec)))
// #define USB_LINK_USB_GLOBAL             __attribute__((section("m_usb_global")))            __attribute__((zero_init))
// #define USB_LINK_USB_BDT                __attribute__((section("m_usb_bdt")))               __attribute__((zero_init))
// #define USB_LINK_USB_GLOBAL_BSS         __attribute__((section(".bss.m_usb_global")))       __attribute__((zero_init))
// #define USB_LINK_USB_BDT_BSS            __attribute__((section(".bss.m_usb_bdt")))          __attribute__((zero_init))
// #define USB_LINK_DMA_NONINIT_DATA       __attribute__((section("m_usb_dma_noninit_data")))  __attribute__((zero_init))
// #define USB_LINK_NONCACHE_NONINIT_DATA  __attribute__((section("NonCacheable")))            __attribute__((zero_init))

// #elif defined(__GNUC__)
// #define USB_WEAK_VAR                    __attribute__((weak))
// #define USB_WEAK_FUN                    __attribute__((weak))
// #define USB_RAM_ADDRESS_ALIGNMENT(n)    __attribute__((aligned(n)))
// #define USB_LINK_DMA_INIT_DATA(sec)     __attribute__((section(#sec)))
// #define USB_LINK_USB_GLOBAL             __attribute__((section("m_usb_global, \"aw\", %nobits @")))
// #define USB_LINK_USB_BDT                __attribute__((section("m_usb_bdt, \"aw\", %nobits @")))
// #define USB_LINK_USB_GLOBAL_BSS         __attribute__((section(".bss.m_usb_global, \"aw\", %nobits @")))
// #define USB_LINK_USB_BDT_BSS            __attribute__((section(".bss.m_usb_bdt, \"aw\", %nobits @")))
// #define USB_LINK_DMA_NONINIT_DATA       __attribute__((section("m_usb_dma_noninit_data, \"aw\", %nobits @")))
// #define USB_LINK_NONCACHE_NONINIT_DATA  __attribute__((section("NonCacheable, \"aw\", %nobits @")))

// #else
// #error The tool-chain is not supported.
// #endif

// #define USB_BDT				USB_LINK_USB_BDT_BSS
// USB_BDT USB_RAM_ADDRESS_ALIGNMENT(512) static uint8_t s_UsbDeviceKhciBdtBuffer[1][512U];
// #define BDT_BASE            (uint32_t)(s_UsbDeviceKhciBdtBuffer[1])

/* Set BDT buffer address */
#define USB_KHCI_BDT_SET_ADDRESS(BDT_BASE, ep, direction, odd, address)         \
    *((volatile uint32_t *)((BDT_BASE & 0xfffffe00U) | \
    (((uint32_t)ep & 0x0fU) << 5U) | (((uint32_t)direction & 1U) << 4U) | \
    (((uint32_t)odd & 1U) << 3U)) + 1U) = address

/* Set BDT control fields*/
#define USB_KHCI_BDT_SET_CONTROL(BDT_BASE, ep, direction, odd, control)         \
    *(volatile uint32_t *)((BDT_BASE & 0xfffffe00U) | \
    (((uint32_t)ep & 0x0fU) << 5U) | (((uint32_t)direction & 1U) << 4U) | \
    (((uint32_t)odd & 1U) << 3U)) = control

/* Get BDT buffer address*/
#define USB_KHCI_BDT_GET_ADDRESS(BDT_BASE, ep, direction, odd)                  \
    (*((volatile uint32_t *)((BDT_BASE & 0xfffffe00U) | \
    (((uint32_t)ep & 0x0fU) << 5U) | (((uint32_t)direction & 1U) << 4U) | \
    (((uint32_t)odd & 1U) << 3U)) + 1U))

/* Get BDT control fields*/
#define USB_KHCI_BDT_GET_CONTROL(BDT_BASE, ep, direction, odd)                  \
    (*(volatile uint32_t *)((BDT_BASE & 0xfffffe00U) | \
    (((uint32_t)ep & 0x0fU) << 5U) | (((uint32_t)direction & 1U) << 4U) |\
    (((uint32_t)odd & 1U) << 3U)))

/* USB  standard descriptor endpoint type */
#define USB_ENDPOINT_CONTROL        (0x00U)
#define USB_ENDPOINT_ISOCHRONOUS    (0x01U)
#define USB_ENDPOINT_BULK           (0x02U)
#define USB_ENDPOINT_INTERRUPT      (0x03U)

/* USB  standard descriptor transfer direction (cannot change the value because iTD use the value directly) */
#define USB_OUT (0U)
#define USB_IN (1U)

////////////////////////////////////////////////////////////////////////////////
//! Define Buffer Address
////////////////////////////////////////////////////////////////////////////////
#define EP_LIST_BASE                (0x20004000)
#define BDT_BASE                    EP_LIST_BASE
#define USBD_MAX_EPS 		        (16)
#define USB_DEVICE_CONFIG_ENDPOINTS USBD_MAX_EPS

#define MIN_BUF_SIZE                (64)

#if defined ( __CC_ARM ) || defined (__ARMCC_VERSION)
volatile uint32_t EPList[USBD_MAX_EPS * 4]  __attribute__((at(EP_LIST_BASE)));
#elif defined ( __GNUC__ )
volatile uint32_t EPList[USBD_MAX_EPS * 4]  __attribute__((section(".usbram")));
#else
#error "Unsupported compiler!"
#endif



#define USB_ENDPOINT_NUMBER_MASK                        (0x0FU)
#define USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK  (0x80U)
#define USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT (7U)
#define USB_IN_MASK                 (0x80)
#define USB_LOGICAL_EP_MASK         (0x7f)

#define USB_CONTROL_ENDPOINT        (0U)
#define USB_CONTROL_MAX_PACKET_SIZE (64U)
#define USB_SETUP_PACKET_SIZE       (8U)

#define EP_BUF_BASE             (uint32_t)(EP_LIST_BASE + 0x200) // align 512 Byte
#define EP0_OUT_BUF_OFFSET      (0)
#define EP0_IN_BUF_OFFSET       (2)
#define EP0_OUT_BUF_BASE        (EP0_OUT_BUF_OFFSET * USBD_MAX_PACKET0 + EP_BUF_BASE)
#define EP0_IN_BUF_BASE         (EP0_IN_BUF_OFFSET * USBD_MAX_PACKET0 + EP_BUF_BASE)
#define EP0_SETUP_BUF_BASE      EP0_OUT_BUF_BASE

#define EP1_BUF_BASE           (4 * USBD_MAX_PACKET0 + EP_BUF_BASE) //(uint32_t)(EP_LIST_BASE + 0x400) 
//? The EP1 OUT buffer starts after the EP0 OUT buffer, SETUP buffer, and EP0 IN buffer.

#define USB_KHCI_BDT_DEVICE_OUT_TOKEN   (0x01U)
#define USB_KHCI_BDT_DEVICE_IN_TOKEN    (0x09U)
#define USB_KHCI_BDT_DEVICE_SETUP_TOKEN (0x0DU)
#define USB_KHCI_BDT_OWN                (0x80U)
#define USB_LONG_TO_LITTLE_ENDIAN(n)    (n)
#define USB_KHCI_BDT_DATA01(x)          ((((uint32_t)(x)) & 0x01U) << 0x06U)
#define USB_KHCI_BDT_BC(x)              ((((uint32_t)(x)) & 0x3FFU) << 0x10U)
#define UBS_KHCI_BDT_KEEP               (0x20U)
#define UBS_KHCI_BDT_NINC               (0x10U)
#define USB_KHCI_BDT_DTS                (0x08U)
#define USB_KHCI_BDT_STALL              (0x04U)
#define _UBS_KHCI_BDT_KEEP              (0x0U)
#define _UBS_KHCI_BDT_NINC              (0x0U)
#define _USB_KHCI_BDT_DTS               (0x0U)
#define _USB_KHCI_BDT_STALL             (0x0U)


typedef struct BUF_INFO {
    uint8_t* transferBuffer; /*!< Address of buffer containing the data to be transmitted */
    uint32_t transferLength; /*!< Length of data to transmit. */
    uint32_t transferDone;   /*!< The data length has been transferred*/
    union {
        uint32_t state; /*!< The state of the endpoint */
        struct {
            uint32_t maxPacketSize : 10U; /*!< The maximum packet size of the endpoint */
            uint32_t stalled : 1U;        /*!< The endpoint is stalled or not */
            uint32_t data0 : 1U;          /*!< The data toggle of the transaction */
            uint32_t bdtOdd : 1U;         /*!< The BDT toggle of the endpoint */
            uint32_t dmaAlign : 1U;       /*!< Whether the transferBuffer is DMA aligned or not */
            uint32_t transferring : 1U;   /*!< The endpoint is transferring */
            uint32_t zlt : 1U;            /*!< zlt flag */
        } stateBitField;
    } stateUnion;
} EP_BUF_INFO;
EP_BUF_INFO endpointState[USB_DEVICE_CONFIG_ENDPOINTS*2]; //EPBufState[(USBD_EP_NUM + 1) * 2];

static uint32_t s_next_ep_buf_addr      = EP1_BUF_BASE;
static uint32_t s_read_ctrl_out_next    = 0;
uint32_t control = 0x00;
uint8_t isResetting = 0x00U;

void ConfigureBDT(void)
{
    USB_KHCI_BDT_SET_ADDRESS(BDT_BASE, 0, 0, 0, EP0_OUT_BUF_BASE);
    USB_KHCI_BDT_SET_ADDRESS(BDT_BASE, 0, 0, 1, EP0_SETUP_BUF_BASE);
    USB_KHCI_BDT_SET_ADDRESS(BDT_BASE, 0, 1, 0, EP0_IN_BUF_BASE);
}

void SetBDTPage(uint32_t addr)
{
    USB_OTG_FS->BDT_PAGE_01 = (addr >>  8) & 0x00FFU;
    USB_OTG_FS->BDT_PAGE_01 = (addr >> 16) & 0x00FFU;
    USB_OTG_FS->BDT_PAGE_01 = (addr >> 24) & 0x00FFU;
}

/*
 *  Get EP CmdStat pointer
 *    Parameters:    EPNum: endpoint number
 */
uint32_t *GetEpCmdStatPtr(uint32_t EPNum)
{
    uint32_t ptr = 0;

    if (EPNum & USB_IN_MASK) {
        EPNum &= ~USB_IN_MASK;
        ptr = 8;
    }

    ptr += EP_LIST_BASE + EPNum * 16;
    return ((uint32_t *)ptr);
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
    ConfigureBDT();
    SetBDTPage(EP_LIST_BASE);
    
    USBD_IntrEna();
    USBD_Reset();
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

void USB_DeviceKhciEndpointTransfer(uint8_t endpoint, uint8_t direction, uint8_t* buffer, uint32_t length)
{
    uint32_t index = ((uint32_t)endpoint << 1U) | (uint32_t)direction;

    /* Flag the endpoint is busy. */
    endpointState[index].stateUnion.stateBitField.transferring = 1U;

    /* Add the data buffer address to the BDT. */
    USB_KHCI_BDT_SET_ADDRESS(BDT_BASE, endpoint, direction, endpointState[index].stateUnion.stateBitField.bdtOdd, (uint32_t)buffer);

    /* Change the BDT control field to start the transfer. */
    USB_KHCI_BDT_SET_CONTROL(BDT_BASE, endpoint, direction, endpointState[index].stateUnion.stateBitField.bdtOdd,\
        USB_LONG_TO_LITTLE_ENDIAN(USB_KHCI_BDT_BC(length) | USB_KHCI_BDT_OWN | USB_KHCI_BDT_DTS | USB_KHCI_BDT_DATA01(endpointState[index].stateUnion.stateBitField.data0)));

    /* Clear the token busy state */
    USB_OTG_FS->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;
}

static void USB_DeviceKhciPrimeNextSetup(void)
{
    /* Update the endpoint state */
    /* Save the buffer address used to receive the setup packet. */
    endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].transferBuffer = (uint8_t*)(EP0_SETUP_BUF_BASE)+endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].stateUnion.stateBitField.bdtOdd * USB_SETUP_PACKET_SIZE;
    /* Clear the transferred length. */
    endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].transferDone = 0U;
    /* Save the data length expected to get from a host. */
    endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].transferLength = USB_SETUP_PACKET_SIZE;
    /* Save the data buffer DMA align flag. */
    endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].stateUnion.stateBitField.dmaAlign = 1U;
    /* Set the DATA0/1 to DATA0. */
    endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].stateUnion.stateBitField.data0 = 0U;

    USB_DeviceKhciEndpointTransfer(USB_CONTROL_ENDPOINT, USB_OUT, endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].transferBuffer, USB_SETUP_PACKET_SIZE);
}


NO_OPTIMIZE_PRE
void NO_OPTIMIZE_INLINE USBD_Reset(void)
{
    NVIC_DisableIRQ(USB_FS_IRQn);

    /* Clear the error state register */
    USB_OTG_FS->ERR_STAT = 0xFFU;
    /* Setting this bit to 1U resets all the BDT ODD ping/pong fields to 0U, which then specifies the EVEN BDT bank. */
    USB_OTG_FS->CTL |= USB_CTL_ODDRST_MASK;
    /* Clear the device address */
    USB_OTG_FS->ADDR = 0U;
    /* Clear the endpoint state and disable the endpoint */
    for (uint8_t count = 0U; count < USB_DEVICE_CONFIG_ENDPOINTS; count++) {
        USB_KHCI_BDT_SET_CONTROL(BDT_BASE, count, USB_OUT, 0U, 0U);
        USB_KHCI_BDT_SET_CONTROL(BDT_BASE, count, USB_OUT, 1U, 0U);
        USB_KHCI_BDT_SET_CONTROL(BDT_BASE, count, USB_IN, 0U, 0U);
        USB_KHCI_BDT_SET_CONTROL(BDT_BASE, count, USB_IN, 1U, 0U);
        endpointState[((uint32_t)count << 1U) | USB_OUT].stateUnion.state = 0U;
        endpointState[((uint32_t)count << 1U) | USB_IN].stateUnion.state = 0U;
        USB_OTG_FS->EP_CTL[count] = 0x00U;
    }

    uint32_t index = (uint8_t)(0 << 1U) | (uint8_t)USB_OUT;
    endpointState[index].transferBuffer = (uint8_t*)EP0_SETUP_BUF_BASE;
    /* Set the endpoint idle */
    endpointState[index].stateUnion.stateBitField.transferring = 0U;
    /* Save the max packet size of the endpoint */
    endpointState[index].stateUnion.stateBitField.maxPacketSize = USBD_MAX_PACKET0;
    /* Set the data toggle to DATA0 */
    endpointState[index].stateUnion.stateBitField.data0 = 0U;
    /* Clear the endpoint stalled state */
    endpointState[index].stateUnion.stateBitField.stalled = 0U;
    /* Set the ZLT field */
    //endpointState[index].stateUnion.stateBitField.zlt = ;
    
    /* Enable the endpoint. */
    USB_OTG_FS->EP_CTL[0] |= USB_ENDPT_EPRXEN_MASK;

    /* Add the data buffer address to the BDT. */
    USB_KHCI_BDT_SET_ADDRESS(BDT_BASE, USB_CONTROL_ENDPOINT, USB_OUT, endpointState[index].stateUnion.stateBitField.bdtOdd, EP0_SETUP_BUF_BASE);
    /* Change the BDT control field to start the transfer. */
    USB_KHCI_BDT_SET_CONTROL(BDT_BASE, USB_CONTROL_ENDPOINT, USB_OUT, endpointState[index].stateUnion.stateBitField.bdtOdd, \
        (uint32_t)(USB_KHCI_BDT_BC(USBD_MAX_PACKET0) | USB_KHCI_BDT_OWN | USB_KHCI_BDT_DTS | USB_KHCI_BDT_DATA01(endpointState[index].stateUnion.stateBitField.data0)));
    /* Clear the token busy state */
    USB_OTG_FS->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;

    /* Enable all error */
    USB_OTG_FS->ERR_ENB = 0xFFU;
    USB_OTG_FS->CTL |= OTG_FS_CTL_USB_EN_SOF_EN;    /*USB device enable */
    USB_OTG_FS->OTG_ISTAT |= 0xED;                  /*Clear Interrupt Status */
    USB_OTG_FS->INT_ENB |= OTG_FS_INT_ENB_USB_RST_EN |
                           OTG_FS_INT_ENB_TOK_DNE_EN | /* SOF intr enable */
                           OTG_FS_INT_ENB_STALL_EN;
    /* Clear reset flag */
    isResetting = 0U;

    s_next_ep_buf_addr = EP1_BUF_BASE;

    NVIC_EnableIRQ(USB_FS_IRQn);
}
NO_OPTIMIZE_POST

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
    if (cfg) {
        
    }
    else {
        
    }
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
        USB_OTG_FS->ADDR &= ~OTG_FS_ADDR_ADDR;
        USB_OTG_FS->ADDR |= adr | 0x7FU;
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

    }
    s_next_ep_buf_addr = EP1_BUF_BASE;
}

/*
 *  Configure USB Device Endpoint according to Descriptor
 *    Parameters:      pEPD:  Pointer to USB_ENDPOINT_DESCRIPTOR
 *                  U8  bLength;
 *                  U8  bDescriptorType;
 *                  U8  bEndpointAddress;
 *                  U8  bmAttributes;
 *                  U16 wMaxPacketSize;
 *                  U8  bInterval;
 *    Return Value:    None
 */
void USBD_ConfigEP(USB_ENDPOINT_DESCRIPTOR *pEPD)
{
    uint32_t num, val, type;
    uint8_t endpoint, direction;
    uint32_t index;

    num  = pEPD->bEndpointAddress;
    val  = pEPD->wMaxPacketSize;
    type = pEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK;
    endpoint = (num & 0x0FU);
    direction = (num & 0x80U) >> 7;
    index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;

    /* Make the endpoint max packet size align with USB Specification 2.0. */
    if (USB_ENDPOINT_ISOCHRONOUS == type) {
        if (val > 1023U)    val = 1023U;
    }
    else {
        if (val > 64U)      val = 64U;
        /* Enable an endpoint to perform handshaking during a transaction to this endpoint. */
        USB_OTG_FS->EP_CTL[endpoint] |= USB_ENDPT_EPHSHK_MASK;
    }
    /* Set the endpoint idle */
    endpointState[index].stateUnion.stateBitField.transferring = 0U;
    /* Save the max packet size of the endpoint */
    endpointState[index].stateUnion.stateBitField.maxPacketSize = val;
    /* Set the data toggle to DATA0 */
    endpointState[index].stateUnion.stateBitField.data0 = 0U;
    /* Clear the endpoint stalled state */
    endpointState[index].stateUnion.stateBitField.stalled = 0U;
    /* Set the ZLT field */
    //endpointState[index].stateUnion.stateBitField.zlt = pEPD->bInterval; //TODO ???

    // /* Enable the endpoint. */   // TODO ???
    // USB_OTG_FS->EP_CTL[endpoint] |= (USB_IN == direction) ? USB_ENDPT_EPTXEN_MASK : USB_ENDPT_EPRXEN_MASK;
    
    /* Prime a transfer to receive next setup packet when the endpoint is control out endpoint. */
    if (USB_OUT == direction) {
        uint8_t epIndex = (endpoint << 1U) | USB_OUT;
        endpointState[epIndex].transferBuffer = (uint8_t*)s_next_ep_buf_addr;
        /* Clear the transferred length. */
        endpointState[epIndex].transferDone = 0U;
        /* Save the data length expected to get from a host. */
        endpointState[epIndex].transferLength = val; //USB_SETUP_PACKET_SIZE;
        /* Save the data buffer DMA align flag. */
        endpointState[epIndex].stateUnion.stateBitField.dmaAlign = 1U;

        /* Add the data buffer address to the BDT. */
        USB_KHCI_BDT_SET_ADDRESS(BDT_BASE, endpoint, USB_OUT, endpointState[epIndex].stateUnion.stateBitField.bdtOdd, (uint32_t)endpointState[epIndex].transferBuffer);
        /* Change the BDT control field to start the transfer. */
        USB_KHCI_BDT_SET_CONTROL(BDT_BASE, endpoint, USB_OUT, endpointState[epIndex].stateUnion.stateBitField.bdtOdd,\
            (uint32_t)(USB_KHCI_BDT_BC(val) | USB_KHCI_BDT_OWN | USB_KHCI_BDT_DTS | USB_KHCI_BDT_DATA01(endpointState[epIndex].stateUnion.stateBitField.data0)));
        
        //USB_OTG_FS->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK; //* Clear the token busy state */
    }
    else {
        uint8_t epIndex = (endpoint << 1U) | USB_IN;
        endpointState[epIndex].transferBuffer = (uint8_t *)s_next_ep_buf_addr;
        /* Clear the transferred length. */
        endpointState[epIndex].transferDone = 0U;
        /* Save the data length expected to get from a host. */
        endpointState[epIndex].transferLength = val;
        /* Save the data buffer DMA align flag. */
        endpointState[epIndex].stateUnion.stateBitField.dmaAlign = 1U;

        /* Add the data buffer address to the BDT. */
        USB_KHCI_BDT_SET_ADDRESS(BDT_BASE, endpoint, USB_IN, endpointState[epIndex].stateUnion.stateBitField.bdtOdd, (uint32_t)endpointState[epIndex].transferBuffer);
        /* Change the BDT control field to start the transfer. */
        USB_KHCI_BDT_SET_CONTROL(BDT_BASE, endpoint, USB_IN, endpointState[epIndex].stateUnion.stateBitField.bdtOdd,\
            (uint32_t)(USB_KHCI_BDT_BC(val) | USB_KHCI_BDT_OWN | USB_KHCI_BDT_DTS | USB_KHCI_BDT_DATA01(endpointState[epIndex].stateUnion.stateBitField.data0)));
    }

    s_next_ep_buf_addr += ROUND_UP(val, MIN_BUF_SIZE);

#if 0
    /* IN EPs */
    if (num & USB_IN_MASK) {
        num &= ~USB_IN_MASK;
        EPBufInfo[EP_IN_IDX(num)].buf_len  = val;
        EPBufInfo[EP_IN_IDX(num)].buf_ptr  = s_next_ep_buf_addr;
        s_next_ep_buf_addr += ROUND_UP(val, MIN_BUF_SIZE);     /* calc new free buffer address */
        ptr  = GetEpCmdStatPtr(num | USB_IN_MASK);
        *ptr = EPBufInfo[EP_IN_IDX(num)].buf_len;

        if (type == USB_ENDPOINT_TYPE_INTERRUPT) {
            
        }
        else if (type == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
            
        }
    }

    /* OUT EPs */
    else {
        EPBufInfo[EP_OUT_IDX(num)].buf_len  = val;
        EPBufInfo[EP_OUT_IDX(num)].buf_ptr  = s_next_ep_buf_addr;
        ptr  = GetEpCmdStatPtr(num);
        // *ptr = N_BYTES(EPBufInfo[EP_OUT_IDX(num)].buf_len) |
        //        BUF_ADDR(EPBufInfo[EP_OUT_IDX(num)].buf_ptr) |
        //        EP_DISABLED;


        if (type == USB_ENDPOINT_TYPE_INTERRUPT) {
            
        }
        else if (type == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
            
        }

        s_next_ep_buf_addr += ROUND_UP(val, MIN_BUF_SIZE);     /* calc new free buffer address */
    }
    #endif
}

/*
 *  Set Direction for USB Device Control Endpoint
 *    Parameters:      dir:   Out (dir == 0), In (dir <> 0)
 *    Return Value:    None
 */
void USBD_DirCtrlEP(U32 dir)
{
    /* Not needed */
}

/*
 *  Enable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */
void USBD_EnableEP(U32 EPNum)
{
    uint8_t endpoint, direction;
    uint32_t index;

    endpoint = (EPNum & 0x0FU);
    direction = (EPNum & 0x80U) >> 7;
    index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;
    /* Enable the endpoint. */
    //endpointState[index].stateUnion.stateBitField.transferring = 1U;
    USB_OTG_FS->EP_CTL[endpoint] |= (USB_IN == direction) ? USB_ENDPT_EPTXEN_MASK : USB_ENDPT_EPRXEN_MASK;
}

/*
 *  Disable USB Endpoint
 *  Parameters:     EPNum: Endpoint Number
 *                      EPNum.0..3: Address
 *                      EPNum.7:    Dir
 *  Return Value:   None
 */
void USBD_DisableEP(U32 EPNum)
{
    uint8_t endpoint, direction;
    uint32_t index;

    endpoint = (EPNum & 0x0FU);
    direction = (EPNum & 0x80U) >> 7;
    index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;

    endpointState[index].stateUnion.stateBitField.maxPacketSize = 0U;
    USB_OTG_FS->EP_CTL[endpoint] &= (USB_IN == direction) ? ~USB_ENDPT_EPTXEN_MASK : ~USB_ENDPT_EPRXEN_MASK;
}

/*
 *  Reset USB Device Endpoint
 *  Parameters:     EPNum: Device Endpoint Number
 *                      EPNum.0..3: Address
 *                      EPNum.7:    Dir
 *  Return Value:   None
 */
void USBD_ResetEP(U32 EPNum)
{
    // Todo::: reset EP vs deinit EP vs disable EP
    // uint8_t endpoint, direction;
    // uint32_t index;

    // endpoint = (EPNum & 0x0FU);
    // direction = (EPNum & 0x80U) >> 7;
    // index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;

    // /* Prime a transfer to receive next setup packet when the endpoint is a control out endpoint. */
    // if ((USB_CONTROL_ENDPOINT == endpoint) && (USB_OUT == direction)) {
    //     USB_DeviceKhciPrimeNextSetup();
    // }
}

/*
 *  Set Stall for USB Device Endpoint
 *  Parameters:     EPNum: Device Endpoint Number
 *                      EPNum.0..3: Address
 *                      EPNum.7:    Dir
 *  Return Value:   None
 */
void USBD_SetStallEP(U32 EPNum)
{
    uint8_t endpoint, direction;
    uint32_t index;

    endpoint = (EPNum & 0x0FU);
    direction = (EPNum & 0x80U) >> 7;
    index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;

    /* Cancel the transfer of the endpoint */
    if (endpointState[index].stateUnion.stateBitField.transferring) {
        endpointState[index].stateUnion.stateBitField.transferring = 0U;
    }
    /* Set endpoint stall flag. */
    endpointState[index].stateUnion.stateBitField.stalled = 1U;
    /* Set endpoint stall in BDT. And then if the host send a IN/OUT tanscation, the device will response a STALL state.*/
    USB_KHCI_BDT_SET_CONTROL(BDT_BASE, endpoint, direction, endpointState[index].stateUnion.stateBitField.bdtOdd,\
                USB_LONG_TO_LITTLE_ENDIAN((uint32_t)(USB_KHCI_BDT_BC(endpointState[index].stateUnion.stateBitField.maxPacketSize) | \
                    USB_KHCI_BDT_DTS | USB_KHCI_BDT_STALL | USB_KHCI_BDT_OWN)));
    USB_OTG_FS->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;

    if ((USB_CONTROL_ENDPOINT == endpoint) && (USB_OUT == direction)){
        s_read_ctrl_out_next = 0;
    }
}


/*
 *  Clear Stall for USB Device Endpoint
 *  Parameters:     EPNum: Device Endpoint Number
 *                      EPNum.0..3: Address
 *                      EPNum.7:    Dir
 *  Return Value:   None
 */
void USBD_ClrStallEP(U32 EPNum)
{
    uint8_t endpoint, direction;
    uint32_t index;

    endpoint = (EPNum & 0x0FU);
    direction = (EPNum & 0x80U) >> 7;
    index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;

    /* Clear the endpoint stall state */
    endpointState[index].stateUnion.stateBitField.stalled = 0U;
    /* Reset the endpoint data toggle to DATA0 */
    endpointState[index].stateUnion.stateBitField.data0 = 0U;

    /* Clear stall state in BDT */
    for (uint8_t i = 0U; i < 2U; i++) {
        USB_KHCI_BDT_SET_CONTROL(BDT_BASE, endpoint, direction, i,
            USB_LONG_TO_LITTLE_ENDIAN((uint32_t)(USB_KHCI_BDT_BC(endpointState[index].stateUnion.stateBitField.maxPacketSize) | USB_KHCI_BDT_DTS | USB_KHCI_BDT_DATA01(0U))));
    }

    /* Clear stall state in endpoint control register */
    USB_OTG_FS->EP_CTL[endpoint] &= ~USB_ENDPT_EPSTALL_MASK;

    if ((USB_CONTROL_ENDPOINT != endpoint)) {
        if (endpointState[index].stateUnion.stateBitField.transferring) {
            endpointState[index].stateUnion.stateBitField.transferring = 0U;
        }
    }

    /* Prime a transfer to receive next setup packet when the endpoint is a control out endpoint. */
    if ((USB_CONTROL_ENDPOINT == endpoint) && (USB_OUT == direction)) {
        USB_DeviceKhciPrimeNextSetup();
    }

    USB_OTG_FS->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;

}

/*
 *  Clear USB Device Endpoint Buffer
 *  Parameters:     EPNum: Device Endpoint Number
 *                      EPNum.0..3: Address
 *                      EPNum.7:    Dir
 *  Return Value:   None
 */
void USBD_ClearEPBuf(U32 EPNum)
{
    uint32_t  cnt, i;
    uint8_t  *dataptr;
    uint8_t endpoint, direction;
    uint32_t index;
    index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;
    dataptr = endpointState[index].transferBuffer;
    cnt = endpointState[index].transferLength;
    for (i = 0; i < cnt; i++) {
        dataptr[i] = 0;
    }
}

/*
 *  Read USB Device Endpoint Data
 *  Parameters:     EPNum: Device Endpoint Number
 *                      EPNum.0..3: Address
 *                      EPNum.7:    Dir
 *                      pData: Pointer to Data Buffer
 *  Return Value:   Number of bytes read
 */
U32 USBD_ReadEP(U32 EPNum, U8 *pData, U32 cnt)
{// OUT buffer
	uint8_t endpoint, direction;
    uint32_t index;
	uint8_t* buffer;
	
    endpoint = (EPNum & 0x0FU);
    direction = (EPNum & 0x80U) >> 7;
    index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;

    if(USB_IN==direction){
        return 0;
    }

    if ((0U == cnt) && (USB_CONTROL_ENDPOINT == (endpoint & 0x0FU))) {
        endpointState[index].stateUnion.stateBitField.transferring = 0U;
        buffer = endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].transferBuffer;
        USB_DeviceKhciPrimeNextSetup();
        memcpy(pData, buffer, cnt);
    }
    else {
        /* Save the tansfer information */
		buffer = endpointState[index].transferBuffer;
        if (0U == endpointState[index].stateUnion.stateBitField.transferring) {
            endpointState[index].transferDone = 0U;
            endpointState[index].transferBuffer = buffer;
            endpointState[index].transferLength = cnt;
        }
        //endpointState[index].stateUnion.stateBitField.dmaAlign = 1U;

        /* Data length needs to less than max packet size in each call. */
        if (cnt > endpointState[index].stateUnion.stateBitField.maxPacketSize) {
            cnt = endpointState[index].stateUnion.stateBitField.maxPacketSize;
        }

        buffer = (uint8_t*)((uint32_t)endpointState[index].transferBuffer + (uint32_t)endpointState[index].transferDone);

        /* Receive data when the device is not resetting. */
        if (0U == isResetting) {
            USB_DeviceKhciEndpointTransfer(endpoint & USB_ENDPOINT_NUMBER_MASK, USB_OUT, buffer, cnt);
            memcpy(pData, buffer, cnt); //TODO
        }
        //memcpy(pData, buffer, cnt); //TODO
    }
    return 0;
}

/*
 *  Write USB Device Endpoint Data
 *  Parameters:   EPNum: Device Endpoint Number
 *                      EPNum.0..3: Address
 *                      EPNum.7:    Dir
 *                   pData: Pointer to Data Buffer
 *                   cnt:   Number of bytes to write
 *  Return Value:   Number of bytes written
 */
U32 USBD_WriteEP(U32 EPNum, U8 *pData, U32 cnt)
{// IN buffer
    uint8_t endpoint, direction;
    uint32_t index;
    uint8_t* buffer;

    endpoint = (EPNum & 0x0FU);
    direction = (EPNum & 0x80U) >> 7;
    index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;

    /* only IN EP */
    if (USB_OUT == direction) {
		return 0;
    }
    
    /* Save the tansfer information */
    if (0U == endpointState[index].stateUnion.stateBitField.transferring) {
        buffer = endpointState[index].transferBuffer;
        endpointState[index].transferDone = 0U;
        endpointState[index].transferBuffer = buffer;
        endpointState[index].transferLength = cnt;
        //endpointState[index].stateUnion.stateBitField.dmaAlign = 1U; //TODO
    }
    /* Data length needs to less than max packet size in each call. */
    if (cnt > endpointState[index].stateUnion.stateBitField.maxPacketSize) {
        cnt = endpointState[index].stateUnion.stateBitField.maxPacketSize;
    }
	/* Send data when the device is not resetting. */
    if (0U == isResetting) {
        USB_DeviceKhciEndpointTransfer(endpoint, USB_IN,\
			(uint8_t*)((uint32_t)endpointState[index].transferBuffer + (uint32_t)endpointState[index].transferDone), cnt);
    }
	/* Prime a transfer to receive next setup packet if the dat length is zero in a control in endpoint. */
    if ((0U == endpointState[index].transferDone) && (0U == cnt) && (USB_CONTROL_ENDPOINT == endpoint)) {
        USB_DeviceKhciPrimeNextSetup();
    }
    /* Flag the endpoint is busy. */
    endpointState[index].stateUnion.stateBitField.transferring = 1U;
    return 0;
}

/*
 *  Get USB Device Last Frame Number
 *  Parameters:     None
 *  Return Value:   Frame Number
 */
U32 USBD_GetFrame(void)
{
    return 0;
}

/*
 *  Get USB Last Error Code
 *  Parameters:   None
 *  Return Value:   Error Code
 */
U32 USBD_GetError(void)
{
    return 0;
}


/*
 *  USB Device Interrupt Service Routine
 */
void USBD_Handler(void)
{
    // uint32_t sts, val, num, i;
    // sts = USB_OTG_FS->INT_STAT;
    // USB_OTG_FS->INT_STAT = sts;
//     /* Device Status Interrupt (Reset, Connect change, Suspend/Resume) */
//     if (sts & USBHSD_INTSTAT_DEV_INT_MASK) {
//         val = USBHSD->DEVCMDSTAT;
//         /* reset interrupt */
//         if (val & USBHSD_DEVCMDSTAT_DRES_C_MASK) {
//             USBHSD->DEVCMDSTAT |= USBHSD_DEVCMDSTAT_DRES_C_MASK;
//             USBD_Reset();
//             usbd_reset_core();
// #ifdef __RTX
//             if (USBD_RTX_DevTask) {
//                 isr_evt_set(USBD_EVT_RESET, USBD_RTX_DevTask);
//             }
// #else
//             if (USBD_P_Reset_Event) {
//                 USBD_P_Reset_Event();
//             }
// #endif
//         }

//         /* connect interrupt */
//         if (val & USBHSD_DEVCMDSTAT_DCON_C_MASK) {
//             USBHSD->DEVCMDSTAT |= USBHSD_DEVCMDSTAT_DCON_C_MASK;
// #ifdef __RTX
//             if (USBD_RTX_DevTask) {
//                 if (val & USBHSD_DEVCMDSTAT_DCON_MASK) {
//                     isr_evt_set(USBD_EVT_POWER_ON,  USBD_RTX_DevTask);
//                 } else {
//                     isr_evt_set(USBD_EVT_POWER_OFF, USBD_RTX_DevTask);
//                 }
//             }
// #else
//             if (USBD_P_Power_Event) {
//                 USBD_P_Power_Event((val & USBHSD_DEVCMDSTAT_DCON_MASK) >> USBHSD_DEVCMDSTAT_DCON_SHIFT);
//             }
// #endif
//         }

//         /* suspend/resume interrupt */
//         if (val & USBHSD_DEVCMDSTAT_DSUS_C_MASK) {
//             USBHSD->DEVCMDSTAT |= USBHSD_DEVCMDSTAT_DSUS_C_MASK;
//             /* suspend interrupt */
//             if (val & USBHSD_DEVCMDSTAT_DSUS_MASK) {
//                 USBD_Suspend();
// #ifdef __RTX
//                 if (USBD_RTX_DevTask) {
//                     isr_evt_set(USBD_EVT_SUSPEND, USBD_RTX_DevTask);
//                 }
// #else
//                 if (USBD_P_Suspend_Event) {
//                     USBD_P_Suspend_Event();
//                 }
// #endif
//             }
//             /* resume interrupt */
//             else {
// #ifdef __RTX

//                 if (USBD_RTX_DevTask) {
//                     isr_evt_set(USBD_EVT_RESUME,  USBD_RTX_DevTask);
//                 }
// #else
//                 if (USBD_P_Resume_Event) {
//                     USBD_P_Resume_Event();
//                 }
// #endif
//             }
//         }
//     }

//     /* Start of Frame */
//     if (sts & USBHSD_INTSTAT_FRAME_INT_MASK) {
// #ifdef __RTX
//         if (USBD_RTX_DevTask) {
//             isr_evt_set(USBD_EVT_SOF, USBD_RTX_DevTask);
//         }
// #else
//         if (USBD_P_SOF_Event) {
//             USBD_P_SOF_Event();
//         }
// #endif
//     }

//     /* EndPoint Interrupt */
//     if (sts & USBHSD_INTEN_EP_INT_EN_MASK) {
//         const uint32_t endpoint_count = ((USBD_EP_NUM + 1) * 2);

//         for (i = 0; i < endpoint_count; i++) {
//             // Iterate through endpoints in the reverse order so IN endpoints
//             // get processed before OUT endpoints if they are both pending.
//             num = endpoint_count - i - 1;
//             if (sts & (1UL << num)) {
//                 /* Setup */
//                 if ((num == 0) && !s_read_ctrl_out_next && (USBHSD->DEVCMDSTAT & USBHSD_DEVCMDSTAT_SETUP_MASK)) {
// #ifdef __RTX
//                     if (USBD_RTX_EPTask[num / 2]) {
//                         isr_evt_set(USBD_EVT_SETUP, USBD_RTX_EPTask[num / 2]);
//                     }
// #else
//                     if (USBD_P_EP[num / 2]) {
//                         USBD_P_EP[num / 2](USBD_EVT_SETUP);
//                     }
// #endif
//                 }

//                 /* OUT */
//                 else if ((num % 2) == 0) {
// #ifdef __RTX
//                     if (USBD_RTX_EPTask[num / 2]) {
//                         isr_evt_set(USBD_EVT_OUT, USBD_RTX_EPTask[num / 2]);
//                     }
// #else
//                     if (USBD_P_EP[num / 2]) {
//                         USBD_P_EP[num / 2](USBD_EVT_OUT);
//                     }

// #endif
//                 }

//                 /* IN */
//                 else {
// #ifdef __RTX
//                     if (USBD_RTX_EPTask[num / 2]) {
//                         isr_evt_set(USBD_EVT_IN,  USBD_RTX_EPTask[num / 2]);
//                     }
// #else
//                     if (USBD_P_EP[num / 2]) {
//                         USBD_P_EP[num / 2](USBD_EVT_IN);
//                     }
// #endif
//                 }
//             }
//         }
//     }

    // end
    NVIC_EnableIRQ(USB_FS_IRQn);
}

void OTG_FS_IRQHandler(void)
{
    NVIC_DisableIRQ(USB_FS_IRQn);
    USBD_SignalHandler();
}
