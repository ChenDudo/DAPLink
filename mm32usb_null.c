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
*      Purpose: Hardware Layer module for MM32F3270
*      Rev.:    V4.70
*---------------------------------------------------------------------------*/

/* Double Buffering is not supported                                         */

#include <rl_usb.h>
#include "mm32_device.h"
#include "hal_gpio.h"
#include "hal_rcc.h"
#include "hal_misc.h"
#include "reg_usb_otg_fs.h"
#include "IO_Config.h"
#include "cortex_m.h"
#include "string.h"

#define __NO_USB_LIB_C
#include "usb_config.c"


////////////////////////////////////////////////////////////////////////////////
//! Define Buffer Descripter Table
////////////////////////////////////////////////////////////////////////////////
 #if defined(__ICCARM__)
 #define USB_WEAK_VAR                    __attribute__((weak))
 #define USB_WEAK_FUN                    __attribute__((weak))
 _Pragma("diag_suppress=Pm120")
 #define USB_ALIGN_PRAGMA(x)             _Pragma(#x)
 _Pragma("diag_default=Pm120")
 #define USB_RAM_ADDRESS_ALIGNMENT(n)    USB_ALIGN_PRAGMA(data_alignment = n)
 _Pragma("diag_suppress=Pm120")
 #define USB_LINK_SECTION_PART(str)      _Pragma(#str)
 #define USB_LINK_DMA_INIT_DATA(sec)     USB_LINK_SECTION_PART(location = #sec)
 #define USB_LINK_USB_GLOBAL             _Pragma("location = \"m_usb_global\"")
 #define USB_LINK_USB_BDT                _Pragma("location = \"m_usb_bdt\"")
 #define USB_LINK_USB_GLOBAL_BSS         _Pragma("location = \".bss.m_usb_global\"")
 #define USB_LINK_USB_BDT_BSS            _Pragma("location = \".bss.m_usb_bdt\"")
 _Pragma("diag_default=Pm120")
 #define USB_LINK_DMA_NONINIT_DATA       _Pragma("location = \"m_usb_dma_noninit_data\"")
 #define USB_LINK_NONCACHE_NONINIT_DATA  _Pragma("location = \"NonCacheable\"")

 #elif defined(__CC_ARM)
 #define USB_WEAK_VAR                    __attribute__((weak))
 #define USB_WEAK_FUN                    __weak
 #define USB_RAM_ADDRESS_ALIGNMENT(n)    __attribute__((aligned(n)))
 #define USB_LINK_DMA_INIT_DATA(sec)     __attribute__((section(#sec)))
 #define USB_LINK_USB_GLOBAL             __attribute__((section("m_usb_global")))            __attribute__((zero_init))
 #define USB_LINK_USB_BDT                __attribute__((section("m_usb_bdt")))               __attribute__((zero_init))
 #define USB_LINK_USB_GLOBAL_BSS         __attribute__((section(".bss.m_usb_global")))       __attribute__((zero_init))
 #define USB_LINK_USB_BDT_BSS            __attribute__((section(".bss.m_usb_bdt")))          __attribute__((zero_init))
 #define USB_LINK_DMA_NONINIT_DATA       __attribute__((section("m_usb_dma_noninit_data")))  __attribute__((zero_init))
 #define USB_LINK_NONCACHE_NONINIT_DATA  __attribute__((section("NonCacheable")))            __attribute__((zero_init))

 #elif defined(__GNUC__)
 #define USB_WEAK_VAR                    __attribute__((weak))
 #define USB_WEAK_FUN                    __attribute__((weak))
 #define USB_RAM_ADDRESS_ALIGNMENT(n)    __attribute__((aligned(n)))
 #define USB_LINK_DMA_INIT_DATA(sec)     __attribute__((section(#sec)))
 #define USB_LINK_USB_GLOBAL             __attribute__((section("m_usb_global, \"aw\", %nobits @")))
 #define USB_LINK_USB_BDT                __attribute__((section("m_usb_bdt, \"aw\", %nobits @")))
 #define USB_LINK_USB_GLOBAL_BSS         __attribute__((section(".bss.m_usb_global, \"aw\", %nobits @")))
 #define USB_LINK_USB_BDT_BSS            __attribute__((section(".bss.m_usb_bdt, \"aw\", %nobits @")))
 #define USB_LINK_DMA_NONINIT_DATA       __attribute__((section("m_usb_dma_noninit_data, \"aw\", %nobits @")))
 #define USB_LINK_NONCACHE_NONINIT_DATA  __attribute__((section("NonCacheable, \"aw\", %nobits @")))

 #else
 #error The tool-chain is not supported.
 #endif

typedef enum _usb_khci_interrupt_type {
    kUSB_KhciInterruptReset = 0x01U,
    kUSB_KhciInterruptError = 0x02U,
    kUSB_KhciInterruptSofToken = 0x04U,
    kUSB_KhciInterruptTokenDone = 0x08U,
    kUSB_KhciInterruptSleep = 0x10U,
    kUSB_KhciInterruptResume = 0x20U,
    kUSB_KhciInterruptAttach = 0x40U,
    kUSB_KhciInterruptStall = 0x80U,
} usb_khci_interrupt_type_t;

/*! @brief How many endpoints are supported in the stack. */
#define USB_DEVICE_CONFIG_ENDPOINTS 	(6U)

/* USB  standard descriptor endpoint type */
#define USB_ENDPOINT_CONTROL        (0x00U)
#define USB_ENDPOINT_ISOCHRONOUS    (0x01U)
#define USB_ENDPOINT_BULK           (0x02U)
#define USB_ENDPOINT_INTERRUPT      (0x03U)

#define MIN_BUF_SIZE                	(64)
#define USB_ENDPOINT_NUMBER_MASK		(0x0FU)
#define USB_SETUP_PACKET_SIZE 			(8U)

#define USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK  (0x80U)
#define USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT (7U)
#define USB_IN_MASK                 	(0x80)
#define USB_OUT 						(0U)
#define USB_IN 							(1U)

#define USB_LOGICAL_EP_MASK         	(0x7f)
#define USB_CONTROL_ENDPOINT        	(0U)
#define USB_CONTROL_MAX_PACKET_SIZE 	(64U)

#define USB_KHCI_BDT_DEVICE_OUT_TOKEN   (0x01U)
#define USB_KHCI_BDT_DEVICE_IN_TOKEN    (0x09U)
#define USB_KHCI_BDT_DEVICE_SETUP_TOKEN (0x0DU)
#define USB_KHCI_BDT_OWN                (0x80U)

#define USB_KHCI_BDT_DATA01(x)          ((((uint32_t)(x)) & 0x01U) << 0x06U)
#define USB_KHCI_BDT_BC(x)              ((((uint32_t)(x)) & 0x3FFU) << 0x10U)
#define UBS_KHCI_BDT_KEEP               (0x20U)
#define UBS_KHCI_BDT_NINC               (0x10U)
#define USB_KHCI_BDT_DTS                (0x08U)
#define USB_KHCI_BDT_STALL              (0x04U)

#define USB_SHORT_TO_LITTLE_ENDIAN(n) (n)
#define USB_LONG_TO_LITTLE_ENDIAN(n) (n)
#define USB_SHORT_FROM_LITTLE_ENDIAN(n) (n)
#define USB_LONG_FROM_LITTLE_ENDIAN(n) (n)

/* Set up packet structure */
typedef struct _usb_setup_struct {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb_setup_struct_t;

/*! @brief Endpoint state structure */
typedef struct _usb_device_khci_endpoint_state_struct {
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
} usb_device_khci_endpoint_state_struct_t;

/*! @brief KHCI state structure */
typedef struct _usb_device_khci_state_struct {
    uint8_t* bdt;				/*!< BDT buffer address */
    USB_OTG_FS_TypeDef*  registerBase;
								/*!< The base address of the register */
    uint8_t setupPacketBuffer[USB_SETUP_PACKET_SIZE * 2]; 
								/*!< The setup request buffer */
    uint8_t* dmaAlignBuffer;	/*!< This buffer is used to fix the transferBuffer or transferLength does
                               not align to 4-bytes when the function USB_DeviceKhciRecv is called.
                               The macro USB_DEVICE_CONFIG_KHCI_DMA_ALIGN is used to enable or disable this feature.
                               If the feature is enabled, when the transferBuffer or transferLength does not align to
                               4-bytes,
                               the transferLength is not more than USB_DEVICE_CONFIG_KHCI_DMA_ALIGN_BUFFER_LENGTH, and
                               the flag isDmaAlignBufferInusing is zero, the dmaAlignBuffer is used to receive data
                               and the flag isDmaAlignBufferInusing is set to 1.
                               When the transfer is done, the received data, kept in dmaAlignBuffer, is copied
                               to the transferBuffer, and the flag isDmaAlignBufferInusing is cleared.
                                */
    usb_device_khci_endpoint_state_struct_t endpointState[USB_DEVICE_CONFIG_ENDPOINTS * 2]; 
								/*!< Endpoint state structures */
    uint8_t isDmaAlignBufferInusing;	/*!< The dmaAlignBuffer is used or not */
    uint8_t isResetting;		/*!< Is doing device reset or not */
    uint8_t controllerId;		/*!< Controller ID */
    uint8_t setupBufferIndex;	/*!< A valid setup buffer flag */
} usb_device_khci_state_struct_t;

/* Set BDT buffer address */
#define USB_KHCI_BDT_SET_ADDRESS(bdt_base, ep, direction, odd, address)         \
    *((volatile uint32_t *)((bdt_base & 0xfffffe00U) | \
    (((uint32_t)ep & 0x0fU) << 5U) | (((uint32_t)direction & 1U) << 4U) | \
    (((uint32_t)odd & 1U) << 3U)) + 1U) = address

/* Set BDT control fields*/
#define USB_KHCI_BDT_SET_CONTROL(bdt_base, ep, direction, odd, control)         \
    *(volatile uint32_t *)((bdt_base & 0xfffffe00U) | \
    (((uint32_t)ep & 0x0fU) << 5U) | (((uint32_t)direction & 1U) << 4U) | \
    (((uint32_t)odd & 1U) << 3U)) = control

/* Get BDT buffer address*/
#define USB_KHCI_BDT_GET_ADDRESS(bdt_base, ep, direction, odd)                  \
    (*((volatile uint32_t *)((bdt_base & 0xfffffe00U) | \
    (((uint32_t)ep & 0x0fU) << 5U) | (((uint32_t)direction & 1U) << 4U) | \
    (((uint32_t)odd & 1U) << 3U)) + 1U))

/* Get BDT control fields*/
#define USB_KHCI_BDT_GET_CONTROL(bdt_base, ep, direction, odd)                  \
    (*(volatile uint32_t *)((bdt_base & 0xfffffe00U) | \
    (((uint32_t)ep & 0x0fU) << 5U) | (((uint32_t)direction & 1U) << 4U) |\
    (((uint32_t)odd & 1U) << 3U)))

	
	
#define USB_DEVICE_CONFIG_KHCI_DMA_ALIGN_BUFFER_LENGTH 64U	
	
	
#define USB_GLOBAL		USB_LINK_USB_GLOBAL_BSS
#define USB_BDT			USB_LINK_USB_BDT_BSS

USB_BDT 	USB_RAM_ADDRESS_ALIGNMENT(512) 	static uint8_t s_UsbDeviceKhciBdtBuffer[512U];
USB_GLOBAL 	USB_RAM_ADDRESS_ALIGNMENT(4) 	static usb_device_khci_state_struct_t khciState;
USB_GLOBAL 	USB_RAM_ADDRESS_ALIGNMENT(4)	static uint32_t s_UsbDeviceKhciDmaAlignBuffer[((64U - 1U) >> 2U) + 1U];

#define BDT_BASE				(uint32_t)(uint8_t*)(s_UsbDeviceKhciBdtBuffer)

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
	/* Enable USB Device */
	USB_OTG_FS->CTL |= OTG_FS_CTL_USB_EN_SOF_EN;
}

////////////////////////////////////////////////////////////////////////////////
// @brief De-initialize the USB device KHCI instance.
// 
////////////////////////////////////////////////////////////////////////////////
static void USB_DeviceKhciDeinit(void)
{
	/* Clear all interrupt flags. */
    khciState.registerBase->INT_STAT = 0xFFU;
    /* Disable all interrupts. */
    khciState.registerBase->INT_ENB &= ~(0xFFU);
    /* Clear device address. */
    khciState.registerBase->ADDR = (0U);
    /* Clear USB_CTL register */
    khciState.registerBase->CTL = 0x00U;
}

static void USB_DeviceKhciCancel(uint8_t ep)
{
    uint8_t index = ((ep & USB_ENDPOINT_NUMBER_MASK) << 1U) | ((ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                    USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);

    /* Cancel the transfer and notify the up layer when the endpoint is busy. */
    if (khciState.endpointState[index].stateUnion.stateBitField.transferring) {
        khciState.endpointState[index].stateUnion.stateBitField.transferring = 0U;
    }
}

void USB_DeviceKhciEndpointDeinit(uint8_t ep)
{
    uint8_t endpoint = (ep & USB_ENDPOINT_NUMBER_MASK);
    uint8_t direction = (ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >> USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT;
    uint8_t index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;

    /* Cancel the transfer of the endpoint */
    USB_DeviceKhciCancel(ep);
    /* Disable the endpoint */
    khciState.registerBase->EP_CTL[endpoint] = 0x00U;
    /* Clear the max packet size */
    khciState.endpointState[index].stateUnion.stateBitField.maxPacketSize = 0U;
}

static void USB_DeviceKhciSetDefaultState(void)
{
	uint8_t interruptFlag;
	
	/* Clear the error state register */
    khciState.registerBase->ERR_STAT = 0xFFU;
	
    /* Setting this bit to 1U resets all the BDT ODD ping/pong fields to 0U, which then specifies the EVEN BDT bank. */
    khciState.registerBase->CTL |= USB_CTL_ODDRST_MASK;
	
    /* Clear the device address */
    khciState.registerBase->ADDR = 0U;
	
    /* Clear the endpoint state and disable the endpoint */
    for (uint8_t count = 0U; count < USB_DEVICE_CONFIG_ENDPOINTS; count++) {
        USB_KHCI_BDT_SET_CONTROL((uint32_t)khciState.bdt, count, USB_OUT, 0U, 0U);
        USB_KHCI_BDT_SET_CONTROL((uint32_t)khciState.bdt, count, USB_OUT, 1U, 0U);
        USB_KHCI_BDT_SET_CONTROL((uint32_t)khciState.bdt, count, USB_IN, 0U, 0U);
        USB_KHCI_BDT_SET_CONTROL((uint32_t)khciState.bdt, count, USB_IN, 1U, 0U);
        khciState.endpointState[((uint32_t)count << 1U) | USB_OUT].stateUnion.state = 0U;
        khciState.endpointState[((uint32_t)count << 1U) | USB_IN ].stateUnion.state = 0U;
        khciState.registerBase->EP_CTL[count] = 0x00U;
    }
	khciState.isDmaAlignBufferInusing = 0U;
	
	/* Clear the BDT odd reset flag */
    khciState.registerBase->CTL &= ~USB_CTL_ODDRST_MASK;
	
	/* Enable all error */
    khciState.registerBase->ERR_ENB = 0xFFU;
	
	 /* Enable reset, sof, token, stall interrupt */
    interruptFlag = kUSB_KhciInterruptReset | kUSB_KhciInterruptTokenDone;
	
	/* Write the interrupt enable register */
    khciState.registerBase->INT_ENB = interruptFlag;

	/* Clear reset flag */
    khciState.isResetting = 0U;

    khciState.registerBase->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;
}

//    USB_OTG_FS->CTL |= OTG_FS_CTL_USB_EN_SOF_EN;    /*USB device enable */
    
void USB_DeviceKhciInit(void)
{
	khciState.registerBase = (USB_OTG_FS_TypeDef*)USB_OTG_FS;

    khciState.dmaAlignBuffer = (uint8_t*)s_UsbDeviceKhciDmaAlignBuffer;

    /* Clear all interrupt flags. */
    khciState.registerBase->INT_STAT = 0xFFU;

    khciState.bdt = (uint8_t*)(s_UsbDeviceKhciBdtBuffer);

    /* Set BDT buffer address */
    khciState.registerBase->BDT_PAGE_01 = (uint8_t)((((uint32_t)khciState.bdt) >> 8U ) & 0xFFU);
    khciState.registerBase->BDT_PAGE_02 = (uint8_t)((((uint32_t)khciState.bdt) >> 16U) & 0xFFU);
    khciState.registerBase->BDT_PAGE_03 = (uint8_t)((((uint32_t)khciState.bdt) >> 24U) & 0xFFU);

    /* Set KHCI device state to default value. */
    USB_DeviceKhciSetDefaultState();
}

void USB_DeviceKhciEndpointTransfer(uint8_t endpoint, uint8_t direction, uint8_t* buffer, uint32_t length)
{
    uint32_t index = ((uint32_t)endpoint << 1U) | (uint32_t)direction;

    /* Flag the endpoint is busy. */
    khciState.endpointState[index].stateUnion.stateBitField.transferring = 1U;

    /* Add the data buffer address to the BDT. */
    USB_KHCI_BDT_SET_ADDRESS((uint32_t)khciState.bdt, endpoint, direction, khciState.endpointState[index].stateUnion.stateBitField.bdtOdd, (uint32_t)buffer);

    /* Change the BDT control field to start the transfer. */
    USB_KHCI_BDT_SET_CONTROL((uint32_t)khciState.bdt, endpoint, direction, khciState.endpointState[index].stateUnion.stateBitField.bdtOdd,
        USB_LONG_TO_LITTLE_ENDIAN(USB_KHCI_BDT_BC(length) | USB_KHCI_BDT_OWN | USB_KHCI_BDT_DTS | USB_KHCI_BDT_DATA01(khciState.endpointState[index].stateUnion.stateBitField.data0)));

    /* Clear the token busy state */
    khciState.registerBase->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;
}


static void USB_DeviceKhciPrimeNextSetup(void)
{
    /* Update the endpoint state */
    /* Save the buffer address used to receive the setup packet. */
    khciState.endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].transferBuffer = (uint8_t*)&khciState.setupPacketBuffer[0] + khciState.endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].stateUnion.stateBitField.bdtOdd * USB_SETUP_PACKET_SIZE;
	
    /* Clear the transferred length. */
    khciState.endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].transferDone = 0U;
    /* Save the data length expected to get from a host. */
    khciState.endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].transferLength = USB_SETUP_PACKET_SIZE;
    /* Save the data buffer DMA align flag. */
    khciState.endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].stateUnion.stateBitField.dmaAlign = 1U;
    /* Set the DATA0/1 to DATA0. */
    khciState.endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].stateUnion.stateBitField.data0 = 0U;

    USB_DeviceKhciEndpointTransfer(USB_CONTROL_ENDPOINT, USB_OUT, khciState.endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].transferBuffer, USB_SETUP_PACKET_SIZE);
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
void USB_DeviceKhciEndpointInit(USB_ENDPOINT_DESCRIPTOR *pEPD)
{	
    uint16_t maxPacketSize = pEPD->wMaxPacketSize;
    uint8_t endpoint = (pEPD->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
    uint8_t direction = (pEPD->bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >> USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT;
    uint8_t index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;
	uint8_t type = pEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK;
	
    /* Make the endpoint max packet size align with USB Specification 2.0. */
    if (USB_ENDPOINT_ISOCHRONOUS == type) {
        if (maxPacketSize > 1023U) maxPacketSize = 1023U;
    }
    else {
		if (maxPacketSize > 64U) maxPacketSize = 64U;
        /* Enable an endpoint to perform handshaking during a transaction to this endpoint. */
        khciState.registerBase->EP_CTL[endpoint] |= USB_ENDPT_EPHSHK_MASK;
    }
    /* Set the endpoint idle */
    khciState.endpointState[index].stateUnion.stateBitField.transferring = 0U;
    /* Save the max packet size of the endpoint */
    khciState.endpointState[index].stateUnion.stateBitField.maxPacketSize = maxPacketSize;
    /* Set the data toggle to DATA0 */
    khciState.endpointState[index].stateUnion.stateBitField.data0 = 0U;
    /* Clear the endpoint stalled state */
    khciState.endpointState[index].stateUnion.stateBitField.stalled = 0U;
    /* Set the ZLT field */
    khciState.endpointState[index].stateUnion.stateBitField.zlt = 1U;
    /* Enable the endpoint. */
    khciState.registerBase->EP_CTL[endpoint] |= (USB_IN == direction) ? USB_ENDPT_EPTXEN_MASK : USB_ENDPT_EPRXEN_MASK;

    /* Prime a transfer to receive next setup packet when the endpoint is control out endpoint. */
    if ((USB_CONTROL_ENDPOINT == endpoint) && (USB_OUT == direction)) {
        USB_DeviceKhciPrimeNextSetup();
    }
}

void USB_DeviceKhciRecv(uint8_t endpointAddress, uint8_t* buffer, uint32_t length)
{
    uint32_t index = ((endpointAddress & USB_ENDPOINT_NUMBER_MASK) << 1U) | USB_OUT;

    if ((0U == length) && (USB_CONTROL_ENDPOINT == (endpointAddress & USB_ENDPOINT_NUMBER_MASK))) {
        khciState.endpointState[index].stateUnion.stateBitField.transferring = 0U;
        USB_DeviceKhciPrimeNextSetup();
    }
    else {
        /* Save the tansfer information */
        if (0U == khciState.endpointState[index].stateUnion.stateBitField.transferring) {
            khciState.endpointState[index].transferDone = 0U;
            khciState.endpointState[index].transferBuffer = buffer;
            khciState.endpointState[index].transferLength = length;
        }
        khciState.endpointState[index].stateUnion.stateBitField.dmaAlign = 1U;

        /* Data length needs to less than max packet size in each call. */
        if (length > khciState.endpointState[index].stateUnion.stateBitField.maxPacketSize) {
            length = khciState.endpointState[index].stateUnion.stateBitField.maxPacketSize;
        }

        buffer = (uint8_t*)((uint32_t)buffer + (uint32_t)khciState.endpointState[index].transferDone);

        if ((khciState.dmaAlignBuffer) && (0U == khciState.isDmaAlignBufferInusing) &&
                (USB_DEVICE_CONFIG_KHCI_DMA_ALIGN_BUFFER_LENGTH >= length) &&
                ((length & 0x03U) || (((uint32_t)buffer) & 0x03U))) {
            khciState.endpointState[index].stateUnion.stateBitField.dmaAlign = 0U;
            buffer = khciState.dmaAlignBuffer;
            khciState.isDmaAlignBufferInusing = 1U;
        }

        /* Receive data when the device is not resetting. */
        if (0U == khciState.isResetting) {
            USB_DeviceKhciEndpointTransfer(endpointAddress & USB_ENDPOINT_NUMBER_MASK, USB_OUT, buffer, length);
        }
    }
}

void USB_DeviceKhciSend(uint8_t endpointAddress, uint8_t* buffer, uint32_t length)
{
    uint32_t index = ((endpointAddress & USB_ENDPOINT_NUMBER_MASK) << 1U) | USB_IN;

    /* Save the tansfer information */
    if (0U == khciState.endpointState[index].stateUnion.stateBitField.transferring) {
        khciState.endpointState[index].transferDone = 0U;
        khciState.endpointState[index].transferBuffer = buffer;
        khciState.endpointState[index].transferLength = length;
        khciState.endpointState[index].stateUnion.stateBitField.dmaAlign = 1U;
    }

    /* Data length needs to less than max packet size in each call. */
    if (length > khciState.endpointState[index].stateUnion.stateBitField.maxPacketSize) {
        length = khciState.endpointState[index].stateUnion.stateBitField.maxPacketSize;
    }

    /* Send data when the device is not resetting. */
    if (0U == khciState.isResetting) {
        USB_DeviceKhciEndpointTransfer(endpointAddress & USB_ENDPOINT_NUMBER_MASK, USB_IN,\
		(uint8_t*)((uint32_t)khciState.endpointState[index].transferBuffer + (uint32_t)khciState.endpointState[index].transferDone), length);
    }

    /* Prime a transfer to receive next setup packet if the dat length is zero in a control in endpoint. */
    if ((0U == khciState.endpointState[index].transferDone) && (0U == length) && (USB_CONTROL_ENDPOINT == (endpointAddress & USB_ENDPOINT_NUMBER_MASK))) {
        USB_DeviceKhciPrimeNextSetup();
    }
}

static void USB_DeviceKhciInterruptReset(void)
{
    /* Set KHCI reset flag */
    khciState.isResetting = 1U;

    /* Clear the reset interrupt */
    khciState.registerBase->INT_STAT = (kUSB_KhciInterruptReset);
}

void USB_DeviceKhciInterruptTokenDone(void)
{
    uint32_t control, length, remainingLength;
    uint8_t* bdtBuffer;
    uint8_t endpoint, direction, bdtOdd, isSetup;
    uint8_t index;
    uint8_t stateRegister = khciState.registerBase->STAT;

    /* Get the endpoint number to identify which one triggers the token done interrupt. */
    endpoint 	= (stateRegister & USB_STAT_ENDP_MASK) 	>> USB_STAT_ENDP_SHIFT;
    direction	= (stateRegister & USB_STAT_TX_MASK)	>> USB_STAT_TX_SHIFT;
    bdtOdd		= (stateRegister & USB_STAT_ODD_MASK) 	>> USB_STAT_ODD_SHIFT;
    /* Clear token done interrupt flag. */
    khciState.registerBase->INT_STAT = kUSB_KhciInterruptTokenDone;
    /* Get the Control field of the BDT element according to the endpoint number, the direction and finished BDT ODD. */
    control = USB_KHCI_BDT_GET_CONTROL((uint32_t)khciState.bdt, endpoint, direction, bdtOdd);
    /* Get the buffer field of the BDT element according to the endpoint number, the direction and finished BDT ODD. */
    bdtBuffer = (uint8_t*)USB_KHCI_BDT_GET_ADDRESS((uint32_t)khciState.bdt, endpoint, direction, bdtOdd);
    /* Get the transferred length. */
    length = ((USB_LONG_FROM_LITTLE_ENDIAN(control)) >> 16U) & 0x3FFU;
    isSetup = (USB_KHCI_BDT_DEVICE_SETUP_TOKEN == ((uint8_t)(((USB_LONG_FROM_LITTLE_ENDIAN(control)) >> 2U) & 0x0FU))) ?
              1U :
              0U;
    index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;

    if (0U == khciState.endpointState[index].stateUnion.stateBitField.transferring) {
        return;
    }

    if (isSetup) {
        khciState.setupBufferIndex = bdtOdd;
    }

    /* USB_IN, Send completed */
    if (direction == USB_IN) {
        /* The transferred length */
        khciState.endpointState[index].transferDone += length;
        /* Remaining length */
        remainingLength = khciState.endpointState[index].transferLength - khciState.endpointState[index].transferDone;
        /* Change the data toggle flag */
        khciState.endpointState[index].stateUnion.stateBitField.data0 ^= 1U;
        /* Change the BDT odd toggle flag */
        khciState.endpointState[index].stateUnion.stateBitField.bdtOdd ^= 1U;
        /* Whether the transfer is completed or not. */
        /* The transfer is completed when one of the following conditions meet:
         * 1. The remaining length is zero.
         * 2. The length of current transcation is less than the max packet size of the current pipe.*/
        if ((0U == remainingLength) || (khciState.endpointState[index].stateUnion.stateBitField.maxPacketSize > length)) {
            khciState.endpointState[index].stateUnion.stateBitField.transferring = 0U;

            /* Whether need to send ZLT when the pipe is control in pipe and the transferred length of current
             * transaction equals to max packet size.            */
            if ((length) && (!(length % khciState.endpointState[index].stateUnion.stateBitField.maxPacketSize))) {
                if (USB_CONTROL_ENDPOINT == endpoint) {
                    usb_setup_struct_t* setup_packet = (usb_setup_struct_t*)(&khciState.setupPacketBuffer[(USB_SETUP_PACKET_SIZE * khciState.setupBufferIndex)]);
                    /* Send the ZLT and terminate the token done interrupt service when the tranferred length in data phase is less than the host request. */
                    if (USB_SHORT_FROM_LITTLE_ENDIAN(setup_packet->wLength) > khciState.endpointState[index].transferLength) {
                        (void)USB_DeviceKhciEndpointTransfer(endpoint, USB_IN, (uint8_t*)NULL, 0U); return;
                    }
                }
                else if (khciState.endpointState[index].stateUnion.stateBitField.zlt) {
                    (void)USB_DeviceKhciEndpointTransfer(endpoint, USB_IN, (uint8_t*)NULL, 0U); return;
                }
            }
        }
        else {
            /* Send remaining data and terminate the token done interrupt service. */
            (void)USB_DeviceKhciSend(endpoint | (USB_IN << 0x07U), khciState.endpointState[index].transferBuffer, remainingLength);
            return;
        }
    }
    else {
        if (!((USB_CONTROL_ENDPOINT == endpoint) && (0U == length))) {
            if (0U == khciState.endpointState[index].stateUnion.stateBitField.dmaAlign) {
                uint8_t* buffer = (uint8_t*)USB_LONG_FROM_LITTLE_ENDIAN(USB_KHCI_BDT_GET_ADDRESS((uint32_t)khciState.bdt, endpoint, USB_OUT, khciState.endpointState[index].stateUnion.stateBitField.bdtOdd));
                uint8_t* transferBuffer =  khciState.endpointState[index].transferBuffer + khciState.endpointState[index].transferDone;
                if (buffer != transferBuffer) {
                    for (uint32_t i = 0U; i < length; i++) {
                        transferBuffer[i] = buffer[i];
                    }
                }
                khciState.isDmaAlignBufferInusing = 0U;
            }
            /* The transferred length */
            khciState.endpointState[index].transferDone += length;
            /* Remaining length */
            remainingLength = khciState.endpointState[index].transferLength - khciState.endpointState[index].transferDone;

            if ((USB_CONTROL_ENDPOINT == endpoint) && isSetup) {
                khciState.endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_OUT].stateUnion.stateBitField.data0 = 1U;
                khciState.endpointState[(USB_CONTROL_ENDPOINT << 1U) | USB_IN ].stateUnion.stateBitField.data0 = 1U;
            }
            else {
                khciState.endpointState[index].stateUnion.stateBitField.data0 ^= 1U;
            }
            khciState.endpointState[index].stateUnion.stateBitField.bdtOdd ^= 1U;
            if ((!khciState.endpointState[index].transferLength) || (!remainingLength) || (khciState.endpointState[index].stateUnion.stateBitField.maxPacketSize > length)) {
                khciState.endpointState[index].stateUnion.stateBitField.transferring = 0U;
            }
            else {
                /* Receive remaining data and terminate the token done interrupt service. */
                USB_DeviceKhciRecv((endpoint) | (USB_OUT << 0x07U), khciState.endpointState[index].transferBuffer, remainingLength);
                return;
            }
        }
    }

    khciState.registerBase->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;
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
    
	USB_DeviceKhciDeinit();
	USB_DeviceKhciInit();
	
	USBD_IntrEna();
    //USBD_Reset();
}

/*
 *  USB Device Connect Function
 *   Called by the User to Connect/Disconnect USB Device
 *    Parameters:      con:   Connect/Disconnect
 *    Return Value:    None
 */
void USBD_Connect(BOOL con)
{
    // not support
    // (con) ? (USB_OTG_FS->CTL |= OTG_FS_CTL_USB_EN_SOF_EN) : \
	// 	(USB_OTG_FS->CTL &= ~OTG_FS_CTL_USB_EN_SOF_EN);
}

/*
 *  USB Device Reset Function
 *   Called automatically on USB Device Reset
 *    Return Value:    None
 */
void USBD_Reset(void)
{
	USB_ENDPOINT_DESCRIPTOR *pEPD;
	
    NVIC_DisableIRQ(USB_FS_IRQn);
	
	USB_DeviceKhciInterruptReset();
	
	pEPD->bmAttributes = USB_ENDPOINT_TYPE_CONTROL;
	pEPD->wMaxPacketSize = USB_CONTROL_MAX_PACKET_SIZE;
	pEPD->bEndpointAddress = USB_CONTROL_ENDPOINT | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
	
    /* Initialize the control IN pipe */
	USB_DeviceKhciEndpointInit((USB_ENDPOINT_DESCRIPTOR *)pEPD);
	
	/* Initialize the control OUT pipe */
	pEPD->bEndpointAddress = USB_CONTROL_ENDPOINT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
	USB_DeviceKhciEndpointInit((USB_ENDPOINT_DESCRIPTOR *)pEPD);
	
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
	khciState.registerBase->CTL |= USB_CTL_RESUME_MASK;
	
	uint16_t i = 1000;
	while(i--){
		__ASM("nop");
	};
	
	khciState.registerBase->CTL &= ~USB_CTL_RESUME_MASK;
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
}

void USBD_ConfigEP(USB_ENDPOINT_DESCRIPTOR *pEPD)
{
    uint8_t epnum = (uint8_t)(pEPD->bEndpointAddress);
	USB_DeviceKhciEndpointInit((USB_ENDPOINT_DESCRIPTOR*)pEPD);

    USBD_ResetEP(epnum);
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
    if (direction){
        
    }
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
/* The function is used to de-initialize a specified endpoint.
 * Current transfer of the endpoint will be canceled and the specified endpoint will be disabled..
 * @param EPNum The endpoint address, Bit7, 0U - USB_OUT, 1U - USB_IN.
 */
	USB_DeviceKhciEndpointDeinit(EPNum);
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
    if (khciState.endpointState[index].stateUnion.stateBitField.transferring) {
        khciState.endpointState[index].stateUnion.stateBitField.transferring = 0U;
    }
    /* Set endpoint stall flag. */
    khciState.endpointState[index].stateUnion.stateBitField.stalled = 1U;
    /* Set endpoint stall in BDT. And then if the host send a IN/OUT tanscation, the device will response a STALL state.*/
    USB_KHCI_BDT_SET_CONTROL(BDT_BASE, endpoint, direction, khciState.endpointState[index].stateUnion.stateBitField.bdtOdd,\
                USB_LONG_TO_LITTLE_ENDIAN((uint32_t)(USB_KHCI_BDT_BC(khciState.endpointState[index].stateUnion.stateBitField.maxPacketSize) | \
                    USB_KHCI_BDT_DTS | USB_KHCI_BDT_STALL | USB_KHCI_BDT_OWN)));
    USB_OTG_FS->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;
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
    khciState.endpointState[index].stateUnion.stateBitField.stalled = 0U;
    /* Reset the endpoint data toggle to DATA0 */
    khciState.endpointState[index].stateUnion.stateBitField.data0 = 0U;

    /* Clear stall state in BDT */
    for (uint8_t i = 0U; i < 2U; i++) {
        USB_KHCI_BDT_SET_CONTROL(BDT_BASE, endpoint, direction, i,
            USB_LONG_TO_LITTLE_ENDIAN((uint32_t)(USB_KHCI_BDT_BC(khciState.endpointState[index].stateUnion.stateBitField.maxPacketSize) | USB_KHCI_BDT_DTS | USB_KHCI_BDT_DATA01(0U))));
    }

    /* Clear stall state in endpoint control register */
    USB_OTG_FS->EP_CTL[endpoint] &= ~USB_ENDPT_EPSTALL_MASK;

    if ((USB_CONTROL_ENDPOINT != endpoint)) {
        if (khciState.endpointState[index].stateUnion.stateBitField.transferring) {
            khciState.endpointState[index].stateUnion.stateBitField.transferring = 0U;
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
    dataptr = khciState.endpointState[index].transferBuffer;
    cnt = khciState.endpointState[index].transferLength;
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

	//memcpy(pData, buffer, cnt); //TODO

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
    uint32_t* buffer;

    endpoint = (EPNum & 0x0FU);
    direction = (EPNum & 0x80U) >> 7;
    index = ((uint8_t)((uint32_t)endpoint << 1U)) | (uint8_t)direction;

    /* only IN EP */
    if (USB_OUT == direction) {
		return 0;
    }
	
	buffer= (uint32_t*)khciState.endpointState[index].transferBuffer + (uint32_t)khciState.endpointState[index].transferDone;
	
    
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


void tokenhandle()
{
//USB_DeviceKhciInterruptTokenDone();
	uint32_t control;
    uint32_t length;
    uint32_t remainingLength;
    uint8_t* bdtBuffer;
	uint8_t endpoint;
    uint8_t direction;
    uint8_t bdtOdd;
    uint8_t isSetup;
    uint8_t index;
    uint8_t stateRegister = khciState.registerBase->STAT;
        
	/* Get the endpoint number to identify which one triggers the token done interrupt. */
    endpoint = (stateRegister & USB_STAT_ENDP_MASK) >> USB_STAT_ENDP_SHIFT;

    /* Get the direction of the endpoint number. */
    direction = (stateRegister & USB_STAT_TX_MASK) >> USB_STAT_TX_SHIFT;

    /* Get the finished BDT ODD. */
    bdtOdd = (stateRegister & USB_STAT_ODD_MASK) >> USB_STAT_ODD_SHIFT;

    /* Clear token done interrupt flag. */
    khciState.registerBase->INT_STAT = kUSB_KhciInterruptTokenDone;

    /* Get the Control field of the BDT element according to the endpoint number, the direction and finished BDT ODD. */
    control = USB_KHCI_BDT_GET_CONTROL((uint32_t)khciState.bdt, endpoint, direction, bdtOdd);

    /* Get the buffer field of the BDT element according to the endpoint number, the direction and finished BDT ODD. */
    bdtBuffer = (uint8_t*)USB_KHCI_BDT_GET_ADDRESS((uint32_t)khciState.bdt, endpoint, direction, bdtOdd);

    /* Get the transferred length. */
    length = ((USB_LONG_FROM_LITTLE_ENDIAN(control)) >> 16U) & 0x3FFU;

    /* Get the transferred length. */
    isSetup = (USB_KHCI_BDT_DEVICE_SETUP_TOKEN == ((uint8_t)(((USB_LONG_FROM_LITTLE_ENDIAN(control)) >> 2U) & 0x0FU))) ? 1U : 0U;

	if (0U == khciState.endpointState[index].stateUnion.stateBitField.transferring) {
        return;
    }

    if (isSetup) {
        khciState.setupBufferIndex = bdtOdd;
    }

	/* USB_IN, Send completed */
    if (USB_IN == direction) {
#ifdef __RTX
            if (USBD_RTX_EPTask[num])   isr_evt_set(USBD_EVT_IN,  USBD_RTX_EPTask[num]);
#else
            if (USBD_P_EP[endpoint])         USBD_P_EP[endpoint](USBD_EVT_IN);
#endif
        }
        if (USB_OUT == direction) {
            if (USB_CONTROL_ENDPOINT == endpoint){
#ifdef __RTX
                if (USBD_RTX_EPTask[num / 2])   isr_evt_set(USBD_EVT_OUT, USBD_RTX_EPTask[num / 2]);
#else
                if (USBD_P_EP[endpoint / 2])         USBD_P_EP[endpoint](USBD_EVT_SETUP);
#endif
			}
            else {
#ifdef __RTX
				if (USBD_RTX_EPTask[num])   isr_evt_set(USBD_EVT_OUT,  USBD_RTX_EPTask[num]);
#else
				if (USBD_P_EP[endpoint])         USBD_P_EP[endpoint](USBD_EVT_OUT);
#endif
			}
		}
}

		
void USBD_Handler(void)
{
    uint32_t status = USB_OTG_FS->INT_STAT;
    //USB_OTG_FS->INT_STAT = status;

    /* Error interrupt */
    if (status & kUSB_KhciInterruptError) {
        USB_OTG_FS->INT_STAT |= kUSB_KhciInterruptError;
#ifdef __RTX
        if (USBD_RTX_DevTask)       isr_evt_set(USBD_EVT_ERROR, USBD_RTX_DevTask);
#else
        if (USBD_P_Error_Event)     USBD_P_Error_Event(1);
#endif
    }
    
    /* Suspend interrupt */
    if (status & kUSB_KhciInterruptSleep) {
        USBD_Suspend();
        USB_OTG_FS->INT_STAT |= kUSB_KhciInterruptSleep;
        //USB_DeviceKhciInterruptSleep(khciState.);
#ifdef __RTX
        if (USBD_RTX_DevTask)       isr_evt_set(USBD_EVT_SUSPEND, USBD_RTX_DevTask);
#else
        if (USBD_P_Suspend_Event)   USBD_P_Suspend_Event();
#endif
    }

    /* Start of Frame interrupt */
    if (status & kUSB_KhciInterruptSofToken) {
        USB_OTG_FS->INT_STAT |= kUSB_KhciInterruptSofToken;
#ifdef __RTX
        if (USBD_RTX_DevTask)       isr_evt_set(USBD_EVT_SOF, USBD_RTX_DevTask);
#else
        if (USBD_P_SOF_Event)       USBD_P_SOF_Event();
#endif
    }

    /* Reset interrupt */
    if (status & kUSB_KhciInterruptReset) {
        USB_OTG_FS->INT_STAT = kUSB_KhciInterruptReset;
        USBD_Reset();
        usbd_reset_core();
#ifdef __RTX
        if (USBD_RTX_DevTask)       isr_evt_set(USBD_EVT_RESET, USBD_RTX_DevTask);
#else
        if (USBD_P_Reset_Event)     USBD_P_Reset_Event();
#endif
    }

    /* Token done interrupt */
    if (status & kUSB_KhciInterruptTokenDone) {
        tokenhandle();
	}

    NVIC_EnableIRQ(USB_FS_IRQn);
}

void OTG_FS_IRQHandler(void)
{
    NVIC_DisableIRQ(USB_FS_IRQn);
    USBD_SignalHandler();
}
