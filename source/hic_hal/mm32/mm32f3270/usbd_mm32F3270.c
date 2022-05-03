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


typedef struct __BUF_DESC {
    uint8_t    stat;
    uint8_t    reserved;
    uint16_t   bc;
    uint32_t   buf_addr;
} BUF_DESC;

BUF_DESC __ALIGNED(512) BD[(USBD_EP_NUM + 1) * 2 * 2];
uint8_t EPBuf[(USBD_EP_NUM + 1) * 2 * 2][64];
uint8_t OutEpSize[USBD_EP_NUM + 1];
uint8_t StatQueue[(USBD_EP_NUM + 1) * 2 * 2 + 1];
uint32_t StatQueueHead = 0;
uint32_t StatQueueTail = 0;
uint32_t LastIstat = 0;
uint8_t UsbSuspended = 0;
uint8_t Ep0ZlpOut = 0;

uint32_t Data1  = 0x55555555;

#define BD_OWN_MASK        0x80
#define BD_DATA01_MASK     0x40
#define BD_KEEP_MASK       0x20
#define BD_NINC_MASK       0x10
#define BD_DTS_MASK        0x08
#define BD_STALL_MASK      0x04

#define TX    1
#define RX    0
#define ODD   0
#define EVEN  1
#define IDX(Ep, dir, Ev_Odd) ((((Ep & 0x0F) * 4) + (2 * dir) + (1 *  Ev_Odd)))

#define SETUP_TOKEN    0x0D
#define IN_TOKEN       0x09
#define OUT_TOKEN      0x01
#define TOK_PID(idx)   ((BD[idx].stat >> 2) & 0x0F)

inline static void protected_and(uint32_t *addr, uint32_t val)
{
    cortex_int_state_t state;
    state = cortex_int_get_and_disable();
    *addr = *addr & val;
    cortex_int_restore(state);
}
inline static void protected_or(uint32_t *addr, uint32_t val)
{
    cortex_int_state_t state;
    state = cortex_int_get_and_disable();
    *addr = *addr | val;
    cortex_int_restore(state);
}
inline static void protected_xor(uint32_t *addr, uint32_t val)
{
    cortex_int_state_t state;
    state = cortex_int_get_and_disable();
    *addr = *addr ^ val;
    cortex_int_restore(state);
}

inline static void stat_enque(uint32_t stat)
{
    cortex_int_state_t state;
    state = cortex_int_get_and_disable();
    StatQueue[StatQueueTail] = stat;
    StatQueueTail = (StatQueueTail + 1) % sizeof(StatQueue);
    cortex_int_restore(state);
}

inline static uint32_t stat_deque()
{
    cortex_int_state_t state;
    uint32_t stat;
    state = cortex_int_get_and_disable();
    stat = StatQueue[StatQueueHead];
    StatQueueHead = (StatQueueHead + 1) % sizeof(StatQueue);
    cortex_int_restore(state);

    return stat;
}

inline static uint32_t stat_is_empty()
{
    cortex_int_state_t state;
    uint32_t empty;
    state = cortex_int_get_and_disable();
    empty = StatQueueHead == StatQueueTail;
    cortex_int_restore(state);
    return empty;
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
//	USB_Clock_Config();
//	
//    USB_OTG_FS->INT_STAT	= 0xFFU;
//    USB_OTG_FS->INT_ENB	   &= ~(0xFFU);
//    USB_OTG_FS->ADDR 		= (0U);
//    USB_OTG_FS->CTL 		= 0x00U;
//	USB_OTG_FS->BDT_PAGE_01 = ((uint32_t)BDT_BASE >> 8U ) & 0xFFU;
//    USB_OTG_FS->BDT_PAGE_02 = ((uint32_t)BDT_BASE >> 16U) & 0xFFU;
//    USB_OTG_FS->BDT_PAGE_03 = ((uint32_t)BDT_BASE >> 24U) & 0xFFU;
//	
//	USBD_IntrEna();
//    USBD_Reset();
	
	OutEpSize[0] = USBD_MAX_PACKET0;

    USB_Clock_Config();

    USBD_IntrEna();

    USB_OTG_FS->BDT_PAGE_01 = (uint8_t)((uint32_t) BD >> 8);
    USB_OTG_FS->BDT_PAGE_02 = (uint8_t)((uint32_t) BD >> 16);
    USB_OTG_FS->BDT_PAGE_03 = (uint8_t)((uint32_t) BD >> 24);
    USB_OTG_FS->INT_STAT    = 0xFF;                 /* clear interrupt flags              */
    /* enable interrupts                                                        */
    USB_OTG_FS->INT_ENB = USB_INTEN_USBRSTEN_MASK | USB_INTEN_TOKDNEEN_MASK | USB_INTEN_SLEEPEN_MASK  |
#ifdef __RTX
            ((USBD_RTX_DevTask   != 0) ? USB_INTEN_SOFTOKEN_MASK : 0) |
            ((USBD_RTX_DevTask   != 0) ? USB_INTEN_ERROREN_MASK  : 0) ;
#else
            ((USBD_P_SOF_Event   != 0) ? USB_INTEN_SOFTOKEN_MASK : 0) |
            ((USBD_P_Error_Event != 0) ? USB_INTEN_ERROREN_MASK  : 0) ;
#endif
    USB_OTG_FS->CTL  = USB_USBCTRL_PDE_MASK;    /* pull dawn on D+ and D- */
	
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
    if (con) {
        USB_OTG_FS->CTL  |= USB_CTL_USBENSOFEN_MASK;            /* enable USB           */
    } else {
        USB_OTG_FS->CTL  &= ~USB_CTL_USBENSOFEN_MASK;           /* disable USB          */
    }
}

/*
 *  USB Device Reset Function
 *   Called automatically on USB Device Reset
 *    Return Value:    None
 */
void USBD_Reset(void)
{
    uint32_t i;

    NVIC_DisableIRQ(USB_FS_IRQn);

    for (i = 1; i < 16; i++) {
        USB_OTG_FS->EP_CTL[i].ENDPT = 0x00;
    }

    memset(StatQueue, 0, sizeof(StatQueue));
    StatQueueHead = 0;
    StatQueueTail = 0;
    LastIstat = 0;
    UsbSuspended = 0;
    Ep0ZlpOut = 0;

    /* EP0 control endpoint                                                     */
    BD[IDX(0, RX, ODD)].bc       = USBD_MAX_PACKET0;
    BD[IDX(0, RX, ODD)].buf_addr = (uint32_t) & (EPBuf[IDX(0, RX, ODD)][0]);
    BD[IDX(0, RX, ODD)].stat     = BD_OWN_MASK | BD_DTS_MASK | BD_DATA01_MASK;
    BD[IDX(0, RX, EVEN)].stat     = 0;
    BD[IDX(0, TX, ODD)].buf_addr = (uint32_t) & (EPBuf[IDX(0, TX, ODD)][0]);
    BD[IDX(0, TX, EVEN)].buf_addr = 0;
    USB_OTG_FS->EP_CTL[0].ENDPT = USB_ENDPT_EPHSHK_MASK | /* enable ep handshaking  */
                              USB_ENDPT_EPTXEN_MASK | /* enable TX (IN) tran.   */
                              USB_ENDPT_EPRXEN_MASK;  /* enable RX (OUT) tran.  */
    Data1 = 0x55555555;
    USB_OTG_FS->CTL    |=  USB_CTL_ODDRST_MASK;
    USB_OTG_FS->INT_STAT   =  0xFF;                /* clear all interrupt status flags   */
    USB_OTG_FS->ERR_STAT =  0xFF;                /* clear all error flags              */
    USB_OTG_FS->ERR_ENB   =  0xFF;                /* enable error interrupt sources     */
    USB_OTG_FS->ADDR    =  0x00;                /* set default address                */

    NVIC_SetPriority(USB_FS_IRQn, 0x01);    /* set second highest priority        */
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
    USB_OTG_FS->INT_ENB |= USB_INTEN_RESUMEEN_MASK;
}

/*
 *  USB Device Resume Function
 *   Called automatically on USB Device Resume
 *    Return Value:    None
 */
void USBD_Resume(void)
{
    /* Performed by Hardware */
	USB_OTG_FS->INT_ENB &= ~USB_INTEN_RESUMEEN_MASK;
}

/*
 *  USB Device Remote Wakeup Function
 *   Called automatically on USB Device Remote Wakeup
 *    Return Value:    None
 */
void USBD_WakeUp(void)
{
    // no neeed
    uint32_t i = 50000;

    if (USBD_DeviceStatus & USB_GETSTATUS_REMOTE_WAKEUP) {
        USB_OTG_FS->CTL |=  USB_CTL_RESUME_MASK;

        while (i--) {
            __NOP();
        }

        USB_OTG_FS->CTL &= ~USB_CTL_RESUME_MASK;
    }
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
        // USB_OTG_FS->ADDR &= ~OTG_FS_ADDR_ADDR;
        // USB_OTG_FS->ADDR |= adr | 0x7FU;
        USB_OTG_FS->ADDR    = adr & 0x7F;
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
    uint32_t num, val;
    num  = pEPD->bEndpointAddress;
    val  = pEPD->wMaxPacketSize;

    if (!(pEPD->bEndpointAddress & 0x80)) {
        OutEpSize[num] = val;
    }

    USBD_ResetEP(num);
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
    if (EPNum & 0x80) {
        EPNum &= 0x0F;
        USB_OTG_FS->EP_CTL[EPNum].ENDPT |= USB_ENDPT_EPHSHK_MASK | /*en ep handshaking*/
                                       USB_ENDPT_EPTXEN_MASK;  /*en TX (IN) tran  */
    } else {
        USB_OTG_FS->EP_CTL[EPNum].ENDPT |= USB_ENDPT_EPHSHK_MASK | /*en ep handshaking*/
                                       USB_ENDPT_EPRXEN_MASK;  /*en RX (OUT) tran.*/
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
	if (EPNum & 0x80) {
        EPNum &= 0x0F;
        USB_OTG_FS->EP_CTL[EPNum].ENDPT &= ~(USB_ENDPT_EPHSHK_MASK |/*dis handshaking */
                                         USB_ENDPT_EPTXEN_MASK);/*dis TX(IN) tran */
    } else {
        USB_OTG_FS->EP_CTL[EPNum].ENDPT &= ~(USB_ENDPT_EPHSHK_MASK |/*dis handshaking */
                                         USB_ENDPT_EPRXEN_MASK);/*dis RX(OUT) tran*/
    }
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
	if (EPNum & 0x80) {
        EPNum &= 0x0F;
        protected_or(&Data1, (1 << ((EPNum * 2) + 1)));
        BD[IDX(EPNum, TX, ODD)].buf_addr = (uint32_t) & (EPBuf[IDX(EPNum, TX, ODD)][0]);
        BD[IDX(EPNum, TX, EVEN)].buf_addr = 0;
    } else {
        protected_and(&Data1, ~(1 << (EPNum * 2)));
        BD[IDX(EPNum, RX, ODD)].bc       = OutEpSize[EPNum];
        BD[IDX(EPNum, RX, ODD)].buf_addr = (uint32_t) & (EPBuf[IDX(EPNum, RX, ODD)][0]);
        BD[IDX(EPNum, RX, ODD)].stat     = BD_OWN_MASK | BD_DTS_MASK;
        BD[IDX(EPNum, RX, EVEN)].stat     = 0;
    }
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
	EPNum &= 0x0F;
    USB_OTG_FS->EP_CTL[EPNum].ENDPT |= USB_ENDPT_EPSTALL_MASK;
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
	USB_OTG_FS->EP_CTL[EPNum & 0x0F].ENDPT &= ~USB_ENDPT_EPSTALL_MASK;
    USBD_ResetEP(EPNum);
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
	
}

/*
 *  Read USB Device Endpoint Data
 *  Parameters:     EPNum: Device Endpoint Number
 *                      EPNum.0..3: Address
 *                      EPNum.7:    Dir
 *                      pData: Pointer to Data Buffer
 *  Return Value:   Number of bytes read
 */
uint32_t USBD_ReadEP(uint32_t EPNum, uint8_t *pData, uint32_t size)
{
    uint32_t n, sz, idx, setup = 0;
    idx = IDX(EPNum, RX, 0);
    sz  = BD[idx].bc;

    if ((EPNum == 0) && Ep0ZlpOut) {
        // This packet was a zero length data out packet. It has already
        // been processed by USB_OTG_FS_IRQHandler. Only toggle the DATAx bit
        // and return a size of 0.
        protected_xor(&Data1, (1 << (idx / 2)));
        return 0;
    }

    if ((EPNum == 0) && (TOK_PID(idx) == SETUP_TOKEN)) {
        setup = 1;
    }

    if (size < sz) {
        util_assert(0);
        sz = size;
    }

    for (n = 0; n < sz; n++) {
        pData[n] = EPBuf[idx][n];
    }

    BD[idx].bc = OutEpSize[EPNum];

    if ((Data1 >> (idx / 2) & 1) == ((BD[idx].stat >> 6) & 1)) {
        uint32_t xfer_size = (pData[7] << 8) | (pData[6] << 0);
        if (setup && (0 == xfer_size)) {     /* if no setup data stage,            */
            protected_and(&Data1, ~1);           /* set DATA0                          */
        } else {
            protected_xor(&Data1, (1 << (idx / 2)));
        }
    }

    if ((Data1 >> (idx / 2)) & 1) {
        BD[idx].stat  = BD_DTS_MASK | BD_DATA01_MASK;
        BD[idx].stat |= BD_OWN_MASK;
    } else {
        BD[idx].stat  = BD_DTS_MASK;
        BD[idx].stat |= BD_OWN_MASK;
    }

    return (sz);
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
{
    uint32_t idx, n;
    EPNum &= 0x0F;
    idx = IDX(EPNum, TX, 0);
    BD[idx].bc = cnt;

    for (n = 0; n < cnt; n++) {
        EPBuf[idx][n] = pData[n];
    }

    if ((Data1 >> (idx / 2)) & 1) {
        BD[idx].stat = BD_OWN_MASK | BD_DTS_MASK;
    } else {
        BD[idx].stat = BD_OWN_MASK | BD_DTS_MASK | BD_DATA01_MASK;
    }

    protected_xor(&Data1, (1 << (idx / 2)));
    return (cnt);
}

/*
 *  Get USB Device Last Frame Number
 *  Parameters:     None
 *  Return Value:   Frame Number
 */
U32 USBD_GetFrame(void)
{
	return ((USB_OTG_FS->FRM_NUML | (USB_OTG_FS->FRM_NUMH << 8) & 0x07FF));
}

#ifdef __RTX
U32 LastError;                          /* Last Error                         */

/*
 *  Get USB Device Last Error Code
 *    Parameters:      None
 *    Return Value:    Error Code
 */

U32 USBD_GetError(void)
{
    return (LastError);
}
#endif

void USBD_Handler(void)
{
    uint32_t istr, num, dir, ev_odd;
    cortex_int_state_t state;
    uint8_t suspended = 0;

    // Get ISTAT
    state = cortex_int_get_and_disable();
    istr = LastIstat;
    LastIstat = 0;
    suspended = UsbSuspended;
    UsbSuspended = 0;
    cortex_int_restore(state);


    /* reset interrupt                                                            */
    if (istr & USB_ISTAT_USBRST_MASK) {
        USBD_Reset();
        usbd_reset_core();
#ifdef __RTX

        if (USBD_RTX_DevTask) {
            isr_evt_set(USBD_EVT_RESET, USBD_RTX_DevTask);
        }

#else

        if (USBD_P_Reset_Event) {
            USBD_P_Reset_Event();
        }

#endif
    }

    /* suspend interrupt                                                          */
    if (istr & USB_ISTAT_SLEEP_MASK) {
        USBD_Suspend();
#ifdef __RTX

        if (USBD_RTX_DevTask) {
            isr_evt_set(USBD_EVT_SUSPEND, USBD_RTX_DevTask);
        }

#else

        if (USBD_P_Suspend_Event) {
            USBD_P_Suspend_Event();
        }

#endif
    }

    /* resume interrupt                                                           */
    if (istr & USB_ISTAT_RESUME_MASK) {
        USBD_Resume();
#ifdef __RTX

        if (USBD_RTX_DevTask) {
            isr_evt_set(USBD_EVT_RESUME, USBD_RTX_DevTask);
        }

#else

        if (USBD_P_Resume_Event) {
            USBD_P_Resume_Event();
        }

#endif
    }

    /* Start Of Frame                                                             */
    if (istr & USB_ISTAT_SOFTOK_MASK) {
#ifdef __RTX

        if (USBD_RTX_DevTask) {
            isr_evt_set(USBD_EVT_SOF, USBD_RTX_DevTask);
        }

#else

        if (USBD_P_SOF_Event) {
            USBD_P_SOF_Event();
        }

#endif
    }

    /* Error interrupt                                                            */
    if (istr == USB_ISTAT_ERROR_MASK) {
#ifdef __RTX
        LastError = USB_OTG_FS->ERR_STAT;

        if (USBD_RTX_DevTask) {
            isr_evt_set(USBD_EVT_ERROR, USBD_RTX_DevTask);
        }

#else

        if (USBD_P_Error_Event) {
            USBD_P_Error_Event(USB_OTG_FS->ERR_STAT);
        }

#endif
        USB_OTG_FS->ERR_STAT = 0xFF;
    }

    /* token interrupt                                                            */
    if (istr & USB_ISTAT_TOKDNE_MASK) {
        while (!stat_is_empty()) {
            uint32_t stat;

            stat = stat_deque();
            num    = (stat >> 4) & 0x0F;
            dir    = (stat >> 3) & 0x01;
            ev_odd = (stat >> 2) & 0x01;

            /* setup packet                                                               */
            if ((num == 0) && (TOK_PID((IDX(num, dir, ev_odd))) == SETUP_TOKEN)) {
                Data1 &= ~0x02;
                BD[IDX(0, TX, EVEN)].stat &= ~BD_OWN_MASK;
                BD[IDX(0, TX, ODD)].stat  &= ~BD_OWN_MASK;
                Ep0ZlpOut = 0;
#ifdef __RTX

                if (USBD_RTX_EPTask[num]) {
                    isr_evt_set(USBD_EVT_SETUP, USBD_RTX_EPTask[num]);
                }

#else

                if (USBD_P_EP[num]) {
                    USBD_P_EP[num](USBD_EVT_SETUP);
                }

#endif

            } else {
                /* OUT packet                                                                 */
                if (TOK_PID((IDX(num, dir, ev_odd))) == OUT_TOKEN) {
                    if (0 == num) {
                        Ep0ZlpOut = stat & (1 << 0);
                    }
#ifdef __RTX

                    if (USBD_RTX_EPTask[num]) {
                        isr_evt_set(USBD_EVT_OUT, USBD_RTX_EPTask[num]);
                    }

#else

                    if (USBD_P_EP[num]) {
                        USBD_P_EP[num](USBD_EVT_OUT);
                    }

#endif
                }

                /* IN packet                                                                  */
                if (TOK_PID((IDX(num, dir, ev_odd))) == IN_TOKEN) {
#ifdef __RTX

                    if (USBD_RTX_EPTask[num]) {
                        isr_evt_set(USBD_EVT_IN,  USBD_RTX_EPTask[num]);
                    }

#else

                    if (USBD_P_EP[num]) {
                        USBD_P_EP[num](USBD_EVT_IN);
                    }

#endif
                }
            }
        }
    }

    if (suspended) {
        USB_OTG_FS->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;
    }
}

void OTG_FS_IRQHandler(void)
{
    uint32_t istat, num, dir, ev_odd;
    uint32_t new_istat;
    uint8_t suspended = 0;

    new_istat = istat = USB_OTG_FS->INT_STAT;

    // Read all tokens
    if (istat & USB_ISTAT_TOKDNE_MASK) {
        while (istat & USB_ISTAT_TOKDNE_MASK) {
            uint8_t stat = USB_OTG_FS->STAT;
            num    = (stat >> 4) & 0x0F;
            dir    = (stat >> 3) & 0x01;
            ev_odd = (stat >> 2) & 0x01;

            // Consume all zero length OUT packets on endpoint 0 to prevent
            // a subsequent SETUP packet from being dropped
            if ((0 == num) && (RX == dir)) {
                uint32_t idx;
                idx = IDX(num, dir, ev_odd);
                if ((TOK_PID(idx) == OUT_TOKEN) && (BD[idx].bc == 0)) {
                    BD[idx].bc = OutEpSize[num];
                    if (BD[idx].stat & BD_DATA01_MASK) {
                        BD[idx].stat = BD_OWN_MASK | BD_DTS_MASK;
                    } else {
                        BD[idx].stat = BD_OWN_MASK | BD_DTS_MASK | BD_DATA01_MASK;
                    }
                    stat |= 1 << 0;
                }
            }

            stat_enque(stat);
            USB_OTG_FS->INT_STAT = USB_ISTAT_TOKDNE_MASK;

            // Check if USB is suspending before checking istat
            suspended = suspended || USB_OTG_FS->CTL & USB_CTL_TXSUSPENDTOKENBUSY_MASK;
            istat = USB_OTG_FS->INT_STAT;
        }
    }

    // Set global istat and suspended flags
    new_istat |= istat;
    protected_or(&LastIstat, new_istat);
    UsbSuspended |= suspended ? 1 : 0;
    USB_OTG_FS->INT_STAT = istat;

    USBD_SignalHandler();
}
