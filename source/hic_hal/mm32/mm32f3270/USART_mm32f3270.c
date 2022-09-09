/*
 * Copyright (c) 2013-2020 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define _USART_MM32F3270_C_

#include "IO_Config.h"
#include "DAP_config.h"
#include "circ_buf.h"
#include "gpio.h"
#include "hal_gpio.h"
#include "hal_misc.h"
#include "hal_rcc.h"
#include "hal_uart.h"
#include "mm32_device.h"
#include "uart.h"
#include "util.h"
#include <string.h>

#include "Driver_USART.h"
#include "USART_mm32f3270.h"

// USART BRR macro
#define USART_DIVIDER(_PCLK_, _BAUD_)           (((_PCLK_)*25)/(4*(_BAUD_)))
#define USART_DIVIDER_MANTISA(_PCLK_, _BAUD_)     (USART_DIVIDER((_PCLK_), (_BAUD_))/100)
#define USART_DIVIDER_FRACTION(_PCLK_, _BAUD_)  (((USART_DIVIDER((_PCLK_), (_BAUD_)) - (USART_DIVIDER_MANTISA((_PCLK_), (_BAUD_)) * 100)) * 16 + 50) / 100)
#define USART_BAUDRATE_DIVIDER(_PCLK_, _BAUD_)   ((USART_DIVIDER_MANTISA((_PCLK_), (_BAUD_)) << 4)|(USART_DIVIDER_FRACTION((_PCLK_), (_BAUD_)) & 0x0F))

#if defined(SWO_UART)

#define SWO_UART_IP UART6
#define SWO_UART_IRQn UART6_IRQn
#define SWO_UART_IRQn_Handler UART6_IRQHandler
#define SWO_UART_ENABLE() RCC_APB2PeriphClockCmd(RCC_APB2ENR_UART6, ENABLE)
#define SWO_UART_DISABLE() RCC_APB2PeriphClockCmd(RCC_APB2ENR_UART6, DISABLE)

#define ARM_USART_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */

 // USART6 Run-Time Information
static USART_INFO USART6_Info = { 0 };
static USART_TRANSFER_INFO USART6_TransferInfo = { 0 };

#define MX_USART6_RX_Pin        1
#define MX_USART6_RX_GPIOx      SWDO_PIN_PORT
#define MX_USART6_RX_GPIO_Pin   (1U << SWDO_PIN_Bit)
#define MX_USART6_RX_GPIO_AF    SWDO_AF
//#define MX_USART6_RX_GPIO_PuPd  0
#define MX_USART6_RX_AF_Pin     SWDO_PIN_Bit
#define MX_USART6_RX_GPIO_Speed 1


#ifdef MX_USART6_TX_Pin
static USART_PIN USART6_tx = { MX_USART6_TX_GPIOx, MX_USART6_TX_GPIO_Pin };
#endif
#ifdef MX_USART6_RX_Pin
static USART_PIN USART6_rx = { MX_USART6_RX_GPIOx, MX_USART6_RX_GPIO_Pin, MX_USART6_RX_GPIO_AF, MX_USART6_RX_AF_Pin, MX_USART6_RX_GPIO_Speed};
#endif
#ifdef MX_USART6_CK_Pin
static USART_PIN USART6_ck = { MX_USART6_CK_GPIOx, MX_USART6_CK_GPIO_Pin };
#endif
#ifdef MX_USART6_RTS_Pin
static USART_PIN USART6_rts = { MX_USART6_RTS_GPIOx, MX_USART6_RTS_GPIO_Pin };
#endif
#ifdef MX_USART6_CTS_Pin
static USART_PIN USART6_cts = { MX_USART6_CTS_GPIOx, MX_USART6_CTS_GPIO_Pin };
#endif

#ifdef MX_USART6_TX_DMA_Instance
static USART_DMA USART6_DMA_Tx = {
    MX_USART6_TX_DMA_Instance,
    MX_USART6_TX_DMA_Number,
    MX_USART6_TX_DMA_Channel,
    MX_USART6_TX_DMA_Priority };
#endif
#ifdef MX_USART6_RX_DMA_Instance
static USART_DMA USART6_DMA_Rx = {
    MX_USART6_RX_DMA_Instance,
    MX_USART6_RX_DMA_Number,
    MX_USART6_RX_DMA_Channel,
    MX_USART6_RX_DMA_Priority };
#endif

// USART6 Resources
static const USART_RESOURCES USART6_Resources = {
    {
        // Capabilities
        1, // supports UART (Asynchronous) mode
#ifdef MX_USART6_CK_Pin
        1, // supports Synchronous Master mode
#else
        0, // supports Synchronous Master mode
#endif
        0, // supports Synchronous Slave mode
        0, // supports UART Single-wire mode
        0, // supports UART IrDA mode
        0, // supports UART Smart Card mode
        0, // Smart Card Clock generator
#ifdef MX_USART6_RTS_Pin
        1, // RTS Flow Control available
#else
        0, // RTS Flow Control available
#endif
#ifdef MX_USART6_CTS_Pin
        1, // CTS Flow Control available
#else
        0, // CTS Flow Control available
#endif
        1, // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
        1, // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#ifdef MX_USART6_RTS_Pin
        1, // RTS Line: 0=not available, 1=available
#else
        0, // RTS Line: 0=not available, 1=available
#endif
#ifdef MX_USART6_CTS_Pin
        1, // CTS Line: 0=not available, 1=available
#else
        0, // CTS Line: 0=not available, 1=available
#endif
        0, // DTR Line: 0=not available, 1=available
        0, // DSR Line: 0=not available, 1=available
        0, // DCD Line: 0=not available, 1=available
        0, // RI Line: 0=not available, 1=available
#ifdef MX_USART6_CTS_Pin
        1, // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#else
        0, // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#endif
        0, // Signal DSR change event: \ref ARM_USART_EVENT_DSR
        0, // Signal DCD change event: \ref ARM_USART_EVENT_DCD
        0, // Signal RI change event: \ref ARM_USART_EVENT_RI
    },

    SWO_UART_IP,
    RCC_GetPCLK2Freq, // RTE_PCLK2,

    // PINS
    {
#ifdef MX_USART6_TX_Pin
        &USART6_tx,
#else
        NULL,
#endif
#ifdef MX_USART6_RX_Pin
        &USART6_rx,
#else
        NULL,
#endif
#ifdef MX_USART6_CK_Pin
        &USART6_ck,
#else
        NULL,
#endif
#ifdef MX_USART6_RTS_Pin
        &USART6_rts,
#else
        NULL,
#endif
#ifdef MX_USART6_CTS_Pin
        &USART6_cts,
#else
        NULL,
#endif
        // MX_USART6_REMAP_DEF,
        // MX_USART6_REMAP,
},

SWO_UART_IRQn,

#ifdef MX_USART6_TX_DMA_Instance
& USART6_DMA_Tx,
#else
NULL,
#endif
#ifdef MX_USART6_RX_DMA_Instance
& USART6_DMA_Rx,
#else
NULL,
#endif

&USART6_Info,
&USART6_TransferInfo };

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_USART_API_VERSION,
    ARM_USART_DRV_VERSION };

/* Driver Capabilities */
static const ARM_USART_CAPABILITIES DriverCapabilities = {
    1, /* supports UART (Asynchronous) mode */
    0, /* supports Synchronous Master mode */
    0, /* supports Synchronous Slave mode */
    0, /* supports UART Single-wire mode */
    0, /* supports UART IrDA mode */
    0, /* supports UART Smart Card mode */
    0, /* Smart Card Clock generator available */
    0, /* RTS Flow Control available */
    0, /* CTS Flow Control available */
    0, /* Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE */
    0, /* Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT */
    0, /* RTS Line: 0=not available, 1=available */
    0, /* CTS Line: 0=not available, 1=available */
    0, /* DTR Line: 0=not available, 1=available */
    0, /* DSR Line: 0=not available, 1=available */
    0, /* DCD Line: 0=not available, 1=available */
    0, /* RI Line: 0=not available, 1=available */
    0, /* Signal CTS change event: \ref ARM_USART_EVENT_CTS */
    0, /* Signal DSR change event: \ref ARM_USART_EVENT_DSR */
    0, /* Signal DCD change event: \ref ARM_USART_EVENT_DCD */
    0, /* Signal RI change event: \ref ARM_USART_EVENT_RI */
    0  /* Reserved (must be zero) */
};

//
//   Functions
//

// Function prototypes
static int32_t USART_Initialize (ARM_USART_SignalEvent_t cb_event, const USART_RESOURCES* usart);
static int32_t USART_Uninitialize (const USART_RESOURCES* usart);
static int32_t USART_PowerControl (ARM_POWER_STATE state, const USART_RESOURCES* usart);
static int32_t USART_Send (const void* data, uint32_t num, const USART_RESOURCES* usart);
static int32_t USART_Receive (void* data, uint32_t num, const USART_RESOURCES* usart);
static int32_t USART_Transfer (const void* data_out, void* data_in, uint32_t num, const USART_RESOURCES* usart);
static uint32_t USART_GetTxCount (const USART_RESOURCES* usart);
static uint32_t USART_GetRxCount (const USART_RESOURCES* usart);
static int32_t USART_Control (uint32_t control, uint32_t arg, const USART_RESOURCES* usart);
static ARM_USART_STATUS USART_GetStatus (const USART_RESOURCES* usart);
static int32_t USART_SetModemControl (ARM_USART_MODEM_CONTROL control, const USART_RESOURCES* usart);
static ARM_USART_MODEM_STATUS USART_GetModemStatus (const USART_RESOURCES* usart);

static ARM_DRIVER_VERSION ARM_USART_GetVersion (void)
{
    return DriverVersion;
}

static ARM_USART_CAPABILITIES USART_GetCapabilities (const USART_RESOURCES* usart)
{
    return DriverCapabilities;
}

static int32_t USART_Initialize (ARM_USART_SignalEvent_t cb_event, const USART_RESOURCES* usart)
{
    if (usart->info->flags & USART_FLAG_INITIALIZED) {
        // Driver is already initialized
        return ARM_DRIVER_OK;
    }

    // Initialize USART Run-time Resources
    usart->info->cb_event = cb_event;

    usart->info->status.tx_busy = 0U;
    usart->info->status.rx_busy = 0U;
    usart->info->status.tx_underflow = 0U;
    usart->info->status.rx_overflow = 0U;
    usart->info->status.rx_break = 0U;
    usart->info->status.rx_framing_error = 0U;
    usart->info->status.rx_parity_error = 0U;

    usart->info->mode = 0U;
    usart->xfer->send_active = 0U;

    // Clear transfer information
    memset (usart->xfer, 0, sizeof (USART_TRANSFER_INFO));

    // Setup pin remap
    // GPIO_AFConfigure(usart->io.afio);
    GPIO_PinAFConfig (usart->io.rx->port, usart->io.rx->pin, usart->io.rx->af);

    RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOC, ENABLE);
    RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOD, ENABLE);

    // Enable TX pin port clock
    if (usart->io.tx) {
        // GPIO_PortClock (usart->io.tx->port, true);
    }

    // Enable RX pin port clock
    if (usart->io.rx) {
        RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOB, ENABLE);
    }

    usart->info->flags = USART_FLAG_INITIALIZED;

    return ARM_DRIVER_OK;
}

static int32_t USART_Uninitialize (const USART_RESOURCES* usart)
{
    if ((usart->info->flags & USART_FLAG_POWERED) != 0U) {
        // If peripheral is powered, power off the peripheral
        (void)USART_PowerControl (ARM_POWER_OFF, usart);
    }

    // Unconfigure USART pins
    if (usart->io.tx)
        // GPIO_PinConfigure(usart->io.tx->port,  usart->io.tx->pin,  GPIO_IN_ANALOG, GPIO_MODE_INPUT);
        if (usart->io.rx) {
            GPIO_InitTypeDef GPIO_InitStructure = { 0 };
            GPIO_InitStructure.GPIO_Pin = usart->io.rx->pin;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_FLOATING;
            GPIO_Init (usart->io.rx->port, &GPIO_InitStructure);
        }
    if (usart->io.ck) { ; }
    // GPIO_PinConfigure(usart->io.ck->port,  usart->io.ck->pin,  GPIO_IN_ANALOG, GPIO_MODE_INPUT);
    if (usart->io.rts) { ; }
    // GPIO_PinConfigure(usart->io.rts->port, usart->io.rts->pin, GPIO_IN_ANALOG, GPIO_MODE_INPUT);
    if (usart->io.cts) { ; }
    // GPIO_PinConfigure(usart->io.cts->port, usart->io.cts->pin, GPIO_IN_ANALOG, GPIO_MODE_INPUT);

    // Unconfigure pin remap
    GPIO_PinAFConfig (usart->io.rx->port, usart->io.rx->pin, 0);

    // Reset USART status flags
    usart->info->flags = 0U;

    return ARM_DRIVER_OK;
}

static int32_t USART_PowerControl (ARM_POWER_STATE state, const USART_RESOURCES* usart)
{
    if ((usart->info->flags & USART_FLAG_INITIALIZED) == 0U) {
        return ARM_DRIVER_ERROR;
    }
    if ((state != ARM_POWER_OFF) &&
        (state != ARM_POWER_FULL) &&
        (state != ARM_POWER_LOW)) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    switch (state) {
        case ARM_POWER_OFF:
            RCC->APB2RSTR |= RCC_APB2RSTR_UART6;
            NVIC_DisableIRQ (usart->irq_num);

            // Disable USART clock
            if (usart->reg == UART6)
                RCC->APB2ENR &= ~RCC_APB2ENR_UART6;

            // Clear Status flags
            usart->info->status.tx_busy = 0U;
            usart->info->status.rx_busy = 0U;
            usart->info->status.tx_underflow = 0U;
            usart->info->status.rx_overflow = 0U;
            usart->info->status.rx_break = 0U;
            usart->info->status.rx_framing_error = 0U;
            usart->info->status.rx_parity_error = 0U;
            usart->xfer->send_active = 0U;

            // Clear transfer information
            memset ((void*)usart->xfer, 0, sizeof (USART_TRANSFER_INFO));

            usart->info->flags &= ~USART_FLAG_POWERED;
            break;

        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_POWER_FULL:
            if ((usart->info->flags & USART_FLAG_INITIALIZED) == 0U) {
                return ARM_DRIVER_ERROR;
            }
            if ((usart->info->flags & USART_FLAG_POWERED) != 0U) {
                return ARM_DRIVER_OK;
            }

            // Clear Status flags
            usart->info->status.tx_busy = 0U;
            usart->info->status.rx_busy = 0U;
            usart->info->status.tx_underflow = 0U;
            usart->info->status.rx_overflow = 0U;
            usart->info->status.rx_break = 0U;
            usart->info->status.rx_framing_error = 0U;
            usart->info->status.rx_parity_error = 0U;

            usart->xfer->send_active = 0U;
            usart->xfer->def_val = 0U;
            usart->xfer->sync_mode = 0U;
            usart->xfer->break_flag = 0U;
            usart->info->mode = 0U;
            usart->info->flow_control = 0U;

            usart->info->flags = USART_FLAG_POWERED | USART_FLAG_INITIALIZED;

            // Enable USART clock
            if (usart->reg == UART6)
                RCC->APB2ENR |= RCC_APB2ENR_UART6;

            // Clear and Enable USART IRQ
            NVIC_ClearPendingIRQ (usart->irq_num);
            NVIC_EnableIRQ (usart->irq_num);

            // Peripheral reset
            RCC->APB2RSTR |= RCC_APB2RSTR_UART6;
            break;
    }
    return ARM_DRIVER_OK;
}

static int32_t USART_Send (const void* data, uint32_t num, const USART_RESOURCES* usart)
{
    int32_t stat;

    if ((data == NULL) || (num == 0U)) {
        // Invalid parameters
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0U) {
        // USART is not configured (mode not selected)
        return ARM_DRIVER_ERROR;
    }

    if (usart->xfer->send_active != 0U) {
        // Send is not completed yet
        return ARM_DRIVER_ERROR_BUSY;
    }

    // Set Send active flag
    usart->xfer->send_active = 1U;

    // Save transmit buffer info
    usart->xfer->tx_buf = (uint8_t*)data;
    usart->xfer->tx_num = num;
    usart->xfer->tx_cnt = 0U;

    // Synchronous mode
    if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
        if (usart->xfer->sync_mode == 0U) {
            usart->xfer->sync_mode = USART_SYNC_MODE_TX;
            // Start dummy reads
            // stat = UART_Receive (&usart->xfer->dump_val, num, usart);
            if (stat == ARM_DRIVER_ERROR_BUSY) { return ARM_DRIVER_ERROR_BUSY; }

        }
    }

#ifdef __USART_DMA_TX
    // DMA mode
#endif
    // Interrupt mode
    {
        // TXE interrupt enable
        usart->reg->IER |= UART_IER_TX; //????chend
    }

    return ARM_DRIVER_OK;
}

static int32_t USART_Receive (void* data, uint32_t num, const USART_RESOURCES* usart)
{
    int32_t stat;

    if ((data == NULL) || (num == 0U)) {
        // Invalid parameters
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0U) {
        // USART is not configured (mode not selected)
        return ARM_DRIVER_ERROR;
    }

    // Check if receiver is busy
    if (usart->info->status.rx_busy == 1U) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    // Disable RXNE Interrupt
    usart->reg->IER &= ~UART_IER_RX;

    // Save number of data to be received
    usart->xfer->rx_num = num;

    // Clear RX statuses
    usart->info->status.rx_break = 0U;
    usart->info->status.rx_framing_error = 0U;
    usart->info->status.rx_overflow = 0U;
    usart->info->status.rx_parity_error = 0U;

    // Save receive buffer info
    usart->xfer->rx_buf = (uint8_t*)data;
    usart->xfer->rx_cnt = 0U;

    // Set RX busy flag
    usart->info->status.rx_busy = 1U;

#ifdef __USART_DMA_RX
    // Synchronous mode
    // DMA mode
#endif
    {
        // Enable RXNE, IDLE and TE interrupts
        usart->reg->IER |= (UART_IER_RXIDLE | UART_IER_RX | UART_IER_TX);
    }

    // Synchronous mode
    if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
        if (usart->xfer->sync_mode == 0U) {
            usart->xfer->sync_mode = USART_SYNC_MODE_RX;
            // Send dummy data
            stat = USART_Send (&usart->xfer->def_val, num, usart);
            if (stat == ARM_DRIVER_ERROR_BUSY) { return ARM_DRIVER_ERROR_BUSY; }
        }
    }

    return ARM_DRIVER_OK;
}

static int32_t USART_Transfer (const void* data_out, void* data_in, uint32_t num, const USART_RESOURCES* usart)
{
    int32_t status;

    if ((data_out == NULL) || (data_in == NULL) || (num == 0U)) {
        // Invalid parameters
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0U) {
        // USART is not configured
        return ARM_DRIVER_ERROR;
    }

    if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {

        // Set xfer mode
        usart->xfer->sync_mode = USART_SYNC_MODE_TX_RX;

        // Receive
        status = USART_Receive (data_in, num, usart);
        if (status != ARM_DRIVER_OK) { return status; }

        // Send
        status = USART_Send (data_out, num, usart);
        if (status != ARM_DRIVER_OK) { return status; }

    }
    else {
        // Only in synchronous mode
        return ARM_DRIVER_ERROR;
    }
    return ARM_DRIVER_OK;
}

static uint32_t USART_GetTxCount (const USART_RESOURCES* usart)
{
#ifdef __USART_DMA_TX

#endif
    {
        return usart->xfer->tx_cnt;
    }
}

static uint32_t USART_GetRxCount (const USART_RESOURCES* usart)
{
#ifdef __USART_DMA_RX

#endif
    {
        return usart->xfer->rx_cnt;
    }
}

















static int32_t USART_Control (uint32_t control, uint32_t arg, const USART_RESOURCES* usart)
{
    uint32_t mode, flow_control, i;
    GPIO_InitTypeDef GPIO_InitStruct;

    if ((usart->info->flags & USART_FLAG_POWERED) == 0U) {
        // USART not powered
        return ARM_DRIVER_ERROR;
    }

    switch (control & ARM_USART_CONTROL_Msk) {
        // Control break
        case ARM_USART_CONTROL_BREAK:
            if (arg) {
                if (usart->xfer->send_active != 0U) { return ARM_DRIVER_ERROR_BUSY; }

                // Set Send active and Break flag
                usart->xfer->send_active = 1U;
                usart->xfer->break_flag = 1U;

                // Enable TX interrupt and send break
                usart->reg->IER |= UART_IER_TX | UART_IER_TXBRK;
            }
            else {
                if (usart->xfer->break_flag) {
                    // Disable TX interrupt
                    usart->reg->IER &= ~UART_IER_TX;

                    // Clear break and Send Active flag
                    usart->xfer->break_flag = 0U;
                    usart->xfer->send_active = 0U;
                }
            }
            return ARM_DRIVER_OK;

            // Abort Send
        case ARM_USART_ABORT_SEND:
            // Disable TX and TC interrupt
            usart->reg->IER &= ~(UART_IER_TX | UART_IER_TXC);

            if (usart->xfer->send_active != 0U) {
                if (usart->dma_tx != NULL) {
                    // In DMA mode - disable DMA channel
                    // DMA disable transmitter
                    //usart->reg->CR3 &= ~USART_CR3_DMAT;
                    // Abort TX DMA transfer
                    //HAL_DMA_Abort (usart->dma_tx->hdma);
                }

                // Wait for loaded frame to be sent
                for (i = 0U; i < 100000U; i++) {
                    if ((usart->reg->CSR & UART_CSR_TXEPT) != 0U) {
                        break;
                    }
                }
            }

            // Clear break flag
            usart->xfer->break_flag = 0U;

            // Clear Send active flag
            usart->xfer->send_active = 0U;
            return ARM_DRIVER_OK;

            // Abort receive
        case ARM_USART_ABORT_RECEIVE:
            // Disable RX interrupt
            usart->reg->IER &= ~UART_IER_RX;

            // If DMA mode - disable DMA channel
            if ((usart->dma_rx != NULL) && (usart->info->status.rx_busy != 0)) {
                // DMA disable Receiver
                //usart->reg->CR3 &= ~USART_CR3_DMAR;
                // Abort RX DMA transfer
                //HAL_DMA_Abort (usart->dma_rx->hdma);
            }

            // Clear RX busy status
            usart->info->status.rx_busy = 0U;

            if ((usart->xfer->sync_mode == USART_SYNC_MODE_RX) && (usart->xfer->send_active != 0U)) {
                return USART_Control (ARM_USART_ABORT_SEND, 0U, usart);
            }

            return ARM_DRIVER_OK;

            // Abort transfer
        case ARM_USART_ABORT_TRANSFER:
            // Disable TX, TC and RX interrupt
            usart->reg->IER &= ~(UART_IER_TX | UART_IER_TXC | UART_IER_RX);

            if (usart->xfer->send_active != 0U) {
                if (usart->dma_tx != NULL) {
                    // In DMA mode - disable DMA channel
                    // DMA disable transmitter
                    //usart->reg->CR3 &= ~USART_CR3_DMAT;
                    // Abort TX DMA transfer
                    //HAL_DMA_Abort (usart->dma_tx->hdma);
                }

                // Wait for loaded frame to be sent
                for (i = 0U; i < 100000U; i++) {
                    if ((usart->reg->CSR & UART_CSR_TXEPT) != 0U) {
                        break;
                    }
                }
            }

            // If DMA mode - disable DMA channel
            if ((usart->dma_rx != NULL) && (usart->info->status.rx_busy != 0U)) {
                // DMA disable Receiver
                //usart->reg->CR3 &= ~USART_CR3_DMAR;
                // Abort RX DMA transfer
                //HAL_DMA_Abort (usart->dma_rx->hdma);
            }

            // Clear busy statuses
            usart->info->status.rx_busy = 0U;
            usart->xfer->send_active = 0U;
            return ARM_DRIVER_OK;

            // Control TX
        case ARM_USART_CONTROL_TX:
            // Check if TX pin available
            if (usart->io.tx == NULL) { return ARM_DRIVER_ERROR; }
            if (arg) {
                if (usart->info->mode != ARM_USART_MODE_SMART_CARD) {
                    if (usart->info->mode == ARM_USART_MODE_SINGLE_WIRE) {
                        // For single-wire tx line is used for both transmission and reception
                        // USART TX pin open-drain with pull-up enabled

                    }
                    else {
                        // USART TX pin function selected

                    }
                }
                usart->info->flags |= USART_FLAG_TX_ENABLED;

                // Transmitter enable
                usart->reg->GCR |= UART_GCR_TX;
            }
            else {
                // Transmitter disable
                usart->reg->GCR &= ~UART_GCR_TX;

                usart->info->flags &= ~USART_FLAG_TX_ENABLED;

                if (usart->info->mode != ARM_USART_MODE_SMART_CARD) {
                    // GPIO pin function selected to disable USART transmission on the pin

                }
            }
            return ARM_DRIVER_OK;

            // Control RX
        case ARM_USART_CONTROL_RX:
            // Check if RX line available except in single-wire mode
            if ((usart->io.rx == NULL) && (usart->info->mode == ARM_USART_MODE_SINGLE_WIRE)) { return ARM_DRIVER_ERROR; }
            if (arg) {
                if (usart->info->mode != ARM_USART_MODE_SMART_CARD) {
                    if (usart->info->mode == ARM_USART_MODE_SINGLE_WIRE) {
                        // For single-wire tx line is used for both transmission and reception
                        // USART TX pin open-drain with pull-up enabled
                        //GPIO_InitStruct.GPIO_Pin       = usart->io.tx->pin;
                        //GPIO_InitStruct.GPIO_Mode      = GPIO_Mode_AF_OD;
                        //GPIO_InitStruct.GPIO_Speed     = usart->io.tx->speed;
                        //GPIO_Init(usart->io.tx->port, &GPIO_InitStruct);

                    }
                    else {
                        // USART RX pin function selected
                        GPIO_Init (usart->io.rx->port, &GPIO_InitStruct);
                        GPIO_PinAFConfig (usart->io.rx->port, usart->io.rx->af_bit, usart->io.rx->af);
                        GPIO_SetBits (usart->io.rx->port, usart->io.rx->pin);
                        GPIO_InitStruct.GPIO_Pin = usart->io.rx->pin;
                        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
                        GPIO_InitStruct.GPIO_Speed = (GPIOSpeed_TypeDef)usart->io.rx->speed;
                        GPIO_Init (usart->io.rx->port, &GPIO_InitStruct);
                    }
                }
                usart->info->flags |= USART_FLAG_RX_ENABLED;

                // Enable Error interrupt, An interrupt is generated whenever DMAR=1 in the USART_CR3 register and FE=1 or ORE=1 or NF=1 in the USART_SR register
                //usart->reg->IER |= UART_IER_E;

                // Break detection interrupt enable
                usart->reg->IER |= UART_IER_RXBRK;

                // Enable Idle line interrupt
                usart->reg->IER |= UART_IER_RXIDLE;

                if (((usart->info->status.rx_busy != 0U) && (usart->dma_rx != NULL)) == false) {
                    usart->reg->IER |= UART_IER_RX;
                }

                // Receiver enable
                usart->reg->GCR |= UART_GCR_RX;

                if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
                    usart->info->flags |= USART_FLAG_TX_ENABLED;
                    // Transmitter enable
                    usart->reg->GCR |= UART_GCR_TX;
                }
            }
            else {
                if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
                    // Transmitter enable
                    usart->reg->GCR &= ~UART_GCR_TX;
                    usart->info->flags &= ~USART_FLAG_TX_ENABLED;
                }

                // Receiver disable
                usart->reg->GCR &= ~UART_GCR_RX;

                usart->info->flags &= ~USART_FLAG_RX_ENABLED;

                if ((usart->info->mode != ARM_USART_MODE_SMART_CARD) &&
                    (usart->info->mode != ARM_USART_MODE_SINGLE_WIRE)) {
                    // GPIO pin function selected to disable USART reception on the pin
                    GPIO_Init (usart->io.rx->port, &GPIO_InitStruct);
                    GPIO_PinAFConfig (usart->io.rx->port, usart->io.rx->af_bit, usart->io.rx->af);
                    GPIO_SetBits (usart->io.rx->port, usart->io.rx->pin);
                    GPIO_InitStruct.GPIO_Pin = usart->io.rx->pin;
                    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
                    GPIO_InitStruct.GPIO_Speed = (GPIOSpeed_TypeDef)usart->io.rx->speed;
                    GPIO_Init (usart->io.rx->port, &GPIO_InitStruct);
                }
            }
            return ARM_DRIVER_OK;
        default: break;
    }

    // Check if busy
    if ((usart->info->status.rx_busy != 0U) || (usart->xfer->send_active != 0U)) {
        return ARM_DRIVER_ERROR_BUSY;
    }

    switch (control & ARM_USART_CONTROL_Msk) {
        case ARM_USART_MODE_ASYNCHRONOUS:
            mode = ARM_USART_MODE_ASYNCHRONOUS;
            break;
        case ARM_USART_MODE_SYNCHRONOUS_MASTER:
            break;
        case ARM_USART_MODE_SYNCHRONOUS_SLAVE:
            return ARM_USART_ERROR_MODE;
        case ARM_USART_MODE_SINGLE_WIRE:
            break;
        case ARM_USART_MODE_IRDA:
            break;
        case ARM_USART_MODE_SMART_CARD:
            break;

            // Default TX value
        case ARM_USART_SET_DEFAULT_TX_VALUE:
            usart->xfer->def_val = (uint16_t)arg;
            return ARM_DRIVER_ERROR_UNSUPPORTED;

            // IrDA pulse
        case ARM_USART_SET_IRDA_PULSE:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

            // SmartCard guard time
        case ARM_USART_SET_SMART_CARD_GUARD_TIME:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

            // SmartCard clock
        case ARM_USART_SET_SMART_CARD_CLOCK:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

            // SmartCard NACK
        case ARM_USART_CONTROL_SMART_CARD_NACK:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

            // Unsupported command
        default: { return ARM_DRIVER_ERROR_UNSUPPORTED; }
    }

    // USART Data bits
    switch (control & ARM_USART_DATA_BITS_Msk) {
        case ARM_USART_DATA_BITS_7:
            usart->reg->CCR |= UART_CCR_CHAR_7b;
            break;
        case ARM_USART_DATA_BITS_8:
            usart->reg->CCR |= UART_CCR_CHAR_8b;
            break;
        case ARM_USART_DATA_BITS_9:
            usart->reg->CCR |= UART_CCR_B8EN;
            usart->reg->GCR |= UART_GCR_SELB8;
            break;
        default: return ARM_USART_ERROR_DATA_BITS;
    }

    // USART Parity
    switch (control & ARM_USART_PARITY_Msk) {
        case ARM_USART_PARITY_NONE:
            usart->reg->CCR &= ~UART_CCR_PEN;
            break;
        case ARM_USART_PARITY_EVEN:
            usart->reg->CCR |= UART_CCR_PEN;
            usart->reg->CCR &= ~UART_CCR_PSEL;
            break;
        case ARM_USART_PARITY_ODD:
            usart->reg->CCR |= UART_CCR_PEN;
            usart->reg->CCR &= ~UART_CCR_PSEL;
            break;
        default: 
            return ARM_USART_ERROR_PARITY;
    }

    // USART Stop bits
    switch (control & ARM_USART_STOP_BITS_Msk) {
        case ARM_USART_STOP_BITS_1:
            usart->reg->CCR &= ~(UART_CCR_SPB0 | UART_CCR_SPB1);
            break;
        case ARM_USART_STOP_BITS_2:   
            usart->reg->CCR &= ~UART_CCR_SPB1;
            usart->reg->CCR |= UART_CCR_SPB0;
            break;
        case ARM_USART_STOP_BITS_1_5: 
            usart->reg->CCR |= UART_CCR_SPB1;
            usart->reg->CCR |= UART_CCR_SPB0;   
            break;
        case ARM_USART_STOP_BITS_0_5: 
            usart->reg->CCR |= UART_CCR_SPB1;
            usart->reg->CCR &= ~UART_CCR_SPB0;
            break;
        default: return ARM_USART_ERROR_STOP_BITS;
    }

    // USART Flow control
    switch (control & ARM_USART_FLOW_CONTROL_Msk) {
        case ARM_USART_FLOW_CONTROL_NONE:
            flow_control = ARM_USART_FLOW_CONTROL_NONE;
            break;
        //case ARM_USART_FLOW_CONTROL_RTS:
        //    return ARM_USART_ERROR_FLOW_CONTROL;
        //    break;
        //case ARM_USART_FLOW_CONTROL_CTS:
        //    return ARM_USART_ERROR_FLOW_CONTROL;
        //    break;
        //case ARM_USART_FLOW_CONTROL_RTS_CTS:
        //    return ARM_USART_ERROR_FLOW_CONTROL;
        //    break;
        default: return ARM_USART_ERROR_FLOW_CONTROL;
    }

    // Clock setting for synchronous mode
    if (mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {

        // Polarity
        switch (control & ARM_USART_CPOL_Msk) {
            case ARM_USART_CPOL0:
                break;
            case ARM_USART_CPOL1:
                //cr2 |= USART_CR2_CPOL;
                break;
            default: return ARM_USART_ERROR_CPOL;
        }

        // Phase
        switch (control & ARM_USART_CPHA_Msk) {
            case ARM_USART_CPHA0:
                break;
            case ARM_USART_CPHA1:
                //cr2 |= USART_CR2_CPHA;
                break;
            default: return ARM_USART_ERROR_CPHA;
        }
    }

    // USART Baudrate
    //val = (uint32_t)(USART_BAUDRATE_DIVIDER (usart->periph_clock ( ), arg));
    //br = ((usart->periph_clock ( ) << 4U) / (val & 0xFFFFU)) >> 4U;
    //
    //
    //// If inside +/- 2% tolerance, baud rate configured correctly
    //if (!(((br * 100U) < (arg * 102U)) && ((br * 100U) > (arg * 98U)))) {
    //    return ARM_USART_ERROR_BAUDRATE;
    //}

    // USART Disable
    usart->reg->GCR &= ~UART_GCR_UART;

    // Configure Baud rate register
    //usart->reg->BRR = val;
    //Configure the UART Baud Rate
    uint32_t apbclock = 0x00;
    if ((usart->reg == UART1) | (usart->reg == UART6)) {
        apbclock = RCC_GetPCLK2Freq();
    }
    else {
        apbclock = RCC_GetPCLK1Freq();
    }
    // Determine the UART_baud
    usart->reg->BRR = (apbclock / arg) / 16;
    usart->reg->FRA = (apbclock / arg) % 16;

    // Configuration is OK - Mode is valid
    usart->info->mode = mode;

    // Save flow control mode
    usart->info->flow_control = flow_control;

    // Configure TX pin regarding mode and transmitter state
    if (usart->io.tx) {
        switch (usart->info->mode) {
            case ARM_USART_MODE_SMART_CARD:
                // USART TX pin function selected
                
                GPIO_InitStruct.GPIO_Pin = usart->io.tx->pin;
                GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
                GPIO_InitStruct.GPIO_Speed = (GPIOSpeed_TypeDef)usart->io.tx->speed;
                GPIO_Init (usart->io.tx->port, &GPIO_InitStruct);
                break;
            case ARM_USART_MODE_SINGLE_WIRE:
                if ((usart->info->flags & (USART_FLAG_TX_ENABLED | USART_FLAG_RX_ENABLED)) != 0U) {
                    // For single-wire tx line is used for both transmission and reception
                    // USART TX pin open-drain with pull-up enabled
                    
                }
                else {
                    // GPIO pin function selected
                    //GPIO_DeInit(usart->io.tx->port, usart->io.tx->pin);
                }
                break;
            default:
                // Synchronous master/slave, asynchronous and IrDA mode
                if (usart->info->flags & USART_FLAG_TX_ENABLED) {
                    // USART TX pin function selected
                    
                }
                else {
                    // GPIO pin function selected
                    //GPIO_DeInit (usart->io.tx->port, usart->io.tx->pin);
                }
                break;
        }
    }

    // Configure RX pin regarding mode and receiver state
    if (usart->io.rx) {
        switch (usart->info->mode) {
            case ARM_USART_MODE_SINGLE_WIRE:
            case ARM_USART_MODE_SMART_CARD:
                // GPIO pin function selected
                //HAL_GPIO_DeInit (usart->io.rx->port, usart->io.rx->pin);
                break;
            default:
                // Synchronous master/slave, asynchronous and IrDA mode
                if (usart->info->flags & USART_FLAG_RX_ENABLED) {
                    // USART RX pin function selected
                    GPIO_Init (usart->io.rx->port, &GPIO_InitStruct);
                    GPIO_PinAFConfig (usart->io.rx->port, usart->io.rx->af_bit, usart->io.rx->af);
                    GPIO_SetBits (usart->io.rx->port, usart->io.rx->pin);
                    GPIO_InitStruct.GPIO_Pin = usart->io.rx->pin;
                    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
                    GPIO_InitStruct.GPIO_Speed = (GPIOSpeed_TypeDef)usart->io.rx->speed;
                    GPIO_Init (usart->io.rx->port, &GPIO_InitStruct);
                }
                else {
                    // GPIO pin function selected
                    //GPIO_DeInit(usart->io.rx->port, usart->io.rx->pin);
                    GPIO_Init (usart->io.rx->port, &GPIO_InitStruct);
                    GPIO_PinAFConfig (usart->io.rx->port, usart->io.rx->af_bit, 0);
                    //GPIO_SetBits (usart->io.rx->port, usart->io.rx->pin);
                    GPIO_InitStruct.GPIO_Pin = usart->io.rx->pin;
                    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
                    GPIO_InitStruct.GPIO_Speed = (GPIOSpeed_TypeDef)usart->io.rx->speed;
                    GPIO_Init (usart->io.rx->port, &GPIO_InitStruct);
                }
                break;
        }
    }

    // Configure CLK pin regarding mode
    if (usart->io.ck) {
        switch (usart->info->mode) {
            case ARM_USART_MODE_SMART_CARD:
            case ARM_USART_MODE_SYNCHRONOUS_MASTER:
                // USART CK pin function selected

                break;
            default:
                // Asynchronous, Single-wire and IrDA mode
                // GPIO pin function selected
                //HAL_GPIO_DeInit (usart->io.ck->port, usart->io.ck->pin);
                break;
        }
    }

    // Configure RTS pin regarding Flow control configuration
    if (usart->io.rts) {
        if ((flow_control == ARM_USART_FLOW_CONTROL_RTS) ||
            (flow_control == ARM_USART_FLOW_CONTROL_RTS_CTS)) {
            // USART RTS Alternate function
        }
        else {
            // GPIO output
        }
    }

    // Configure CTS pin regarding Flow control configuration
    if (usart->io.cts) {
        if ((flow_control == ARM_USART_FLOW_CONTROL_CTS) ||
            (flow_control == ARM_USART_FLOW_CONTROL_RTS_CTS)) {
            // USART CTS Alternate function
        }
        else {
            // GPIO input
        }
    }

    // Configure USART control registers

    // USART Enable
    usart->reg->GCR |= UART_GCR_UART;

    // Set configured flag
    usart->info->flags |= USART_FLAG_CONFIGURED;

    return ARM_DRIVER_OK;
}












static ARM_USART_STATUS USART_GetStatus (const USART_RESOURCES* usart)
{
  ARM_USART_STATUS status;

  memset((void*)&status, 0, sizeof(status));

  if ((usart->info->flags & USART_FLAG_POWERED) == 0U) {
    return status;
  }

  if (usart->xfer->send_active != 0U) {
    status.tx_busy        = 1U;
  } else {
    status.tx_busy        = ((usart->reg->CSR & UART_CSR_TXEPT) ? (0U) : (1U));
  }
  status.rx_busy          = usart->info->status.rx_busy;
  status.tx_underflow     = usart->info->status.tx_underflow;
  status.rx_overflow      = usart->info->status.rx_overflow;
  status.rx_break         = usart->info->status.rx_break;
  status.rx_framing_error = usart->info->status.rx_framing_error;
  status.rx_parity_error  = usart->info->status.rx_parity_error;

  return status;
}

static int32_t USART_SetModemControl (ARM_USART_MODEM_CONTROL control, const USART_RESOURCES* usart)
{
  if ((control != ARM_USART_RTS_CLEAR) && 
      (control != ARM_USART_RTS_SET)   &&
      (control != ARM_USART_DTR_CLEAR) &&
      (control != ARM_USART_DTR_SET)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  switch (control) {
    case ARM_USART_RTS_CLEAR:
      if ((usart->info->flow_control == ARM_USART_FLOW_CONTROL_NONE) ||
          (usart->info->flow_control == ARM_USART_FLOW_CONTROL_CTS)) {
        if (usart->io.rts) {
          //HAL_GPIO_WritePin (usart->io.rts->port, usart->io.rts->pin, GPIO_PIN_SET);
        }
      } else {
        // Hardware RTS
        return ARM_DRIVER_ERROR;
      }
      break;
    case ARM_USART_RTS_SET:
      if ((usart->info->flow_control == ARM_USART_FLOW_CONTROL_NONE) ||
          (usart->info->flow_control == ARM_USART_FLOW_CONTROL_CTS)) {
        if (usart->io.rts) {
            GPIO_ResetBits(usart->io.rts->port, usart->io.rts->pin);
        }
      } else {
        // Hardware RTS
        return ARM_DRIVER_ERROR;
      }
      break;
    case ARM_USART_DTR_CLEAR:
    case ARM_USART_DTR_SET:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

static ARM_USART_MODEM_STATUS USART_GetModemStatus (const USART_RESOURCES* usart)
{
  ARM_USART_MODEM_STATUS modem_status;

  modem_status.cts = 0U;
  if ((usart->info->flow_control == ARM_USART_FLOW_CONTROL_NONE) ||
      (usart->info->flow_control == ARM_USART_FLOW_CONTROL_RTS)) {
    if (usart->io.cts) {
      if (GPIO_ReadInputDataBit(usart->io.cts->port, usart->io.cts->pin) == 0) { //GPIO_PIN_RESET
        modem_status.cts = 1U;
      }
    }
  }
  modem_status.dsr = 0U;
  modem_status.ri  = 0U;
  modem_status.dcd = 0U;

  return modem_status;
}

// static void USART_SignalEvent(uint32_t event)
//{
//     // function body
// }

/**
  \fn          void USART_IRQHandler (const USART_RESOURCES *usart)
  \brief       USART Interrupt handler.
  \param[in]   usart     Pointer to USART resources
*/
void USART_IRQHandler (const USART_RESOURCES* usart)
{
  uint32_t val, sr, event;
  uint16_t data;

  // Read USART status register
  sr = usart->reg->ISR;

  // Reset local variables
  val   = 0U;
  event = 0U;
  data  = 0U;

  /*------- Read Data register not empty ------------*/
  if (sr & UART_ISR_RX & usart->reg->IER) {
    // Check for RX overflow
    if (usart->info->status.rx_busy == 0U) {
      // Receive has not been started

      // Dummy read to flush received data
      usart->reg->RDR; // ????

      usart->info->status.rx_overflow = 1;
      event |= ARM_USART_EVENT_RX_OVERFLOW;
    }
    else {
      if ((usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER)  &&
          (usart->xfer->sync_mode == USART_SYNC_MODE_TX)) {
        // Dummy read in synchronous transmit only mode
        usart->reg->RDR;
      } else {
        // Read data from RX FIFO into receive buffer
        data = (uint16_t)usart->reg->RDR;
      }

      *(usart->xfer->rx_buf++) = (uint8_t)data;

      // If nine bit data, no parity
      //val = usart->reg->CR1;
      //if (((val & USART_CR1_PCE) == 0U) && ((val & USART_CR1_M)   != 0U)) {
      //  *(usart->xfer->rx_buf++) = (uint8_t)(data >> 8U);
      //}
      usart->xfer->rx_cnt++;

      // Check if requested amount of data is received
      if (usart->xfer->rx_cnt == usart->xfer->rx_num) {

        // Disable IDLE interrupt
        usart->reg->IER &= ~UART_IER_RXIDLE;

        // Clear RX busy flag and set receive transfer complete event
        usart->info->status.rx_busy = 0U;
        if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
          val = usart->xfer->sync_mode;
          usart->xfer->sync_mode = 0U;
          switch (val) {
            case USART_SYNC_MODE_TX:
              event |= ARM_USART_EVENT_SEND_COMPLETE;
              break;
            case USART_SYNC_MODE_RX:
              event |= ARM_USART_EVENT_RECEIVE_COMPLETE;
              break;
            case USART_SYNC_MODE_TX_RX:
              event |= ARM_USART_EVENT_TRANSFER_COMPLETE;
              break;
            default: break;
          }
        } else {
          event |= ARM_USART_EVENT_RECEIVE_COMPLETE;
        }
      }
    }
  }

  /*-------------- IDLE line ----------------*/
  if ((sr & UART_ISR_RXIDLE & usart->reg->IER) && ((sr & UART_ISR_RX & usart->reg->IER) == 0U)) {
    // Dummy read to clear IDLE interrupt
    usart->reg->RDR;
    event |= ARM_USART_EVENT_RX_TIMEOUT;
  }

  // Transmit data register empty
  if (sr & UART_ISR_TXC & usart->reg->IER) {

    // Break handling
    if (usart->xfer->break_flag) {
      // Send break
      usart->reg->CCR |= UART_CCR_BRK;

      // Disable TXE interrupt
      usart->reg->IER &= ~UART_IER_TX;
    } else {
      if(usart->xfer->tx_num != usart->xfer->tx_cnt) {
        if ((usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) &&
             (usart->xfer->sync_mode == USART_SYNC_MODE_RX)) {
          // Dummy write in synchronous receive only mode
          data = usart->xfer->def_val;
        } else {
          // Write data to TX FIFO
          data = *(usart->xfer->tx_buf++);

          // If nine bit data, no parity
          //val = usart->reg->CR1;
          //if (((val & USART_CR1_PCE) == 0U) &&
          //    ((val & USART_CR1_M)   != 0U)) {
          //  data |= *(usart->xfer->tx_buf++) << 8U;
          //}
        }
      }
      usart->xfer->tx_cnt++;

      // Write to data register
      usart->reg->TDR = data;

      // Check if all data is transmitted
      if (usart->xfer->tx_num == usart->xfer->tx_cnt) {
        // Disable TXE interrupt
        usart->reg->IER &= ~UART_IER_TX;

        // Enable TC interrupt
        usart->reg->IER |= UART_IER_TXC;

        // Clear current TC as this is not final one
        sr &= ~UART_ISR_TXC; 

        usart->xfer->send_active = 0U;

        // Set send complete event
        if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
          if ((usart->xfer->sync_mode == USART_SYNC_MODE_TX)    &&
              ((usart->info->flags & USART_FLAG_RX_ENABLED) == 0U)) {
            usart->xfer->sync_mode = 0U;
            event |= ARM_USART_EVENT_SEND_COMPLETE;
          }
        } else {
          event |= ARM_USART_EVENT_SEND_COMPLETE;
        }
      }
    }
  }

  /*-------- Transmission complete ------*/
  if (sr & UART_ISR_TXC & usart->reg->IER) {
    // Disable transmission complete interrupt
    usart->reg->IER &= ~UART_IER_TXC;
    usart->xfer->send_active = 0U;
    if (usart->xfer->sync_mode == USART_SYNC_MODE_TX) {
      usart->xfer->sync_mode = 0U;
    }
    event |= ARM_USART_EVENT_TX_COMPLETE;
  }

  // RX Overrun
  if ((sr & UART_ISR_RXOERR) != 0U) {
    // Shift register has been overwritten
    // Dummy data read to clear the ORE flag
    usart->reg->RDR;
      usart->reg->ICR = UART_ICR_RXOERR;
    usart->info->status.rx_overflow = 1U;
    event |= ARM_USART_EVENT_RX_OVERFLOW;
  }

  // Framing error
  if ((sr & UART_ISR_RXFERR) != 0U) {
    // Dummy data read to clear the FE flag
    usart->reg->RDR;
      usart->reg->ICR = UART_ICR_RXFERR;
    usart->info->status.rx_framing_error = 1U;
    event |= ARM_USART_EVENT_RX_FRAMING_ERROR;
  }

  // Parity error
  if ((sr & UART_ISR_RXPERR) != 0U) {
    // Dummy data read to clear the PE flag
    usart->reg->RDR;
      usart->reg->ICR = UART_ICR_RXPERR;
    usart->info->status.rx_parity_error = 1U;
    event |= ARM_USART_EVENT_RX_PARITY_ERROR;
  }

  // Break Detection
  if ((sr & UART_ISR_RXBRK) != 0U) {
    // Clear Break detection flag
    usart->reg->ICR &= ~UART_ICR_RXBRK;
    usart->info->status.rx_break = 1U;
    event |= ARM_USART_EVENT_RX_BREAK;
  }

  // CTS changed (this only works if flow control on CTS line is enabled)
  //if ((sr & USART_SR_CTS) != 0U) {
  //  // Clear CTS flag
  //  usart->reg->SR &= ~USART_SR_CTS;
  //
  //  event |= ARM_USART_EVENT_CTS;
  //}

  // Send Event
  if ((event && usart->info->cb_event) != 0U) {
    usart->info->cb_event (event);
  }
}

// USART6 Driver Wrapper functions
static ARM_USART_CAPABILITIES USART6_GetCapabilities (void) { return USART_GetCapabilities (&USART6_Resources); }
static int32_t USART6_Initialize (ARM_USART_SignalEvent_t cb_event) { return USART_Initialize (cb_event, &USART6_Resources); }
static int32_t USART6_Uninitialize (void) { return USART_Uninitialize (&USART6_Resources); }
static int32_t USART6_PowerControl (ARM_POWER_STATE state) { return USART_PowerControl (state, &USART6_Resources); }
static int32_t USART6_Send (const void* data, uint32_t num) { return USART_Send (data, num, &USART6_Resources); }
static int32_t USART6_Receive (void* data, uint32_t num) { return USART_Receive (data, num, &USART6_Resources); }
static int32_t USART6_Transfer (const void* data_out, void* data_in, uint32_t num) { return USART_Transfer (data_out, data_in, num, &USART6_Resources); }
static uint32_t USART6_GetTxCount (void) { return USART_GetTxCount (&USART6_Resources); }
static uint32_t USART6_GetRxCount (void) { return USART_GetRxCount (&USART6_Resources); }
static int32_t USART6_Control (uint32_t control, uint32_t arg) { return USART_Control (control, arg, &USART6_Resources); }
static ARM_USART_STATUS USART6_GetStatus (void) { return USART_GetStatus (&USART6_Resources); }
static int32_t USART6_SetModemControl (ARM_USART_MODEM_CONTROL control) { return USART_SetModemControl (control, &USART6_Resources); }
static ARM_USART_MODEM_STATUS USART6_GetModemStatus (void) { return USART_GetModemStatus (&USART6_Resources); }
void USART6_IRQHandler (void) { USART_IRQHandler (&USART6_Resources); }

#ifdef MX_USART6_TX_DMA_Instance
void USART6_TX_DMA_Handler (uint32_t events)
{
    USART_TX_DMA_Complete (&USART6_Resources);
}
#endif
#ifdef MX_USART6_RX_DMA_Instance
void USART6_RX_DMA_Handler (uint32_t events)
{
    USART_RX_DMA_Complete (&USART6_Resources);
}
#endif

extern ARM_DRIVER_USART Driver_USART6;
// USART6 Driver Control Block
ARM_DRIVER_USART Driver_USART6 = {
    ARM_USART_GetVersion,
    USART6_GetCapabilities,
    USART6_Initialize,
    USART6_Uninitialize,
    USART6_PowerControl,
    USART6_Send,
    USART6_Receive,
    USART6_Transfer,
    USART6_GetTxCount,
    USART6_GetRxCount,
    USART6_Control,
    USART6_GetStatus,
    USART6_SetModemControl,
    USART6_GetModemStatus };

#endif /**(SWO_UART)**/
