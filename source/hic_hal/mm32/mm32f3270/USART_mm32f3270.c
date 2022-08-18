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

#include <string.h>
#include "mm32_device.h"
#include "hal_gpio.h"
#include "hal_uart.h"
#include "hal_misc.h"
#include "hal_rcc.h"
#include "uart.h"
#include "gpio.h"
#include "util.h"
#include "circ_buf.h"
#include "IO_Config.h"

#include "Driver_USART.h"
#include "USART_mm32f3270.h"

#define SWO_UART                UART6
#define SWO_UART_IRQn           UART6_IRQn
#define SWO_UART_IRQn_Handler   UART6_IRQHandler
#define SWO_UART_ENABLE()       RCC_APB2PeriphClockCmd(RCC_APB2ENR_UART6, ENABLE)
#define SWO_UART_DISABLE()      RCC_APB2PeriphClockCmd(RCC_APB2ENR_UART6, DISABLE)

#define ARM_USART_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)  /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = { 
    ARM_USART_API_VERSION,
    ARM_USART_DRV_VERSION
};

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

static ARM_DRIVER_VERSION ARM_USART_GetVersion(void)
{
  return DriverVersion;
}

static ARM_USART_CAPABILITIES ARM_USART_GetCapabilities(void)
{
  return DriverCapabilities;
}

static int32_t ARM_USART_Initialize(ARM_USART_SignalEvent_t cb_event, const USART_RESOURCES *usart)
{
	if (usart->info->flags & USART_FLAG_INITIALIZED) {
    // Driver is already initialized
    return ARM_DRIVER_OK;
  }

  // Initialize USART Run-time Resources
  usart->info->cb_event = cb_event;

  usart->info->status.tx_busy          = 0U;
  usart->info->status.rx_busy          = 0U;
  usart->info->status.tx_underflow     = 0U;
  usart->info->status.rx_overflow      = 0U;
  usart->info->status.rx_break         = 0U;
  usart->info->status.rx_framing_error = 0U;
  usart->info->status.rx_parity_error  = 0U;

  usart->info->mode        = 0U;
  usart->xfer->send_active = 0U;

  // Clear transfer information
  memset(usart->xfer, 0, sizeof(USART_TRANSFER_INFO));

  // Setup pin remap
  //GPIO_AFConfigure(usart->io.afio);

  // Enable TX pin port clock
  if (usart->io.tx) {
    //GPIO_PortClock (usart->io.tx->port, true);
  }

  // Enable RX pin port clock
  if (usart->io.rx) {
    //GPIO_PortClock (usart->io.rx->port, true);
  }

  // Enable CK pin port clock
  if (usart->io.ck) {
    //GPIO_PortClock (usart->io.ck->port, true);
  }

  // Enable RTS pin port clock
  if (usart->io.rts) {
    //GPIO_PortClock (usart->io.rts->port, true);
  }

  // Enable CTS pin port clock
  if (usart->io.cts) {
    //GPIO_PortClock (usart->io.cts->port, true);
  }

  usart->info->flags = USART_FLAG_INITIALIZED;

  return ARM_DRIVER_OK;
}

static int32_t ARM_USART_Uninitialize(const USART_RESOURCES *usart)
{
  // Unconfigure USART pins
  if (usart->io.tx)
	  //GPIO_PinConfigure(usart->io.tx->port,  usart->io.tx->pin,  GPIO_IN_ANALOG, GPIO_MODE_INPUT);
  if (usart->io.rx)
	  //GPIO_PinConfigure(usart->io.rx->port,  usart->io.rx->pin,  GPIO_IN_ANALOG, GPIO_MODE_INPUT);
  if (usart->io.ck)
	  //GPIO_PinConfigure(usart->io.ck->port,  usart->io.ck->pin,  GPIO_IN_ANALOG, GPIO_MODE_INPUT);
  if (usart->io.rts)
	  //GPIO_PinConfigure(usart->io.rts->port, usart->io.rts->pin, GPIO_IN_ANALOG, GPIO_MODE_INPUT);
  if (usart->io.cts) 
	  //GPIO_PinConfigure(usart->io.cts->port, usart->io.cts->pin, GPIO_IN_ANALOG, GPIO_MODE_INPUT);

  // Unconfigure pin remap
  //GPIO_AFConfigure(usart->io.afio_def);

  // Reset USART status flags
  usart->info->flags = 0U;

  return ARM_DRIVER_OK;
}

static int32_t ARM_USART_PowerControl(ARM_POWER_STATE state, const USART_RESOURCES *usart)
{
    switch (state)
    {
    case ARM_POWER_OFF:
		RCC->APB2RSTR |= RCC_APB2RSTR_UART6;
		NVIC_DisableIRQ (usart->irq_num);
	
#ifdef __USART_DMA
      if (usart->dma_rx) {
        // Deinitialize DMA
        DMA_ChannelUninitialize (usart->dma_rx->dma_num, usart->dma_rx->ch_num);
      }

      if (usart->dma_tx) {
        // Deinitialize DMA
        DMA_ChannelUninitialize (usart->dma_tx->dma_num, usart->dma_tx->ch_num);
      }
#endif
		
// Disable USART clock
      if (usart->reg == UART6) { RCC->APB2ENR &= ~RCC_APB2ENR_UART6; }

      // Clear Status flags
      usart->info->status.tx_busy          = 0U;
      usart->info->status.rx_busy          = 0U;
      usart->info->status.tx_underflow     = 0U;
      usart->info->status.rx_overflow      = 0U;
      usart->info->status.rx_break         = 0U;
      usart->info->status.rx_framing_error = 0U;
      usart->info->status.rx_parity_error  = 0U;
      usart->xfer->send_active             = 0U;

      usart->info->flags &= ~USART_FLAG_POWERED;	
        break;

    case ARM_POWER_LOW:
        return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
		      if ((usart->info->flags & USART_FLAG_INITIALIZED) == 0U) {
        return ARM_DRIVER_ERROR;
      }
      if ((usart->info->flags & USART_FLAG_POWERED)     != 0U) {
        return ARM_DRIVER_OK;
      }

      // Clear Status flags
      usart->info->status.tx_busy          = 0U;
      usart->info->status.rx_busy          = 0U;
      usart->info->status.tx_underflow     = 0U;
      usart->info->status.rx_overflow      = 0U;
      usart->info->status.rx_break         = 0U;
      usart->info->status.rx_framing_error = 0U;
      usart->info->status.rx_parity_error  = 0U;

      usart->xfer->send_active             = 0U;
      usart->xfer->def_val                 = 0U;
      usart->xfer->sync_mode               = 0U;
      usart->xfer->break_flag              = 0U;
      usart->info->mode                    = 0U;
      usart->info->flow_control            = 0U;

      usart->info->flags = USART_FLAG_POWERED | USART_FLAG_INITIALIZED;

      // Enable USART clock
      if (usart->reg == UART6) { RCC->APB2ENR |= RCC_APB2ENR_UART6; }

		// Clear and Enable USART IRQ
		NVIC_ClearPendingIRQ(usart->irq_num);
		NVIC_EnableIRQ(usart->irq_num);

#ifdef __USART_DMA
      if (usart->dma_rx) {
         // Initialize DMA
         DMA_ChannelInitialize (usart->dma_rx->dma_num, usart->dma_rx->ch_num);
      }
      if (usart->dma_tx) {
        // Initialize DMA
        DMA_ChannelInitialize (usart->dma_tx->dma_num, usart->dma_tx->ch_num);
      }
#endif
		// Peripheral reset
		RCC->APB2RSTR |= RCC_APB2RSTR_UART6;
        break;
    }
    return ARM_DRIVER_OK;
}

static int32_t ARM_USART_Send(const void *data, uint32_t num, const USART_RESOURCES *usart)
{
#ifdef __USART_DMA_TX
  uint32_t cfg, cr1;
#endif

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
  usart->xfer->tx_buf = (uint8_t *)data;
  usart->xfer->tx_num = num;
  usart->xfer->tx_cnt = 0U;

#ifdef __USART_DMA_TX
  cfg = DMA_MEMORY_INCREMENT;
#endif

  // Synchronous mode
  if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
    if (usart->xfer->sync_mode == 0U) {
      usart->xfer->sync_mode = USART_SYNC_MODE_TX;
      // Start dummy reads
      //stat = UART_Receive (&usart->xfer->dump_val, num, usart);
      //if (stat == ARM_DRIVER_ERROR_BUSY) { return ARM_DRIVER_ERROR_BUSY; }

#ifdef __USART_DMA_TX
    } else {
      if (usart->xfer->sync_mode == USART_SYNC_MODE_RX) {
        // Dummy DMA writes (do not increment source address)
        cfg = 0;
      }
#endif
    }
  }

#ifdef __USART_DMA_TX
  // DMA mode
  if (usart->dma_tx) {
    // Configure and enable tx DMA channel
    cfg |= ((usart->dma_tx->priority << DMA_PRIORITY_POS) & DMA_PRIORITY_MASK) |
             DMA_READ_MEMORY                |
             DMA_TRANSFER_COMPLETE_INTERRUPT;

    cr1 = usart->reg->CR1;
    if (((cr1 & USART_CR1_M) != 0U) && ((cr1 & USART_CR1_PCE) == 0U)) {
      // 9-bit data frame, no parity
      cfg |= DMA_PERIPHERAL_DATA_16BIT | DMA_MEMORY_DATA_16BIT;
    } else {
      // 8-bit data frame
      cfg |= DMA_PERIPHERAL_DATA_8BIT | DMA_MEMORY_DATA_8BIT;
    }

    DMA_ChannelConfigure(usart->dma_tx->instance,
                         cfg,
                         (uint32_t)(&usart->reg->DR),
                         (uint32_t)(uint32_t)data,
                         num);
    DMA_ChannelEnable(usart->dma_tx->instance);

    // DMA Enable transmitter
    usart->reg->CR3 |= USART_CR3_DMAT;
  } else
#endif
  // Interrupt mode
  {
    // TXE interrupt enable
    usart->reg->GCR |= UART_GCR_TX; //????chend
  }

  return ARM_DRIVER_OK;
}

static int32_t ARM_USART_Receive(void *data, uint32_t num)
{
}

static int32_t ARM_USART_Transfer(const void *data_out, void *data_in, uint32_t num)
{
}

static uint32_t ARM_USART_GetTxCount(void)
{
}

static uint32_t ARM_USART_GetRxCount(void)
{
}

static int32_t ARM_USART_Control(uint32_t control, uint32_t arg)
{
}

static ARM_USART_STATUS ARM_USART_GetStatus(void)
{
}

static int32_t ARM_USART_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
}

static ARM_USART_MODEM_STATUS ARM_USART_GetModemStatus(void)
{
}

static void ARM_USART_SignalEvent(uint32_t event)
{
    // function body
}

// End USART Interface

extern \
ARM_DRIVER_USART Driver_USART0;
ARM_DRIVER_USART Driver_USART0 = {
    ARM_USART_GetVersion,
    ARM_USART_GetCapabilities,
    ARM_USART_Initialize,
    ARM_USART_Uninitialize,
    ARM_USART_PowerControl,
    ARM_USART_Send,
    ARM_USART_Receive,
    ARM_USART_Transfer,
    ARM_USART_GetTxCount,
    ARM_USART_GetRxCount,
    ARM_USART_Control,
    ARM_USART_GetStatus,
    ARM_USART_SetModemControl,
    ARM_USART_GetModemStatus
};
