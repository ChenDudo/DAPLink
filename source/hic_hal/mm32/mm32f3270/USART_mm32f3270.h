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
#ifndef __USART_MM32F3270_H
#define __USART_MM32F3270_H


#include "Driver_USART.h"
#include "mm32_device.h"

// USART flags
#define USART_FLAG_INITIALIZED      ((uint8_t)(1U))
#define USART_FLAG_POWERED          ((uint8_t)(1U << 1))
#define USART_FLAG_CONFIGURED       ((uint8_t)(1U << 2))
#define USART_FLAG_TX_ENABLED       ((uint8_t)(1U << 3))
#define USART_FLAG_RX_ENABLED       ((uint8_t)(1U << 4))

// USART synchronous xfer modes
#define USART_SYNC_MODE_TX           ( 1UL )
#define USART_SYNC_MODE_RX           ( 2UL )
#define USART_SYNC_MODE_TX_RX        (USART_SYNC_MODE_TX | \
                                      USART_SYNC_MODE_RX)















// USART Transfer Information (Run-Time)
typedef struct _USART_TRANSFER_INFO {
  uint32_t              rx_num;         // Total number of receive data
  uint32_t              tx_num;         // Total number of transmit data
  uint8_t              *rx_buf;         // Pointer to in data buffer
  uint8_t              *tx_buf;         // Pointer to out data buffer
  uint32_t              rx_cnt;         // Number of data received
  uint32_t              tx_cnt;         // Number of data sent
  uint16_t              dump_val;       // Variable for dumping DMA data
  uint16_t              def_val;        // Default transfer value
  uint32_t              sync_mode;      // Synchronous mode flag
  uint8_t               break_flag;     // Transmit break flag
  uint8_t               send_active;    // Send active flag
} USART_TRANSFER_INFO;

// USART Information (Run-time)
typedef struct _USART_INFO {
  ARM_USART_SignalEvent_t cb_event;            // Event Callback
  ARM_USART_STATUS        status;              // Status flags
  uint8_t                 flags;               // Current USART flags
  uint32_t                mode;                // Current USART mode
  uint32_t                flow_control;        // Flow control
} USART_INFO;

// USART DMA
typedef const struct _USART_DMA {
  DMA_Channel_TypeDef *instance;        // Channel registry interface
  uint8_t              dma_num;         // DMA number
  uint8_t              ch_num;          // Channel number
  uint8_t              priority;        // Channel priority
} USART_DMA;

// USART pin
typedef const struct _USART_PIN {
  GPIO_TypeDef         *port;           // Port
  uint16_t              pin;            // Pin
  uint16_t              af;             // Alternate function
  uint16_t              af_bit;         //pupd;           // Pull up/down
  uint16_t              speed;          // Speed
} USART_PIN;

// USART Input/Output Configuration
typedef const struct _USART_IO {
  USART_PIN            *tx;             // TX  Pin identifier
  USART_PIN            *rx;             // RX  Pin identifier
  USART_PIN            *ck;             // CLK Pin identifier
  USART_PIN            *rts;            // RTS Pin identifier
  USART_PIN            *cts;            // CTS Pin identifier
} USART_IO;

// USART Resources definition
typedef const struct {
  ARM_USART_CAPABILITIES  capabilities;        // Capabilities
  UART_TypeDef            *reg;                 // USART peripheral pointer
  uint32_t                (*periph_clock)(void);                // Peripheral clock frequency
  USART_IO                io;                  // USART Input/Output pins
  IRQn_Type               irq_num;             // USART IRQ Number
  USART_DMA               *dma_tx;             // Transmit stream register interface
  USART_DMA               *dma_rx;             // Receive stream register interface
  USART_INFO              *info;               // Run-Time Information
  USART_TRANSFER_INFO     *xfer;               // USART transfer information
} USART_RESOURCES;


////////////////////////////////////////////////////////////////////////////////
/// @defgroup BEEP_Exported_Variables
/// @{
#ifdef _USART_MM32F3270_C_

#define GLOBAL
#else
#define GLOBAL extern
#endif



#undef GLOBAL

/// @}

////////////////////////////////////////////////////////////////////////////////
/// @defgroup BEEP_Exported_Functions
/// @{



/// @}

/// @}

/// @}

////////////////////////////////////////////////////////////////////////////////
#endif /*__USART_MM32F3270_H */
////////////////////////////////////////////////////////////////////////////////
