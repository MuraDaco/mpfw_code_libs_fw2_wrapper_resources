//  *******************************************************************************
//  
//  mpfw / fw2 - Multi Platform FirmWare FrameWork 
//      library that contains the wrapper code to manage platform resources
//  Copyright (C) (2023) Marco Dau
//  
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Affero General Public License as published
//  by the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//  
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Affero General Public License for more details.
//  
//  You should have received a copy of the GNU Affero General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.
//  
//  You can contact me by the following email address
//  marco <d o t> ing <d o t> dau <a t> gmail <d o t> com
//  
//  *******************************************************************************
/*
 * rsDbgConsole.cpp
 *
 *  Created on: July, 5 2023
 *      Author: Marco Dau
 */

/*

    - uart module management
        - doc:
            - LPC546XX_LPC546xx_Datasheet.pdf
                - an overview of micro
            - LPC546xx User manual:
                - UM10912_LPC546xx_User_Manual.pdf
                    - chapter to see:
                        - Chapter 10: LPC546xx I/O pin configuration (IOCON)
                            - pag 196 - Table 256 (Type D I/O Control registers: FUNC values and pin functions)
                        - Chapter 25: LPC546xx USARTs
            - board schematics:
                - LPC54608_Dev_brd_schematic_Rev-D_170614.pdf
            - UM11035.pdf
                - board description
                    - switches/buttons, jumpers & connectors functions
        - code:
            - determine the uart pin to use
                - board schematichs at 
                    - sheet 3: 
                        - LPC54608FET180
                            - pin P0_29-ISP_FC0_RXD (B12 - P0_29)
                            - pin P0_30-ISP_FC0_TXD (A 2 - P0_30)
                    - sheet 11  
                        - 74AVC4TD245BQ chip
                            - pin P0_29-ISP_FC0_RXD (B2)
                            - pin P0_30-ISP_FC0_TXD (B3)
                            - pin BRIDGE_UART_RXD (A2)
                            - pin BRIDGE_UART_TXD (A3)
                    - sheet 12
                        - LPC4322JET100 micro 
                            - pin BRIDGE_UART_RXD (G10 - P2[0])
                            - pin BRIDGE_UART_TXD (G 7 - P2[1])
                    - sheet 13: 
                        - microB USB connector <-> LPC4322JET100 micro
                            - pin USB_DM_LINK <-> (E1 - USB0_DM)
                            - pin USB_DP_LINK <-> (E2 - USB0_DP)
            - pin configuration
                - LPC546xx User manual at
                    - Chapter 10: LPC546xx I/O pin configuration (IOCON)
                        - pag 196 - Table 256 (Type D I/O Control registers: FUNC values and pin functions)
                - code
                    - see "BOARD_InitDEBUG_UARTPins(.)" function
            - setting uart module
                - 
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    RESET_PeripheralReset(BOARD_DEBUG_UART_RST);
    NVIC_SetPriority

 */

// module include
#include <rsDbgConsole.h>

// sdk
#include "fsl_usart.h"
#include "fsl_common.h"
#include "fsl_iocon.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "board.h"

// lib include
#include <dgFormatDigit.h>

using namespace std;
using namespace fw2::core::core;

namespace fw2 { namespace wrapper { namespace resources	{



constexpr char rsDbgConsole::endl[];

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * DATA Definitions
 ******************************************************************************/
#define DEMO_USART          USART0
#define DEMO_USART_IRQn     FLEXCOMM0_IRQn
#define USART_NVIC_PRIO     5

uint8_t                     rsDbgConsole::background_buffer[100];
usart_rtos_handle_t         rsDbgConsole::handle;
struct _usart_handle        rsDbgConsole::t_handle;
struct rtos_usart_config    rsDbgConsole::usart_config= {
        .baudrate    = 115200,
        .parity      = kUSART_ParityDisabled,                       // define in fsl_usart.h -> typedef enum _usart_parity_mode
        .stopbits    = kUSART_OneStopBit,                           // define in fsl_usart.h -> typedef enum _usart_stop_bit_count
        .srcclk      = BOARD_DEBUG_UART_CLK_FREQ;                   // define in board.h     -> #define BOARD_DEBUG_UART_CLK_FREQ CLOCK_GetFlexCommClkFreq(0)
        .base        = DEMO_USART;
        .buffer      = rsDbgConsole::background_buffer,
        .buffer_size = sizeof(rsDbgConsole::background_buffer),
    };

/*******************************************************************************
 * FUNCTION Definitions
 ******************************************************************************/

bool rsDbgConsole::CheckStatus            (int p_status)    {
    return (kStatus_Success == p_status);
}

void rsDbgConsole::Init             (void)      {
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    // IOCON configuration
    BOARD_InitDEBUG_UARTPins();
    
}


void rsDbgConsole::InitFreeRTOS             (void)      {

    NVIC_SetPriority(DEMO_USART_IRQn, USART_NVIC_PRIO);

    if (0 > USART_RTOS_Init(&handle, &t_handle, &usart_config))
    {
        vTaskSuspend(NULL);
    }
    
}

void rsDbgConsole::DeinitFreeRTOS             (void)      {
    USART_RTOS_Deinit(&handle);
    vTaskSuspend(NULL);

}


int rsDbgConsole::Tx             (uint8_t* p_Buffer, uint8_t p_SizeBuffer)      {
    int l_status;
    // ...................................0.........1.........2.........3.........4.........5.........
    // ...................................123456789012345678901234567890123456789012345678901234567890
    USART_RTOS_Send(&handle, (uint8_t* ) "\r\nTxFreeRTOS -> char*\r\n", 23);
    /* Send introduction message. */

    l_status = USART_RTOS_Send(&handle, p_Buffer, p_SizeBuffer);
    if (0 > l_status)
    {
        vTaskSuspend(NULL);
    }

    return l_status;
}

int rsDbgConsole::Tx             (char* p_Buffer, uint8_t p_SizeBuffer)      {
    int l_status;
    // ...................................0.........1.........2.........3.........4.........5.........
    // ...................................123456789012345678901234567890123456789012345678901234567890
    USART_RTOS_Send(&handle, (uint8_t* ) "\r\nTxFreeRTOS -> char*\r\n", 23);
    /* Send introduction message. */

    l_status = USART_RTOS_Send(&handle, (uint8_t* ) p_Buffer, p_SizeBuffer);
    if (0 > l_status)
    {
        vTaskSuspend(NULL);
    }

    return l_status;
}

int rsDbgConsole::Tx             (const char* p_Buffer, uint8_t p_SizeBuffer)      {
    int l_status;
    // ...................................0.........1.........2.........3.........4.........5.........
    // ...................................123456789012345678901234567890123456789012345678901234567890
    USART_RTOS_Send(&handle, (uint8_t* ) "\r\nTxFreeRTOS -> const char*\r\n", 29);
    /* Send introduction message. */
    l_status = USART_RTOS_Send(&handle, (uint8_t* ) p_Buffer, p_SizeBuffer);
    if (0 > l_status)
    {
        vTaskSuspend(NULL);
    }

    return l_status;
}

int rsDbgConsole::RxFreeRTOS             (char* p_Buffer, uint8_t p_SizeBuffer, uint8_t* p_ptrByteRxCounter)      {
    // .......................0.........1.........2.........3.........4.........5.........6.........7.........8.........9.........0.........1
    // .......................12345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
    const char *send_buffer_overrun = "\r\nRing buffer overrun!\r\n";

    int l_status;

    l_status = USART_RTOS_Receive(&handle, (uint8_t *) p_Buffer, p_SizeBuffer, (size_t*) p_ptrByteRxCounter);

    if (l_status == kStatus_USART_RxRingBufferOverrun)
    {
        /* Notify about hardware buffer overrun */
        if (kStatus_Success !=
            USART_RTOS_Send(&handle, (uint8_t *)send_buffer_overrun, strlen(send_buffer_overrun)))
        {
            vTaskSuspend(NULL);
        }
    }

    return l_status;
}


int rsDbgConsole::RxFreeRTOS             (const char* p_Buffer, uint8_t p_SizeBuffer, uint8_t* p_ptrByteRxCounter)      {
    // .......................0.........1.........2.........3.........4.........5.........6.........7.........8.........9.........0.........1
    // .......................12345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
    const char *send_buffer_overrun = "\r\nRing buffer overrun!\r\n";

    int l_status;

    l_status = USART_RTOS_Receive(&handle, (uint8_t *) p_Buffer, p_SizeBuffer, (size_t*) p_ptrByteRxCounter);


    if (l_status == kStatus_USART_RxRingBufferOverrun)
    {
        /* Notify about hardware buffer overrun */
        if (kStatus_Success !=
            USART_RTOS_Send(&handle, (uint8_t *)send_buffer_overrun, strlen(send_buffer_overrun)))
        {
            vTaskSuspend(NULL);
        }
    }

    return l_status;
}



#define DIGIT_NUM_3     3
#define DIGIT_NUM_5     5
#define DIGIT_NUM_6     6
#define HANDLE_SERIAL USART4

void rsDbgConsole::Tx       (uint8_t p_Num)  {
    dgFormatDigit l_FormatNum (&p_Num, 1, DIGIT_NUM_3, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum.vValueToString();

    // ******************************************************************************************
    TxEnable();
    USART_WriteBlocking(HANDLE_SERIAL, (uint8_t* ) l_FormatNum.m_StringValueField, DIGIT_NUM_3);
}

void rsDbgConsole::Tx       (uint16_t p_Num)  {
    dgFormatDigit l_FormatNum (&p_Num, 2, DIGIT_NUM_5, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum.vValueToString();

    // ******************************************************************************************
    TxEnable();
    USART_WriteBlocking(HANDLE_SERIAL, (uint8_t* ) l_FormatNum.m_StringValueField, DIGIT_NUM_5);
}

void rsDbgConsole::Tx       (uint32_t p_Num)  {
    dgFormatDigit l_FormatNum (&p_Num, 4, DIGIT_NUM_6, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum.vValueToString();

    // ******************************************************************************************
    TxEnable();
    USART_WriteBlocking(HANDLE_SERIAL, (uint8_t* ) l_FormatNum.m_StringValueField, DIGIT_NUM_6);
}

void rsDbgConsole::Tx       (uint8_t* p_CharBufferTx)  {
	uint8_t l_SizeBuf = 0;
    // determine the size of string
    for(uint8_t i=0;i<254; i++) {
        if(*(p_CharBufferTx+i) == 0) {
            l_SizeBuf = i;
            break;
        }
    }

    // ******************************************************************************************
    TxEnable();
    USART_WriteBlocking(HANDLE_SERIAL, p_CharBufferTx, l_SizeBuf);
}

void rsDbgConsole::Tx       (char* p_CharBufferTx)  {
	uint8_t l_SizeBuf = 0;
    // determine the size of string
    for(uint8_t i=0;i<254; i++) {
        if(*(p_CharBufferTx+i) == 0) {
            l_SizeBuf = i;
            break;
        }
    }

    // ******************************************************************************************
    TxEnable();
    USART_WriteBlocking(HANDLE_SERIAL, (uint8_t* ) (p_CharBufferTx), l_SizeBuf);
}

void rsDbgConsole::Tx       (char const *p_CharBufferTx)  {
	uint8_t l_SizeBuf = 0;
    // determine the size of string
    for(uint8_t i=0;i<254; i++) {
        if(*(p_CharBufferTx+i) == 0) {
            l_SizeBuf = i;
            break;
        }
    }

    // ******************************************************************************************
    TxEnable();
    USART_WriteBlocking(HANDLE_SERIAL, (uint8_t* const) (p_CharBufferTx), l_SizeBuf);
}

void rsDbgConsole::Tx       (char* p_CharBufferTx, uint8_t p_SizeBuf)  {
    // ******************************************************************************************
    TxEnable();
    USART_WriteBlocking(HANDLE_SERIAL, (uint8_t* ) (p_CharBufferTx), p_SizeBuf);
}

void rsDbgConsole::Tx       (char const *p_CharBufferTx,  uint8_t p_SizeBuf)  {
    // ******************************************************************************************
    TxEnable();
    USART_WriteBlocking(HANDLE_SERIAL, (uint8_t* const) (p_CharBufferTx), p_SizeBuf);
}

void rsDbgConsole::Rx       (uint8_t* p_CharBufferTx, size_t p_SizeOf)  {
    // ******************************************************************************************
    RxEnable();
    USART_ReadBlocking(DEMO_USART_MARCO_TEST, p_CharBufferTx, p_SizeOf);
    p_CharBufferTx[p_SizeOf] = 0;
}

void rsDbgConsole::Rx       (char* p_CharBufferTx, size_t p_SizeOf)  {
    // ******************************************************************************************
    RxEnable();
    USART_ReadBlocking(DEMO_USART_MARCO_TEST, (uint8_t *) p_CharBufferTx, p_SizeOf);
    p_CharBufferTx[p_SizeOf] = 0;
}

char rsDbgConsole::Rx    (void)  {
    uint8_t ch = 0;

    // ******************************************************************************************
    RxEnable();
    USART_ReadBlocking(DEMO_USART_MARCO_TEST, &ch, 1);

    return ch;
}



} } }   // fw2::wrapper::resources

