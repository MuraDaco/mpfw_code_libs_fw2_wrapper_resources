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
 * rsSerial.cpp
 *
 *  Created on: Nov, 30 2022
 *      Author: Marco Dau
 */

// module include
#include <rsSerial.h>

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


bool rsSerial::CheckStatus            (int p_status)    {
    return (kStatus_Success == p_status);
}


constexpr char rsSerial::endl[];

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define DEMO_USART_MARCO_TEST                    USART4
#define DEMO_USART_CLK_FREQ                      CLOCK_GetFlexCommClkFreq(0)
#define DEMO_USART_MARCO_TEST_CLK_FREQ           CLOCK_GetFlexCommClkFreq(0)

// *********************************************
#define APP_BOARD_MARCO_TEST_SW2_PORT 0U
#define APP_BOARD_MARCO_TEST_SW2_PIN  6U

#define APP_BOARD_MARCO_TEST_LED2_PORT 3U
#define APP_BOARD_MARCO_TEST_LED2_PIN  3U

// *********************************************
#define APP_BOARD_MARCO_TEST_SW3_PORT 0U
#define APP_BOARD_MARCO_TEST_SW3_PIN  5U

#define APP_BOARD_MARCO_TEST_LED3_PORT 2U
#define APP_BOARD_MARCO_TEST_LED3_PIN  2U


void rsSerial::InitPins_IOCON (void)      {

    /* Enables the clock for the IOCON block. 0 = Disable; 1 = Enable.: 0x01u */
    CLOCK_EnableClock(kCLOCK_Iocon);

    // *********************************************************************************************************
    // usart4 RX & TX pin configuration
    const uint32_t port3_pin26_config = (/* Pin is configured as FC4_RXD_SDA_MOSI */
                                         IOCON_PIO_FUNC3 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Input filter disabled */
                                         IOCON_PIO_INPFILT_OFF |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN29 (coords: K5) is configured as FC4_RXD_SDA_MOSI */
    IOCON_PinMuxSet(IOCON, 3U, 26U, port3_pin26_config);

    const uint32_t port3_pin27_config = (/* Pin is configured as FC4_TXD_SDA_MOSI */
                                         IOCON_PIO_FUNC3 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Input filter disabled */
                                         IOCON_PIO_INPFILT_OFF |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN30 (coords: P14) is configured as FC4_TXD_SDA_MOSI */
    IOCON_PinMuxSet(IOCON, 3U, 27U, port3_pin27_config);


    // *********************************************************************************************************
    // pin to read the sw2 button state
    const uint32_t port0_pin6_config = (/* Pin is configured as PIO0_6 */
                                        IOCON_PIO_FUNC0 |
                                        /* Selects pull-up function */
                                        IOCON_PIO_MODE_PULLUP |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Input filter disabled */
                                        IOCON_PIO_INPFILT_OFF |
                                        /* Standard mode, output slew rate control is enabled */
                                        IOCON_PIO_SLEW_STANDARD |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN6 (coords: A5) is configured as PIO0_6 */
    IOCON_PinMuxSet(IOCON, 0U, 6U, port0_pin6_config);


    // pin to drive led 2 & max 485-DE
    const uint32_t port3_pin3_config = (/* Pin is configured as PIO3_3 */
                                        IOCON_PIO_FUNC0 |
                                        /* Selects pull-up function */
                                        IOCON_PIO_MODE_PULLUP |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Input filter disabled */
                                        IOCON_PIO_INPFILT_OFF |
                                        /* Standard mode, output slew rate control is enabled */
                                        IOCON_PIO_SLEW_STANDARD |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT3 PIN3 (coords: A13) is configured as PIO3_3 */
    IOCON_PinMuxSet(IOCON, 3U, 3U, port3_pin3_config);


    // *********************************************************************************************************
    // pin to read the sw3 button state
    const uint32_t port0_pin5_config = (/* Pin is configured as PIO0_6 */
                                        IOCON_PIO_FUNC0 |
                                        /* Selects pull-up function */
                                        IOCON_PIO_MODE_PULLUP |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Input filter disabled */
                                        IOCON_PIO_INPFILT_OFF |
                                        /* Standard mode, output slew rate control is enabled */
                                        IOCON_PIO_SLEW_STANDARD |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN6 (coords: E7) is configured as PIO0_5 */
    IOCON_PinMuxSet(IOCON, 0U, 5U, port0_pin5_config);


    // pin to drive led 3 & max 485-RE
    const uint32_t port2_pin2_config = (/* Pin is configured as PIO2_2 */
                                        IOCON_PIO_FUNC0 |
                                        /* Selects pull-up function */
                                        IOCON_PIO_MODE_PULLUP |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Input filter disabled */
                                        IOCON_PIO_INPFILT_OFF |
                                        /* Standard mode, output slew rate control is enabled */
                                        IOCON_PIO_SLEW_STANDARD |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT3 PIN3 (coords: C3) is configured as PIO3_3 */
    IOCON_PinMuxSet(IOCON, 2U, 2U, port2_pin2_config);

}

void rsSerial::InitPins_GPIO    (void)      {

    // enable clock & reset peripheral according with the compile time definitions
    GPIO_PortInit(GPIO, APP_BOARD_MARCO_TEST_SW2_PORT );
    GPIO_PortInit(GPIO, APP_BOARD_MARCO_TEST_LED2_PORT);

    GPIO_PortInit(GPIO, APP_BOARD_MARCO_TEST_SW3_PORT );
    GPIO_PortInit(GPIO, APP_BOARD_MARCO_TEST_LED3_PORT);

    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        0,
    };

    GPIO_PinInit (GPIO, APP_BOARD_MARCO_TEST_LED2_PORT, APP_BOARD_MARCO_TEST_LED2_PIN, &led_config);
    // rs485 tx enable
    GPIO_PinWrite(GPIO, APP_BOARD_MARCO_TEST_LED2_PORT, APP_BOARD_MARCO_TEST_LED2_PIN, 1);

    GPIO_PinInit (GPIO, APP_BOARD_MARCO_TEST_LED3_PORT, APP_BOARD_MARCO_TEST_LED3_PIN, &led_config);
    // rs485 rx disable
    GPIO_PinWrite(GPIO, APP_BOARD_MARCO_TEST_LED3_PORT, APP_BOARD_MARCO_TEST_LED3_PIN, 1);
}

void rsSerial::InitUSART        (void)      {
    // ............................12345678901234567890123456789012345678901234567890
    uint8_t l_strFirstUARTMsg[] = "krInitStatic -> UART initialization completed\r\n";

    usart_config_t config;

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUSART_ParityDisabled;
     * config.stopBitCount = kUSART_OneStopBit;
     * config.loopback = false;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;

    USART_Init(DEMO_USART_MARCO_TEST, &config, DEMO_USART_MARCO_TEST_CLK_FREQ  );

    USART_WriteBlocking(DEMO_USART_MARCO_TEST, l_strFirstUARTMsg, sizeof(l_strFirstUARTMsg) - 1);

}


void rsSerial::Init             (void)      {
    /* attach 12 MHz clock to FLEXCOMM4 (USART) */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);
    RESET_PeripheralReset(kFC4_RST_SHIFT_RSTn);

    InitPins_IOCON();
    InitPins_GPIO();
    InitUSART();

}

void rsSerial::TxEnable    (void)  {
    // tx enable
    GPIO_PinWrite(GPIO, APP_BOARD_MARCO_TEST_LED2_PORT, APP_BOARD_MARCO_TEST_LED2_PIN, 1);
    // rx disable
    GPIO_PinWrite(GPIO, APP_BOARD_MARCO_TEST_LED3_PORT, APP_BOARD_MARCO_TEST_LED3_PIN, 1);
}

void rsSerial::RxEnable    (void)  {
    // tx disable
    GPIO_PinWrite(GPIO, APP_BOARD_MARCO_TEST_LED2_PORT, APP_BOARD_MARCO_TEST_LED2_PIN, 0);
    // rx enable
    GPIO_PinWrite(GPIO, APP_BOARD_MARCO_TEST_LED3_PORT, APP_BOARD_MARCO_TEST_LED3_PIN, 0);
}

void rsSerial::Tx    (char p_Char)  {
    //cout << p_Char;
    p_Char = 0;
}

#define DIGIT_NUM_3     3
#define DIGIT_NUM_5     5
#define DIGIT_NUM_6     6
#define HANDLE_SERIAL USART4

void rsSerial::Tx       (uint8_t p_Num)  {
    dgFormatDigit l_FormatNum (&p_Num, 1, DIGIT_NUM_3, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum.vValueToString();

    // ******************************************************************************************
    TxEnable();
    USART_WriteBlocking(HANDLE_SERIAL, (uint8_t* ) l_FormatNum.m_StringValueField, DIGIT_NUM_3);
}

void rsSerial::Tx       (uint16_t p_Num)  {
    dgFormatDigit l_FormatNum (&p_Num, 2, DIGIT_NUM_5, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum.vValueToString();

    // ******************************************************************************************
    TxEnable();
    USART_WriteBlocking(HANDLE_SERIAL, (uint8_t* ) l_FormatNum.m_StringValueField, DIGIT_NUM_5);
}

void rsSerial::Tx       (uint32_t p_Num)  {
    dgFormatDigit l_FormatNum (&p_Num, 4, DIGIT_NUM_6, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum.vValueToString();

    // ******************************************************************************************
    TxEnable();
    USART_WriteBlocking(HANDLE_SERIAL, (uint8_t* ) l_FormatNum.m_StringValueField, DIGIT_NUM_6);
}

void rsSerial::Tx       (uint8_t* p_CharBufferTx)  {
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

void rsSerial::Tx       (char* p_CharBufferTx)  {
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

void rsSerial::Tx       (char const *p_CharBufferTx)  {
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

void rsSerial::Tx       (char* p_CharBufferTx, uint8_t p_SizeBuf)  {
    // ******************************************************************************************
    TxEnable();
    USART_WriteBlocking(HANDLE_SERIAL, (uint8_t* ) (p_CharBufferTx), p_SizeBuf);
}

void rsSerial::Tx       (char const *p_CharBufferTx,  uint8_t p_SizeBuf)  {
    // ******************************************************************************************
    TxEnable();
    USART_WriteBlocking(HANDLE_SERIAL, (uint8_t* const) (p_CharBufferTx), p_SizeBuf);
}

void rsSerial::Rx       (uint8_t* p_CharBufferTx, size_t p_SizeOf)  {
    // ******************************************************************************************
    RxEnable();
    USART_ReadBlocking(DEMO_USART_MARCO_TEST, p_CharBufferTx, p_SizeOf);
    p_CharBufferTx[p_SizeOf] = 0;
}

void rsSerial::Rx       (char* p_CharBufferTx, size_t p_SizeOf)  {
    // ******************************************************************************************
    RxEnable();
    USART_ReadBlocking(DEMO_USART_MARCO_TEST, (uint8_t *) p_CharBufferTx, p_SizeOf);
    p_CharBufferTx[p_SizeOf] = 0;
}

char rsSerial::Rx    (void)  {
    uint8_t ch = 0;

    // ******************************************************************************************
    RxEnable();
    USART_ReadBlocking(DEMO_USART_MARCO_TEST, &ch, 1);

    return ch;
}


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_USART USART0
#define DEMO_USART_IRQHandler FLEXCOMM0_IRQHandler
#define DEMO_USART_IRQn FLEXCOMM0_IRQn
/* Task priorities. */
#define USART_NVIC_PRIO 5

uint8_t                     rsSerial::background_buffer[100];
usart_rtos_handle_t         rsSerial::handle;
struct _usart_handle        rsSerial::t_handle;
struct rtos_usart_config    rsSerial::usart_config= {
        .baudrate    = 115200,
        .parity      = kUSART_ParityDisabled,
        .stopbits    = kUSART_OneStopBit,
        .buffer      = rsSerial::background_buffer,
        .buffer_size = sizeof(rsSerial::background_buffer),
    };

void rsSerial::Init2             (void)      {
    // CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);    // define in board.h -> #define BOARD_DEBUG_UART_CLK_ATTACH kFRO12M_to_FLEXCOMM0
    // RESET_PeripheralReset(BOARD_DEBUG_UART_RST);     // define in board.h -> #define BOARD_DEBUG_UART_RST kFC0_RST_SHIFT_RSTn
    // DbgConsole_Init()
    BOARD_InitDebugConsole();

    // IOCON configuration
    BOARD_InitDEBUG_UARTPins();
    
}


void rsSerial::InitFreeRTOS             (void)      {
    usart_config.srcclk = BOARD_DEBUG_UART_CLK_FREQ;
    usart_config.base   = DEMO_USART;

    NVIC_SetPriority(DEMO_USART_IRQn, USART_NVIC_PRIO);

    if (0 > USART_RTOS_Init(&handle, &t_handle, &usart_config))
    {
        vTaskSuspend(NULL);
    }
    
}

void rsSerial::DeinitFreeRTOS             (void)      {
    USART_RTOS_Deinit(&handle);
    vTaskSuspend(NULL);

}


int rsSerial::TxFreeRTOS             (char* p_Buffer, uint8_t p_SizeBuffer)      {
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

int rsSerial::TxFreeRTOS             (const char* p_Buffer, uint8_t p_SizeBuffer)      {
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

int rsSerial::RxFreeRTOS             (char* p_Buffer, uint8_t p_SizeBuffer, uint8_t* p_ptrByteRxCounter)      {
    // .......................0.........1.........2.........3.........4.........5.........6.........7.........8.........9.........0.........1
    // .......................12345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
    const char *send_buffer_overrun = "\r\nRing buffer overrun!\r\n";

    int l_status;

    l_status = USART_RTOS_Receive(&handle, (uint8_t *) p_Buffer, p_SizeBuffer, (size_t*) p_ptrByteRxCounter);

    dgFormatDigit l_FormatNum (p_ptrByteRxCounter, 1, DIGIT_NUM_3, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum.vValueToString();
    USART_RTOS_Send(&handle, (uint8_t* ) l_FormatNum.m_StringValueField, DIGIT_NUM_3);

    dgFormatDigit l_FormatNum0 (&p_Buffer[0], 1, DIGIT_NUM_3, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum0.vValueToString();
    USART_RTOS_Send(&handle, (uint8_t* ) l_FormatNum0.m_StringValueField, DIGIT_NUM_3);

    dgFormatDigit l_FormatNum1 (&p_Buffer[1], 1, DIGIT_NUM_3, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum1.vValueToString();
    USART_RTOS_Send(&handle, (uint8_t* ) l_FormatNum1.m_StringValueField, DIGIT_NUM_3);

    dgFormatDigit l_FormatNum2 (&p_Buffer[2], 1, DIGIT_NUM_3, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum2.vValueToString();
    USART_RTOS_Send(&handle, (uint8_t* ) l_FormatNum2.m_StringValueField, DIGIT_NUM_3);

    dgFormatDigit l_FormatNum3 (&p_Buffer[3], 1, DIGIT_NUM_3, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum3.vValueToString();
    USART_RTOS_Send(&handle, (uint8_t* ) l_FormatNum3.m_StringValueField, DIGIT_NUM_3);

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


int rsSerial::RxFreeRTOS             (const char* p_Buffer, uint8_t p_SizeBuffer, uint8_t* p_ptrByteRxCounter)      {
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

} } }   // fw2::wrapper::resources

