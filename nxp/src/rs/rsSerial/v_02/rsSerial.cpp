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

#define DEMO_MARCO_USART_FLEXCOMM_ID            4                       // see Table 118 (UM10912 User Manual)          & SYSCON_Type struct (LPC54608.h)
#define DEMO_MARCO_USART_REG_PTR                USART4                  // see Table 422 & 429 (UM10912 User Manual)    & USART_Type struct  (LPC54608.h)
#define DEMO_MARCO_USART_CLOCK_ATTACH           kFRO12M_to_FLEXCOMM4    // see Table 118 & 157 (UM10912 User Manual)    & SYSCON_Type (LPC54608.h) - typedef enum _clock_attach_id (fsl_clock.h)
#define DEMO_MARCO_USART_CLK_FREQ               CLOCK_GetFlexCommClkFreq(DEMO_MARCO_USART_FLEXCOMM_ID)
#define DEMO_MARCO_USART_IRQn                   FLEXCOMM4_IRQn          // see Table B1-4 (**DDI0403E_e_armv7m_arm.pdf** *pg B1-525*) & typedef enum IRQn USART_Type struct  (LPC54608.h)
#define DEMO_MARCO_USART_NVIC_PRIO              5

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

usart_handle_t  rsSerial::t_handle_demo_marco;


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


void rsSerial::Init             (void)      {

    // - step 1
    //      - configuring pins: tx, rx, rs485-enable-tx, rs485-enable-rx
    InitPins_IOCON();
    InitPins_GPIO();

    // - step 2
    //      - configuring UART module

    // - step 2.1.a
    //      - load default configuration
    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUSART_ParityDisabled;
     * config.stopBitCount = kUSART_OneStopBit;
     * config.loopback = false;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    usart_config_t config;
    USART_GetDefaultConfig(&config);
    // - step 2.1.b
    //      - default configuration customization
    config.baudRate_Bps = 9600;
    config.enableTx     = true;
    config.enableRx     = true;


    // - step 2.2
    //      - enabling and configuring UART
    // - step 2.2.a 
    //      - attach 12 MHz clock to FLEXCOMM4 (USART)
    // . CLOCK_AttachClk
    //      . input
    //          - typedef enum _clock_attach_id / clock_attach_id_t in fsl_clock.h
    //      . register
    //          - SYSCON->MAINCLKSELA / SYSCON->FCLKSEL4 (see Table Table 118 & 157 [UM10912_LPC546xx_User_Manual.pdf])
    CLOCK_AttachClk(DEMO_MARCO_USART_CLOCK_ATTACH);

    // - step 2.2.b 
    //      - enable clock and reset/start UART
    //      - set UART registers based on configuration previously set
    // . CLOCK_EnableClock(s_flexcommClocks[idx]); 
    //      . input 
    //          - typedef enum _clock_ip_name" in fsl_clock.h
    //      . register
    //          - SYSCON->AHBCLKCTRLSET register (see Table Table 118 & 140 [UM10912_LPC546xx_User_Manual.pdf])
    // . RESET_PeripheralReset(s_flexcommResets[idx]);
    //      . input:
    //          - typedef enum _SYSCON_RSTn in fsl_reset.h
    //      . register
    //          - SYSCON->PRESETCTRL / SYSCON->PRESETCTRLSET / SYSCON->PRESETCTRLCLR (see Table Table 118 & 130 [UM10912_LPC546xx_User_Manual.pdf])
    USART_Init(DEMO_MARCO_USART_REG_PTR, &config, DEMO_MARCO_USART_CLK_FREQ  );

    // - step 2.2.c
    //      - enabling and configuring interrupt UART to use a no-blocking tx-rx procedures
    //
    //      - set priority interrupt 
    //  . NVIC_SetPriority
    //      . inputs
    //          - typedef enum IRQn / IRQn_Type in LPC54608.h
    //          - integer in the range of 0...7 (0 max priority / 7 min priority)
    //      . register
    //          - NVIC->IP regiter (see Table 89 & 104 [UM10912_LPC546xx_User_Manual.pdf])
    NVIC_SetPriority(DEMO_MARCO_USART_IRQn, DEMO_MARCO_USART_NVIC_PRIO);

    //      - set handler of the interrupt
    //          - hang the "USART_TransferHandleIRQ" function (fsl_uasart.c) to "FLEXCOMM0_DriverIRQHandler" function (fsl_flexcomm.c)
    //          - N.B.: "FLEXCOMM0_DriverIRQHandler" is called by "FLEXCOMM0_IRQHandler" function (startup_lpc54608.cpp)
    //          - see Table 88 in [UM10912_LPC546xx_User_Manual.pdf] & Table B1-4 in [DDI0403E_e_armv7m_arm.pdf]
    // . EnableIRQ
    //      . input
    //          - typedef enum IRQn / IRQn_Type in LPC54608.h
    //      . register
    //          - NVIC->ISER register (see Table 89 & 90 [UM10912_LPC546xx_User_Manual.pdf])
    USART_TransferCreateHandle(DEMO_MARCO_USART_REG_PTR, &t_handle_demo_marco, TxCallback, &usart_user_data);

    // send the first packet 
    Tx("rsSerial build.005 - krInitStatic -> UART init OK\r\nOOO");

}

rsSerial::usart_user_data_t rsSerial::usart_user_data;
uint8_t rsSerial::StatusEndTx;

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::TxCallback          (USART_Type *base, usart_handle_t *handle, status_t status, void *userData)  {
    ((usart_user_data_t *) userData)->tx_state = 1;
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::TxEnable    (void)  {
    usart_user_data.tx_state = 0;
    // tx enable
    GPIO_PinWrite(GPIO, APP_BOARD_MARCO_TEST_LED2_PORT, APP_BOARD_MARCO_TEST_LED2_PIN, 1);
    // rx disable
    GPIO_PinWrite(GPIO, APP_BOARD_MARCO_TEST_LED3_PORT, APP_BOARD_MARCO_TEST_LED3_PIN, 1);
    
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::RxEnable    (void)  {

    while(0 == usart_user_data.tx_state){}

    // tx disable
    GPIO_PinWrite(GPIO, APP_BOARD_MARCO_TEST_LED2_PORT, APP_BOARD_MARCO_TEST_LED2_PIN, 0);
    // rx enable
    GPIO_PinWrite(GPIO, APP_BOARD_MARCO_TEST_LED3_PORT, APP_BOARD_MARCO_TEST_LED3_PIN, 0);
}

#define DIGIT_NUM_3     3
#define DIGIT_NUM_5     5
#define DIGIT_NUM_6     6

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx    (char p_Char)  {
    p_Char = 0;
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx       (uint8_t p_Num)  {
    dgFormatDigit l_FormatNum (&p_Num, 1, DIGIT_NUM_3, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum.vValueToString();

    // ------------------------------
    TxEnable();
    usart_transfer_t xfer = { (uint8_t* ) l_FormatNum.m_StringValueField, DIGIT_NUM_3 };
    USART_TransferSendNonBlocking(DEMO_MARCO_USART_REG_PTR, &t_handle_demo_marco, &xfer);
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx       (uint16_t p_Num)  {
    dgFormatDigit l_FormatNum (&p_Num, 2, DIGIT_NUM_5, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum.vValueToString();

    // ------------------------------
    TxEnable();
    usart_transfer_t xfer = { (uint8_t* ) l_FormatNum.m_StringValueField, DIGIT_NUM_5 };
    USART_TransferSendNonBlocking(DEMO_MARCO_USART_REG_PTR, &t_handle_demo_marco, &xfer);
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx       (uint32_t p_Num)  {
    dgFormatDigit l_FormatNum (&p_Num, 4, DIGIT_NUM_6, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum.vValueToString();

    // ------------------------------
    TxEnable();
    usart_transfer_t xfer = { (uint8_t* ) l_FormatNum.m_StringValueField, DIGIT_NUM_6 };
    USART_TransferSendNonBlocking(DEMO_MARCO_USART_REG_PTR, &t_handle_demo_marco, &xfer);
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx       (uint8_t* p_CharBufferTx)  {
	uint8_t l_SizeBuf = 0;
    // determine the size of string
    for(uint8_t i=0;i<254; i++) {
        if(*(p_CharBufferTx+i) == 0) {
            l_SizeBuf = i;
            break;
        }
    }

    // ------------------------------
    TxEnable();
    usart_transfer_t xfer = { (uint8_t* ) p_CharBufferTx, l_SizeBuf };
    USART_TransferSendNonBlocking(DEMO_MARCO_USART_REG_PTR, &t_handle_demo_marco, &xfer);
}


// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx       (char* p_CharBufferTx)  {
	uint8_t l_SizeBuf = 0;
    // determine the size of string
    for(uint8_t i=0;i<254; i++) {
        if(*(p_CharBufferTx+i) == 0) {
            l_SizeBuf = i;
            break;
        }
    }

    // ------------------------------
    TxEnable();
    usart_transfer_t xfer = { (uint8_t* ) p_CharBufferTx, l_SizeBuf };
    USART_TransferSendNonBlocking(DEMO_MARCO_USART_REG_PTR, &t_handle_demo_marco, &xfer);
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx       (char const *p_CharBufferTx)  {
	uint8_t l_SizeBuf = 0;
    // determine the size of string
    for(uint8_t i=0;i<254; i++) {
        if(*(p_CharBufferTx+i) == 0) {
            l_SizeBuf = i;
            break;
        }
    }

    // ------------------------------
    TxEnable();
    usart_transfer_t xfer = { (uint8_t* ) p_CharBufferTx, l_SizeBuf };
    USART_TransferSendNonBlocking(DEMO_MARCO_USART_REG_PTR, &t_handle_demo_marco, &xfer);

    while(0 == usart_user_data.tx_state){}

}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx       (char* p_CharBufferTx, uint8_t p_SizeBuf)  {
    // ------------------------------
    TxEnable();
    usart_transfer_t xfer = { (uint8_t* ) p_CharBufferTx, p_SizeBuf };
    USART_TransferSendNonBlocking(DEMO_MARCO_USART_REG_PTR, &t_handle_demo_marco, &xfer);
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx       (char const *p_CharBufferTx,  uint8_t p_SizeBuf)  {
    // ------------------------------
    TxEnable();
    usart_transfer_t xfer = { (uint8_t* ) p_CharBufferTx, p_SizeBuf };
    USART_TransferSendNonBlocking(DEMO_MARCO_USART_REG_PTR, &t_handle_demo_marco, &xfer);
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Rx       (uint8_t* p_CharBufferTx, size_t p_SizeOf)  {
    // ------------------------------
    RxEnable();
    USART_ReadBlocking(DEMO_MARCO_USART_REG_PTR, p_CharBufferTx, p_SizeOf);
    p_CharBufferTx[p_SizeOf] = 0;
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Rx       (char* p_CharBufferTx, size_t p_SizeOf)  {
    // ------------------------------
    RxEnable();
    USART_ReadBlocking(DEMO_MARCO_USART_REG_PTR, (uint8_t *) p_CharBufferTx, p_SizeOf);
    p_CharBufferTx[p_SizeOf] = 0;
}

// ******************************************************************************************
// ******************************************************************************************
char rsSerial::Rx    (void)  {
    uint8_t ch = 0;

    // ------------------------------
    RxEnable();
    USART_ReadBlocking(DEMO_MARCO_USART_REG_PTR, &ch, 1);

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
        .baudrate    = 9600, // 115200,
        .parity      = kUSART_ParityDisabled,
        .stopbits    = kUSART_OneStopBit,
        .buffer      = rsSerial::background_buffer,
        .buffer_size = sizeof(rsSerial::background_buffer),
    };

void rsSerial::Init2             (void)      {
	CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

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

