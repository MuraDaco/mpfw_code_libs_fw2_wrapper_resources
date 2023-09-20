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

/*

    - doc
        - micro
            - Reference Manual (register description)
                - rm0410-stm32f76xxx-and-stm32f77xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
            - Device Datasheet (alternate function for each pin)
                - stm32f769ni.pdf
            - Programming manual (asm instruction)                
                - pm0253-stm32f7-series-and-stm32h7-series-cortexm7-processor-programming-manual-stmicroelectronics.pdf
        - board
            - schematics
                - mb1225-f769i-b02_schematic.pdf
 */


// module include
#include <rsSerial.h>

// sdk
#include "sdk_uart_conf.h"
#include "rsSerial_sdk_uart.h"
#include "sdk_uart_prv/sdk_uart_prv.h"

// lib include
#include <dgFormatDigit.h>

using namespace std;
using namespace fw2::core::core;

namespace fw2 { namespace wrapper { namespace resources	{


constexpr char rsSerial::endl[];

// ###################################################################################################################################################################################################
// ##### start NEW
// ###################################################################################################################################################################################################



// rs485 tx/rx enable pin
#define RS485_TX_RX_ENABLE_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOJ_CLK_ENABLE()

#define RS485_TX_ENABLE_GPIO_PORT                   ((GPIO_TypeDef*)GPIOJ)
#define RS485_TX_ENABLE_GPIO_PIN                    ((uint32_t)GPIO_PIN_0) // CN13 - D4 (la numerazione parte da D0 e va in su)

#define RS485_RX_ENABLE_GPIO_PORT                   ((GPIO_TypeDef*)GPIOJ)
#define RS485_RX_ENABLE_GPIO_PIN                    ((uint32_t)GPIO_PIN_1) // CN13 - D2


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
// #define TRANSMITTER_BOARD
#define UART_DEMO_MARCO



// ###################################################################################################################################################################################################
// ##### end NEW
// ###################################################################################################################################################################################################

void rsSerial::TxEnable   (void)  {
    // enable rs485-tx
    HAL_GPIO_WritePin(RS485_TX_ENABLE_GPIO_PORT, RS485_TX_ENABLE_GPIO_PIN, GPIO_PIN_SET);
    // disable rs485-rx
    HAL_GPIO_WritePin(RS485_RX_ENABLE_GPIO_PORT, RS485_RX_ENABLE_GPIO_PIN, GPIO_PIN_SET);

    /* Reset uart process flag */
    UartReady = RESET;
}

void rsSerial::RxEnable   (void)  {
    // disnable rs485-tx
    HAL_GPIO_WritePin(RS485_TX_ENABLE_GPIO_PORT, RS485_TX_ENABLE_GPIO_PIN, GPIO_PIN_RESET);
    // enable rs485-rx
    HAL_GPIO_WritePin(RS485_RX_ENABLE_GPIO_PORT, RS485_RX_ENABLE_GPIO_PIN, GPIO_PIN_RESET);

    /* Reset uart process flag */
    UartReady = RESET;
}

void rsSerial::WaitingTransferEnd   (void)  {
    // Wait for the end of the transfer
    while (UartReady != SET)
    {
    }
}

bool rsSerial::CheckStatus            (int p_status)    {
    return (0 == p_status);
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
    
    int l_status = HAL_UART_Transmit_DMA(&UartHandle, (uint8_t* ) l_FormatNum.m_StringValueField, DIGIT_NUM_3);
    if(HAL_OK != l_status)
    {
        Error_Handler();
    }

    WaitingTransferEnd();
}

void rsSerial::Tx       (uint16_t p_Num)  {
    dgFormatDigit l_FormatNum (&p_Num, 2, DIGIT_NUM_5, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum.vValueToString();

    // ******************************************************************************************
    TxEnable();

    int l_status = HAL_UART_Transmit_DMA(&UartHandle, (uint8_t* ) l_FormatNum.m_StringValueField, DIGIT_NUM_5);
    if(HAL_OK != l_status)
    {
        Error_Handler();
    }

    WaitingTransferEnd();
}

void rsSerial::Tx       (uint32_t p_Num)  {
    dgFormatDigit l_FormatNum (&p_Num, 4, DIGIT_NUM_6, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum.vValueToString();

    // ******************************************************************************************
    TxEnable();

    int l_status = HAL_UART_Transmit_DMA(&UartHandle, (uint8_t* ) l_FormatNum.m_StringValueField, DIGIT_NUM_6);
    if(HAL_OK != l_status)
    {
        Error_Handler();
    }

    WaitingTransferEnd();
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

    int l_status = HAL_UART_Transmit_DMA(&UartHandle, p_CharBufferTx, l_SizeBuf);
    if(HAL_OK != l_status)
    {
        Error_Handler();
    }

    WaitingTransferEnd();
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

    int l_status = HAL_UART_Transmit_DMA(&UartHandle, (uint8_t* ) p_CharBufferTx, l_SizeBuf);
    if(HAL_OK != l_status)
    {
        Error_Handler();
    }

    WaitingTransferEnd();
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

    int l_status = HAL_UART_Transmit_DMA(&UartHandle, (uint8_t* const) p_CharBufferTx, l_SizeBuf);
    if(HAL_OK != l_status)
    {
        Error_Handler();
    }

    WaitingTransferEnd();
}

void rsSerial::Tx       (char* p_CharBufferTx, uint8_t p_SizeBuf)  {
    // ******************************************************************************************
    TxEnable();

    int l_status = HAL_UART_Transmit_DMA(&UartHandle, (uint8_t *) p_CharBufferTx, p_SizeBuf);
    if(HAL_OK != l_status)
    {
        Error_Handler();
    }

    WaitingTransferEnd();
}

void rsSerial::Tx       (uint8_t* p_CharBufferTx, uint8_t p_SizeBuf, uint8_t p_length)  {
    if(p_SizeBuf < p_length) return;
    // ******************************************************************************************
    TxEnable();

    int l_status = HAL_UART_Transmit_DMA(&UartHandle, p_CharBufferTx, p_length);
    if(HAL_OK != l_status)
    {
        Error_Handler();
    }

    WaitingTransferEnd();
}


#define SDK_USART_TX_STATUS__FREE               0 // 
#define SDK_USART_TX_STATUS__RUNNING	        1 // kUSART_TxBusy
#define SDK_USART_TX_STATUS__SEND_COMPLETED     2 // kUSART_TxIdle
#define SDK_USART_RX_STATUS__FREE               0
#define SDK_USART_RX_STATUS__RUNNING            3 // kUSART_RxBusy
#define SDK_USART_RX_STATUS__READY_TO_READ      4 // kUSART_RxIdle

uint8_t rsSerial::status;


void rsSerial::RxStart       (uint8_t* p_CharBufferTx, size_t p_SizeOf, uint8_t p_packet_length)  {
    // ******************************************************************************************
    RxEnable();

	int l_status = HAL_UART_Receive_DMA(&UartHandle, p_CharBufferTx, p_SizeOf);
    if(HAL_OK != l_status)
    {
        Error_Handler();
    }

    WaitingTransferEnd();

    p_CharBufferTx[p_SizeOf] = 0;
    status = SDK_USART_RX_STATUS__READY_TO_READ;
}

void rsSerial::RxStart       (uint8_t* p_CharBufferTx, size_t p_SizeOf)  {
    // ******************************************************************************************
    RxEnable();

	int l_status = HAL_UART_Receive_DMA(&UartHandle, p_CharBufferTx, p_SizeOf);
    if(HAL_OK != l_status)
    {
        Error_Handler();
    }

    WaitingTransferEnd();

    p_CharBufferTx[p_SizeOf] = 0;
    status = SDK_USART_RX_STATUS__READY_TO_READ;
}

void rsSerial::RxStart       (char* p_CharBufferTx, size_t p_SizeOf)  {

    // ******************************************************************************************
    RxEnable();

	int l_status = HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)p_CharBufferTx, p_SizeOf);
    if(HAL_OK != l_status)
    {
        Error_Handler();
    }

    WaitingTransferEnd();
    status = SDK_USART_RX_STATUS__READY_TO_READ;

}


char rsSerial::RxStart    (void)  {
    uint8_t ch = 0;

    // ******************************************************************************************
    RxEnable();

	int l_status = HAL_UART_Receive_DMA(&UartHandle, &ch, 1);
    if(HAL_OK != l_status)
    {
        Error_Handler();
    }

    WaitingTransferEnd();

    return ch;
}

char rsSerial::Rx    (void)  {
    uint8_t ch = 0;

    // ******************************************************************************************
    RxEnable();

	int l_status = HAL_UART_Receive_DMA(&UartHandle, &ch, 1);
    if(HAL_OK != l_status)
    {
        Error_Handler();
    }

    WaitingTransferEnd();

    return ch;
}

bool rsSerial::RxCheck     (void)   {
    status = SDK_USART_RX_STATUS__FREE;

    return (SDK_USART_RX_STATUS__READY_TO_READ == status);
}

bool rsSerial::CheckService    (void)  {
    return false;
}


void rsSerial::InitUART    (void)
{
    // *******************************************************************************
    // *******************************************************************************
    // configure rs485 tx/rx enable pin

    // 1. enable clock
    RS485_TX_RX_ENABLE_GPIO_CLK_ENABLE();

    // 2. GPIO pin configuration
    GPIO_InitTypeDef  gpio_init_structure;

    /* Configure the GPIO_LED pin */
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;

    gpio_init_structure.Pin   = RS485_TX_ENABLE_GPIO_PIN;
    HAL_GPIO_Init(RS485_TX_ENABLE_GPIO_PORT, &gpio_init_structure);

    gpio_init_structure.Pin   = RS485_RX_ENABLE_GPIO_PIN;
    HAL_GPIO_Init(RS485_RX_ENABLE_GPIO_PORT, &gpio_init_structure);

    // HAL_GPIO_WritePin(RS485_TX_RX_ENABLE_GPIO_PORT, RS485_TX_RX_ENABLE_GPIO_PIN, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(RS485_TX_RX_ENABLE_GPIO_PORT, RS485_TX_RX_ENABLE_GPIO_PIN, GPIO_PIN_RESET);

    // *******************************************************************************
    // *******************************************************************************


    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART configured as follows:
        - Word Length = 8 Bits
        - Stop Bit = One Stop bit
        - Parity = None
        - BaudRate = 9600 baud
        - Hardware flow control disabled (RTS and CTS signals) */
    UartHandle.Instance        = USARTx;

    UartHandle.Init.BaudRate   = 9600;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits   = UART_STOPBITS_1;
    UartHandle.Init.Parity     = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode       = UART_MODE_TX_RX;
    if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
    {
      Error_Handler();
    }  
    if(HAL_UART_Init(&UartHandle) != HAL_OK)
    {
      Error_Handler();
    }

}


void rsSerial::Init             (void)      {
    
    // initialization / configuration of MCU register
    
    /* Configure the MPU attributes */
    MPU_Config();
    
    /* Enable the CPU Cache */
    CPU_CACHE_Enable();
    
    /* STM32F7xx_hal.h
        STM32F7xx HAL library initialization:
        - Configure the Flash ART accelerator
        - Systick timer is configured by default as source of time base, but user 
          can eventually implement his proper time base source (a general purpose 
          timer for example or other time source), keeping in mind that Time base 
          duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
          handled in milliseconds basis.
        - Set NVIC Group Priority to 4
        - Low Level Initialization
       */
    HAL_Init();
    
    /* Configure the system clock to 216 MHz */
    SystemClock_Config();

    /* Configure LED1 & LED2 */
    BSP_LED_Init(LED1);
    BSP_LED_Init(LED2);

    InitUART();
}

} } }   // fw2::wrapper::resources

