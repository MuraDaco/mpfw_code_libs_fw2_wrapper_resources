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
 * rsSerial.h
 *
 *  Created on: Nov, 30 2022
 *      Author: Marco Dau
 */

#ifndef RS_SERIAL_H_
#define RS_SERIAL_H_

// system lib
#include <cstdint>
#include <cstddef>

#include <drivers/freertos/fsl_usart_freertos.h>

using namespace std;

namespace fw2 { namespace wrapper { namespace resources	{

class rsSerial {

public:

    static constexpr char endl[3] = {'\r', '\n', 0};
    static void Init    (void                                       );

    static void Tx      (char     p_Char                                    );
    static void Tx      (uint8_t  p_Num                                     );
    static void Tx      (uint16_t p_Num                                     );
    static void Tx      (uint32_t p_Num                                     );
    static void Tx      (uint8_t* p_CharBufferTx                            );
    static void Tx      (char* p_CharBufferTx                               );
    static void Tx      (char* p_CharBufferTx,          uint8_t p_SizeBuf   );
    static void Tx      (char const *p_CharBufferTx                         );
    static void Tx      (uint8_t* p_CharBufferTx, uint8_t p_SizeBuf, uint8_t p_length);

    static char Rx          (void);
    static char RxStart     (void);
    static void RxStart     (uint8_t* p_CharBufferRx, size_t p_SizeOf);
    static void RxStart     (char*    p_CharBufferRx, size_t p_SizeOf);
    static void RxStart     (uint8_t* p_CharBufferRx, size_t p_SizeOf, uint8_t p_packet_lenght);
    static bool RxCheck     (void);

    static bool CheckStatus     (int p_status);

    static bool CheckService    (void);

private:

    static void InitPins_IOCON      (void);
    static void InitPins_GPIO       (void);
    static void TxEnable            (void);
    static void RxEnable            (void);
    static void TxCallback          (USART_Type *base, usart_handle_t *handle, status_t status, void *userData);


    typedef struct _usart_user_data {
        volatile uint8_t tx_state;
        volatile uint8_t rx_state;
    } usart_user_data_t;

    static usart_user_data_t usart_user_data;

    static usart_handle_t t_handle_demo_marco;

    static uint8_t background_buffer[];
    static usart_rtos_handle_t handle;
    static _usart_handle t_handle;
    static rtos_usart_config usart_config;

};

} } }   // fw2::wrapper::resources

#endif /* RS_SERIAL_H_ */
