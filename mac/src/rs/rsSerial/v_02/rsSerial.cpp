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
 *  Created on: July, 26 2023
 *      Author: Marco Dau
 */

// module include
#include "rsSerial.h"

#include <uyProtocol.h>
using namespace fw2::core::core;

// sdk
#include <sdk_usart.h>

// lib include

using namespace std;


namespace fw2 { namespace wrapper { namespace resources	{

uint8_t rsSerial::buffer_default[100];

sdk_usart::callback_protocol_functions_t callback_protocol_functions = {
	.callback_get_packet_length = uyProtocol::get_packet_length,
	.callback_get_status = uyProtocol::get_status,
	.callback_check_packet_end = uyProtocol::check_packet_end,
	.callback_set_buffer_size = uyProtocol::set_buffer_size
};

bool rsSerial::CheckStatus            (int p_status)    {
    p_status = 0;
    return (true);
}


constexpr char rsSerial::endl[];

/*******************************************************************************
 * Definitions
 ******************************************************************************/


void rsSerial::Init             (void)      {

    sdk_usart::init(buffer_default, sizeof(buffer_default), &callback_protocol_functions);
}


// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx    (char p_Char)  {
    p_Char = 0;
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx       (uint8_t p_Num)  {
    p_Num = 0;
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx       (uint16_t p_Num)  {
    p_Num = 0;
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx       (uint32_t p_Num)  {
    p_Num = 0;
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx       (uint8_t* p_CharBufferTx)  {
    sdk_usart::send_blocking(p_CharBufferTx);
}


// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx       (char* p_CharBufferTx)  {

    sdk_usart::send_blocking(p_CharBufferTx);

}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx       (char const *p_CharBufferTx)  {
    // p_CharBufferTx is a string therefore buffer_size is not necessary

    sdk_usart::send_blocking(p_CharBufferTx);
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::Tx       (char* p_CharBufferTx, uint8_t p_SizeBuf)  {
    sdk_usart::send_blocking((uint8_t*) p_CharBufferTx, p_SizeBuf);
}


// ******************************************************************************************
// ******************************************************************************************
void rsSerial::RxStart       (uint8_t* p_CharBufferTx, size_t p_SizeOf, uint8_t p_packet_lenght     )  {
    sdk_usart::receive_non_blocking(p_CharBufferTx, p_SizeOf, p_packet_lenght);
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::RxStart       (uint8_t* p_CharBufferTx, size_t p_SizeOf)  {
    sdk_usart::receive_non_blocking(p_CharBufferTx, p_SizeOf);
}

// ******************************************************************************************
// ******************************************************************************************
void rsSerial::RxStart       (char* p_CharBufferTx, size_t p_SizeOf)  {
    sdk_usart::receive_non_blocking(p_CharBufferTx, p_SizeOf);
}

// ******************************************************************************************
// ******************************************************************************************
char rsSerial::RxStart    (void)  {
    uint8_t ch = 0;

    return ch;
}

// ******************************************************************************************
// ******************************************************************************************
char rsSerial::Rx    (void)  {
    for(;;){
        if(sdk_usart::receive_check()) break; else sdk_usart::receive_non_blocking(1);
    }

    return buffer_default[0];
}

// ******************************************************************************************
// ******************************************************************************************
bool rsSerial::RxCheck      (void)  {

    return sdk_usart::receive_check();

}


// ******************************************************************************************
// ******************************************************************************************
bool rsSerial::CheckService      (void)  {

    return sdk_usart::check_loop_end();

}

} } }   // fw2::wrapper::resources

