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
 * rsSerialConnection.cpp
 *
 *  Created on: July, 26 2023
 *      Author: Marco Dau
 */

// module include

// libs
#include <krThread.h>
using namespace fw2::wrapper::core;

#include <uyProtocol.h>
using namespace fw2::core::core;

// sdk
#include <fsl_usart.h>



callback_protocol_functions_t g_callback_protocol_functions = {

    .tx_buffer_management       = uyProtocol::tx_buffer_management,
    .rx_buffer_management       = uyProtocol::rx_buffer_management,
    
    .tx_status_get__running     = uyProtocol::status_get_tx__running,
    .tx_status_get__completed   = uyProtocol::status_get_tx__completed,
    .rx_status_get__running     = uyProtocol::status_get_rx__running,
    .rx_status_get__completed   = uyProtocol::status_get_rx__completed,

    .tx_status_set__free        = uyProtocol::status_set_tx__free,
    .rx_status_set__free        = uyProtocol::status_set_rx__free

};


uyProtocol::callback_sdk_functions_t uyProtocol::g_callback_sdk_functions = {

    .reg_write  = USART_reg_write,
    .reg_read   = USART_reg_read,

};

uyProtocol::callback_sleep_t uyProtocol::g_sleep_ms = wp_sleep_for;