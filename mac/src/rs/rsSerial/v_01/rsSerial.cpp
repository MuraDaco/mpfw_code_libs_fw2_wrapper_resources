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

#include <string>
#include <string.h>
#include <iostream>

// module include
#include <rsSerial.h>

// lib include
#include <dgFormatDigit.h>

using namespace std;
using namespace fw2::core::core;

namespace fw2 { namespace wrapper { namespace resources	{

constexpr char rsSerial::endl[];


bool rsSerial::CheckStatus            (int p_status)    {
    return (0 == p_status);
}


void rsSerial::Init    (void)       {
    
}

void rsSerial::Tx    (char p_Char)  {
    cout << p_Char;
}

void rsSerial::Tx    (uint8_t p_Num)  {
    dgFormatDigit l_FormatNum (&p_Num, 1, 3, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum.vValueToString();
    string l_str(l_FormatNum.m_StringValueField);
    cout << l_str;
}

void rsSerial::Tx    (uint16_t p_Num)  {
    dgFormatDigit l_FormatNum (&p_Num, 2, 5, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum.vValueToString();
    string l_str(l_FormatNum.m_StringValueField);
    cout << l_str;
}

void rsSerial::Tx    (uint32_t p_Num)  {
    dgFormatDigit l_FormatNum (&p_Num, 4, 6, dgFormatDigit::kEncoding_Decimal, 0, false);
    l_FormatNum.vValueToString();
    string l_str(l_FormatNum.m_StringValueField);
    cout << l_str;
}

// void rsSerial::Tx    (uint8_t* p_CharBufferTx)  {
//     string l_str(p_CharBufferTx);
//     cout << l_str;
// }

void rsSerial::Tx    (char* p_CharBufferTx)  {
    cout << endl << "rsSerial::Tx -> char* 1p" << endl;
    string l_str(p_CharBufferTx);
    cout << l_str;
}

void rsSerial::Tx   (char const *p_CharBufferTx)  {
    string l_str(p_CharBufferTx);
    cout << l_str;
}

void rsSerial::Tx   (char* p_CharBufferTx, __attribute__((unused)) uint8_t p_SizeBuf)  {
    cout << endl << "rsSerial::Tx -> char* 2p" << endl;

    string l_str(p_CharBufferTx);
    cout << l_str;
}

void rsSerial::Tx   (char const *p_CharBufferTx,  __attribute__((unused)) uint8_t p_SizeBuf)  {
    string l_str(p_CharBufferTx);
    cout << l_str;
}


void rsSerial::Rx    (char* p_CharBufferRx, size_t p_SizeOf)  {
    cout << endl << "rsSerial::Rx" << endl;

    string l_str;
    cin >> l_str;
    strncpy(p_CharBufferRx, l_str.c_str(), p_SizeOf-1);
    p_CharBufferRx[p_SizeOf-1] = 0;
}

char rsSerial::Rx    (void)  {
    char ch;
    cin >> ch;
    return ch;
}

void rsSerial::Init2             (void)      {
    
    
}



void rsSerial::InitFreeRTOS             (void)      {
    
}

void rsSerial::DeinitFreeRTOS             (void)      {

}


int rsSerial::TxFreeRTOS             (char* p_CharBufferTx, __attribute__((unused)) uint8_t p_SizeBuffer)      {
    int l_status = 0;
    cout << endl << "rsSerial::TxFreeRTOS -> char* 2p" << endl;

    string l_str(p_CharBufferTx);
    cout << l_str;

    return l_status;
}

int rsSerial::TxFreeRTOS             (const char* p_CharBufferTx, __attribute__((unused)) uint8_t p_SizeBuffer)      {
    int l_status = 0;
    cout << endl << "rsSerial::TxFreeRTOS -> const char* 2p" << endl;

    string l_str(p_CharBufferTx);
    cout << l_str;

    return l_status;
}

int rsSerial::RxFreeRTOS             (char* p_CharBufferRx, __attribute__((unused)) uint8_t p_SizeOf, __attribute__((unused)) uint8_t* p_ptrByteRxCounter)      {
    int l_status = 0;

    cout << endl << "rsSerial::RxFreeRTOS" << endl;

    string l_str;
    cin >> l_str;
    strncpy(p_CharBufferRx, l_str.c_str(), p_SizeOf-1);
    p_CharBufferRx[p_SizeOf-1] = 0;
    *p_ptrByteRxCounter = p_SizeOf-1;
    return l_status;
}

} } }   // fw2::wrapper::resources
