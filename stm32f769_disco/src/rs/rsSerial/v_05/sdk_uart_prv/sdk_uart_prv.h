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
/**
  ******************************************************************************
  * @file    UART/UART_TwoBoards_ComDMA/Inc/main.h 
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SDK_UART_PRV_H
#define SDK_UART_PRV_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f769i_discovery.h"

#ifndef RS_SERIAL_GLOBAL_DEFINE
  #define RS_SERIAL_EXTERN extern
  // #warning "extern set"
#else
  #define RS_SERIAL_EXTERN
  // #warning "extern NOT set"
#endif

/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE
  
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
extern "C" {
#endif



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RS_SERIAL_EXTERN __IO ITStatus UartReady; // = RESET;
RS_SERIAL_EXTERN __IO uint32_t UserButtonStatus; // = 0;  /* set to 1 after User Button interrupt  */

void MPU_Config(void);
void SystemClock_Config(void);
void Error_Handler(void);
void CPU_CACHE_Enable(void);


#ifdef __cplusplus
}
#endif

#endif /* SDK_UART_PRV_H */

