#   *******************************************************************************
#   
#   mpfw / fw2 - Multi Platform FirmWare FrameWork 
#       library that contains the wrapper code to manage platform resources
#   Copyright (C) (2023) Marco Dau
#   
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU Affero General Public License as published
#   by the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#   
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU Affero General Public License for more details.
#   
#   You should have received a copy of the GNU Affero General Public License
#   along with this program.  If not, see <https://www.gnu.org/licenses/>.
#   
#   You can contact me by the following email address
#   marco <d o t> ing <d o t> dau <a t> gmail <d o t> com
#   
#   *******************************************************************************

## ******************************************************************
## __________________________________________________________________
## INCLUDE DIRECTORIES Definitions

include(${CODE_MAIN_CMAKE_LIBS_FW2_WRAPPER_RESOURCES_VER_DEF_DIR}/set_class_src_ver.cmake)

set(LIBS_FW2_WRAPPER_RESOURCES_PLATFORM_DIR         ${LIBS_FW2_WRAPPER_RESOURCES_DIR}/${WP_PLATFORM_STR}              )
set(LIBS_FW2_WRAPPER_RESOURCES_PLATFORM_SRC_DIR     ${LIBS_FW2_WRAPPER_RESOURCES_PLATFORM_DIR}/src                    )

set(RS_SERIAL_DIR                    rsSerial/v_${RS_SERIAL_VER}         )
set(RS_SERIAL_CPP_DIR                ${RS_SERIAL_DIR}                    )


set(CODE_DIR_LIB_FW2_WP_RS_INCLUDE
    ${SDK_PLATFORM_UART_CONF_DIR}
    ${LIBS_FW2_WRAPPER_RESOURCES_PLATFORM_SRC_DIR}/rs/${RS_SERIAL_DIR}
    ${LIBS_FW2_WRAPPER_RESOURCES_PLATFORM_SRC_DIR}/rs/${RS_SERIAL_DIR}/sdk_uart_pub
)
