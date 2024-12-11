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
## MODULE DIRECTORY Definitions
trace_execution()

set(LIBS_FW2_WRAPPER_RESOURCES_COMMON_SRC_DIR     ${LIBS_FW2_WRAPPER_RESOURCES_COMMON_DIR}/src                    )

set(TB_KR_INIT_DIR                   krInit/v_${TB_KR_INIT_VER}          )
set(TB_KR_INIT_CPP_DIR               ${TB_KR_INIT_DIR}                   )

set(CODE_FILES_FW2_WP_RS_LIB_RS
    ${LIBS_FW2_WRAPPER_RESOURCES_PLATFORM_SRC_DIR}/rs/${RS_SERIAL_CPP_DIR}/rsSerial.cpp
)

set(CODE_FILES_FW2_WP_RS_LIB_TB
    ${LIBS_FW2_WRAPPER_RESOURCES_COMMON_SRC_DIR}/tb/kr/${TB_KR_INIT_CPP_DIR}/krInitStaticTbl.cpp
)


## ******************************************************************
#________________________________________
## LIB FILES

set(CODE_CPP_FILES_FW2_WP_RS_LIB

    ${CODE_FILES_FW2_WP_RS_LIB_RS}
    ${CODE_FILES_FW2_WP_RS_LIB_TB}

)

end_include()
