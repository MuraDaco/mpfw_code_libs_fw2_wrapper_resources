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
trace_execution_cmakelists()
enable_language( ASM)

# step 1 - 1st level path definition
set(CODE_DIR        ../../../../../../.. )
include(${CODE_DIR}/${CMAKE_MODULE_VER_RPATH}/set_apps_libs_dirs.cmake )
set(LIB_NAME        fw2_wp_rs_lib )

# step 2 - 2nd level path definition - include path definition
include(include_dependencies.cmake )

# step 3 - set cpp sources files
include(set_cpp_files.cmake )
include(${LIBS_FW2_WRAPPER_RESOURCES_PLATFORM_VER_DIR_PRJ}/set_cpp_files.cmake )

# step 6 - library creation
add_library(${LIB_NAME}
    ${CODE_CPP_FILES_FW2_WP_RS_LIB}
    ${CODE_CPP_FILES_FW2_WP_RS_LIB_PLATFORM}
)

# step 7 - executable file name set
set(EXECUTABLE ${LIB_NAME} )

# step 8 - compile option set
include(${LIBS_SDK_PLATFORM_CMAKE_PUBLIC_DIR}/toolchain_compile_option.cmake )

# step 9 - include directories set
target_include_directories(${LIB_NAME} PUBLIC 
    ${CODE_DIR_LIB_SDK_PLATFORM_INCLUDE}
    ${CODE_DIR_LIB_FW2_INCLUDE}
    ${CODE_DIR_LIB_FW2_WP_RS_INCLUDE}
    ${CODE_DIR_LIB_FW2_WP_KR_INCLUDE}
)


# step 10 - compile definition set
target_compile_definitions(${LIB_NAME} PRIVATE FW2_WP_RS_LIB_VERSION=1.1 )

message(STATUS "---- end   section of <module_libs_fw2_wrapper_resources> module " )
