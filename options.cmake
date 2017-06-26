################################################################################
#
# Generic CMake options openPOWERLINK demo applications
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holders nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
################################################################################

MESSAGE(STATUS "CMAKE_SYSTEM_NAME is ${CMAKE_SYSTEM_NAME}")
MESSAGE(STATUS "CMAKE_SYSTEM_PROCESSOR is ${CMAKE_SYSTEM_PROCESSOR}")

################################################################################
# Set global directories
################################################################################
SET(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
SET(COMMON_SOURCE_DIR ${OPLK_BASE_DIR}/apps/common/src)
SET(OPENCONFIG_PROJ_DIR ${OPLK_BASE_DIR}/apps/common/openCONFIGURATOR_projects)
SET(CONTRIB_SOURCE_DIR ${OPLK_BASE_DIR}/contrib)
SET(OPLK_INCLUDE_DIR ${OPLK_BASE_DIR}/stack/include)
SET(TOOLS_DIR ${OPLK_BASE_DIR}/tools)
SET(BOARDS_DIR ${OPLK_BASE_DIR}/hardware/boards)
SET(OBJDICT_DIR ${OPLK_BASE_DIR}/apps/common/objdicts)

################################################################################
# Include CMake Modules
################################################################################
SET(CMAKE_MODULE_PATH ${OPLK_BASE_DIR}/apps/common/cmake ${CMAKE_MODULE_PATH})

# include standard cmake modules
INCLUDE(CMakeDependentOption)

# include project specific modules
INCLUDE(findoplklib)
INCLUDE(linkoplklib)

################################################################################
# Set options
################################################################################

STRING(TOLOWER "${CMAKE_SYSTEM_NAME}" SYSTEM_NAME_DIR)
STRING(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SYSTEM_PROCESSOR_DIR)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    ${OPLK_BASE_DIR}/bin/${SYSTEM_NAME_DIR}/${SYSTEM_PROCESSOR_DIR} CACHE PATH "openPOWERLINK apps install prefix" FORCE
    )
ENDIF()

SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE INTERNAL "Available Build Configurations" FORCE)

SET(CFG_DEBUG_LVL "0xC0000000L" CACHE STRING "Debug Level for debug output")

# set global include directories
INCLUDE_DIRECTORIES (
    ${OPLK_INCLUDE_DIR}
    ${CONTRIB_SOURCE_DIR}
    ${COMMON_SOURCE_DIR}
    ${OBJDICT_DIR}
)

INCLUDE(configure-linux)
