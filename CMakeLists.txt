cmake_minimum_required(VERSION 2.8.3)
project(ros_powerlink)

SET(OPLK_BASE_DIR /home/jan/openPOWERLINK_V2.6.0)
SET(CFG_BUILD_KERNEL_STACK "Linux Kernel Module")
SET(CMAKE_BUILD_TYPE "Release")
SET(CFG_STORE_RESTORE)

INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/options.cmake)
INCLUDE(linux.cmake)

SET(OBJDICT CiA401_CN)
FIND_OPLK_LIBRARY("cn")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
)

add_message_files(
    FILES
    openPowerlinkMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

# Set generic sources and include directories for this project
SET(SOURCES
    ${SOURCE_DIR}/main.cpp
    ${SOURCE_DIR}/app.cpp
    ${SOURCE_DIR}/event.c
    ${COMMON_SOURCE_DIR}/eventlog/eventlog.c
    ${COMMON_SOURCE_DIR}/eventlog/eventlogstring.c
    ${CONTRIB_SOURCE_DIR}/console/printlog.c
    ${COMMON_SOURCE_DIR}/obdcreate/obdcreate.c
    )

INCLUDE_DIRECTORIES(
    ${OBJDICT_DIR}/${OBJDICT}
    ${CONTRIB_SOURCE_DIR}
    )

# PResChaining is not supported by non-openMAC targets
#ADD_DEFINITIONS(-DCONFIG_DLL_PRES_CHAINING_CN)
ADD_DEFINITIONS(-DNMT_MAX_NODE_ID=0)
ADD_DEFINITIONS(-DCONFIG_INCLUDE_PDO)
ADD_DEFINITIONS(-DCONFIG_INCLUDE_SDO_ASND)
ADD_DEFINITIONS(-DCONFIG_INCLUDE_MASND)

IF(CFG_STORE_RESTORE)
    ADD_DEFINITIONS(-DCONFIG_OBD_USE_STORE_RESTORE)
ENDIF(CFG_STORE_RESTORE)


################################################################################
# Setup compile definitions depending on configuration

# PCAP is used for directlink of userspace daemon
IF (CFG_KERNEL_STACK_DIRECTLINK OR CFG_KERNEL_STACK_USERSPACE_DAEMON)
    ADD_DEFINITIONS(-DCONFIG_USE_PCAP)
    SET (DEMO_SOURCES ${DEMO_SOURCES} ${COMMON_SOURCE_DIR}/pcap/pcap-console.c)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK OR CFG_KERNEL_STACK_USERSPACE_DAEMON)

# Setup compile definitions depending on configuration
IF (CFG_KERNEL_STACK_DIRECTLINK)
    UNSET (CFG_DEMO_CN_CONSOLE_USE_SYNCTHREAD CACHE)
    ADD_DEFINITIONS(-DCONFIG_KERNELSTACK_DIRECTLINK)
ELSE (CFG_KERNEL_STACK_DIRECTLINK)
    OPTION (CFG_DEMO_CN_CONSOLE_USE_SYNCTHREAD "Create separate thread for syncronous data exchange" ON)
    IF (CFG_DEMO_CN_CONSOLE_USE_SYNCTHREAD)
        ADD_DEFINITIONS(-DCONFIG_USE_SYNCTHREAD)
    ENDIF (CFG_DEMO_CN_CONSOLE_USE_SYNCTHREAD)
ENDIF (CFG_KERNEL_STACK_DIRECTLINK)

################################################################################
# Set the executable

ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCES} ${ARCH_SOURCES})
SET_PROPERTY(TARGET ${PROJECT_NAME}
             PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG;DEF_DEBUG_LVL=${CFG_DEBUG_LVL})

################################################################################
# Libraries to link

OPLK_LINK_LIBRARIES(${PROJECT_NAME})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${ARCH_LIBRARIES}  ${catkin_LIBRARIES})

################################################################################
# Installation rules

INSTALL(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${PROJECT_NAME})
