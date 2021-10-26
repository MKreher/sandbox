cmake_minimum_required(VERSION 3.13.1)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE ZDebug)
endif()

if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/configuration/${BOARD}")
  message(FATAL_ERROR
          "Board ${BOARD} is not supported.\n"
          "Please make sure board specific configuration files are added to "
          "${CMAKE_CURRENT_SOURCE_DIR}/configuration/${BOARD}")
endif()

if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/configuration/${BOARD}/app_${CMAKE_BUILD_TYPE}.conf")
  message(FATAL_ERROR
          "Configuration file for build type ${CMAKE_BUILD_TYPE} is missing.\n"
          "Please add file ${CMAKE_CURRENT_SOURCE_DIR}/configuration/${BOARD}/app_${CMAKE_BUILD_TYPE}.conf")
endif()

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/configuration/${BOARD}/dts.overlay")
  set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/configuration/${BOARD}/dts.overlay")
endif()

# For the sake of demonstration, we add the driver directory as a zephyr module
# by hand. If your driver is a project that's managed by west, you can remove this line.
list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/barcode_module
  )
  
# Define configuration files.
set(CONF_FILE "configuration/${BOARD}/app_${CMAKE_BUILD_TYPE}.conf")

################################################################################

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(TicTac)

################################################################################

# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)
# NORDIC SDK APP END

# Include source directory
target_include_directories(app PRIVATE src)

zephyr_include_directories(
  configuration/common
  configuration/${BOARD}
  )

# Application directories
#add_subdirectory(src/blablub)
