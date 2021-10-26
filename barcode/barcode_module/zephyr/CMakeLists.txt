# SPDX-License-Identifier: Apache-2.0

if(CONFIG_E3000H_BARCODE_DRIVER)
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    e3000h.c
    )
endif()
