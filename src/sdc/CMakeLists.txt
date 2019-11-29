# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
target_sources(${LRS_TARGET}
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/sdc-options.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-timestamp.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-private.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-motion.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-rolling-shutter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-device.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-color.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-active.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-factory.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/advanced_mode/sdc-rs_advanced_mode.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/advanced_mode/sdc-presets.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/advanced_mode/sdc-advanced_mode.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-factory.h"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-device.h"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-options.h"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-private.h"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-timestamp.h"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-active.h"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-motion.h"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-rolling-shutter.h"
        "${CMAKE_CURRENT_LIST_DIR}/sdc-color.h"
)
