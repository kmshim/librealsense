// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved.

#include <mutex>
#include <chrono>
#include <vector>
#include <iterator>
#include <cstddef>

#include "device.h"
#include "context.h"
#include "image.h"
#include "metadata-parser.h"

#include "sdc-rolling-shutter.h"
#include "sdc-private.h"
#include "sdc-options.h"
#include "sdc-timestamp.h"

namespace librealsense
{
    sdc_rolling_shutter::sdc_rolling_shutter(std::shared_ptr<context> ctx,
                                             const platform::backend_device_group& group)
        : device(ctx, group), sdc_device(ctx, group)
    {
        using namespace sdc;

        auto pid = group.uvc_devices.front().pid;
        if ((_fw_version >= firmware_version("19.11.7.0")) && (pid == SDC30_PID))
        {
            get_depth_sensor().register_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,
                std::make_shared<uvc_xu_option<uint8_t>>(get_depth_sensor(),
                                                         depth_xu,
                                                         SDC_ENABLE_AUTO_WHITE_BALANCE,
                                                         "Enable Auto White Balance"));

            // RS400 rolling-shutter Skus allow to get low-quality color image from the same viewport as the depth
            get_depth_sensor().register_pixel_format(pf_uyvyl);
            get_depth_sensor().register_pixel_format(pf_rgb888);
            get_depth_sensor().register_pixel_format(pf_w10);
        }

        get_depth_sensor().unregister_option(RS2_OPTION_EMITTER_ON_OFF);

        if ((_fw_version >= firmware_version("0.0.0.0") &&
             _fw_version < firmware_version("19.11.7.0")))
        {
            get_depth_sensor().register_option(RS2_OPTION_INTER_CAM_SYNC_MODE,
                std::make_shared<sdc_external_sync_mode>(*_hw_monitor));
        }
    }
}
