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

#include "sdc-active.h"
#include "sdc-private.h"
#include "sdc-options.h"
#include "sdc-timestamp.h"

namespace librealsense
{
    sdc_active::sdc_active(std::shared_ptr<context> ctx,
                           const platform::backend_device_group& group)
        : device(ctx, group), sdc_device(ctx, group)
    {
        using namespace sdc;

        auto pid = group.uvc_devices.front().pid;
        if (pid == SDC30_PID)
        {
            auto& depth_ep = get_depth_sensor();
            
			if (_fw_version >= firmware_version("20.11.7.0"))
			{
				auto emitter_enabled = std::make_shared<sdc_emitter_option>(depth_ep);
				depth_ep.register_option(RS2_OPTION_EMITTER_ENABLED, emitter_enabled);

				auto laser_power = std::make_shared<uvc_xu_option<uint16_t>>(depth_ep,
					depth_xu,
					SDC_LASER_POWER,
					"Manual laser power in mw. applicable only when laser power mode is set to Manual");
				depth_ep.register_option(RS2_OPTION_LASER_POWER,
					std::make_shared<auto_disabling_control>(
					laser_power,
					emitter_enabled,
					std::vector<float>{0.f, 2.f}, 1.f));

				depth_ep.register_option(RS2_OPTION_PROJECTOR_TEMPERATURE,
					std::make_shared<sdc_asic_and_projector_temperature_options>(depth_ep,
					RS2_OPTION_PROJECTOR_TEMPERATURE));
			}
        }
    }
}
