// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

#include "sdc-device.h"

namespace librealsense
{
    class sdc_rolling_shutter : public virtual sdc_device
    {
    public:
        sdc_rolling_shutter(std::shared_ptr<context> ctx,
                            const platform::backend_device_group& group);
    };
}
