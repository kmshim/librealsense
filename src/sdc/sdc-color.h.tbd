// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

#include "sdc-device.h"
#include "stream.h"

namespace librealsense
{
    class sdc_color : public virtual sdc_device
    {
    public:
        std::shared_ptr<uvc_sensor> create_color_device(std::shared_ptr<context> ctx,
                                                        const std::vector<platform::uvc_device_info>& all_device_infos);

        sdc_color(std::shared_ptr<context> ctx,
                  const platform::backend_device_group& group);

    protected:
        std::shared_ptr<stream_interface> _color_stream;

    private:
        friend class sdc_color_sensor;

        uint8_t _color_device_idx = -1;


        lazy<std::vector<uint8_t>> _color_calib_table_raw;
        std::shared_ptr<lazy<rs2_extrinsics>> _color_extrinsic;
    };

    class sdc_color_sensor : public uvc_sensor,
                             public video_sensor_interface,
                             public roi_sensor_base
    {
    public:
        explicit sdc_color_sensor(sdc_color* owner,
            std::shared_ptr<platform::uvc_device> uvc_device,
            std::unique_ptr<frame_timestamp_reader> timestamp_reader)
            : uvc_sensor("RGB Camera", uvc_device, move(timestamp_reader), owner), _owner(owner)
        {}

        rs2_intrinsics get_intrinsics(const stream_profile& profile) const override;
        stream_profiles init_stream_profiles() override;
        processing_blocks get_recommended_processing_blocks() const override;

    private:
        const sdc_color* _owner;
    };

}
