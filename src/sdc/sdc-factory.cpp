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

#include "sdc-factory.h"
#include "sdc-private.h"
#include "sdc-options.h"
#include "sdc-timestamp.h"
#include "sdc-rolling-shutter.h"
#include "sdc-active.h"
//#include "sdc-color.h"
//#include "sdc-motion.h"
#include "sync.h"

namespace librealsense
{
	class sdc30_device : public sdc_active
	{
		public:
		sdc30_device(std::shared_ptr<context> ctx,
			const platform::backend_device_group group,
			bool register_device_notifications)
			: device(ctx, group, register_device_notifications),
			sdc_device(ctx, group),
			sdc_active(ctx, group){}

		std::shared_ptr<matcher> create_matcher(const frame_holder& frame) const override;

		std::vector<tagged_profile> get_profiles_tags() const override
		{
			std::vector<tagged_profile> tags;
			auto usb_spec = get_usb_spec();
			if (usb_spec >= platform::usb3_type || usb_spec == platform::usb_undefined)
			{
				tags.push_back({ RS2_STREAM_DEPTH, -1, 320, 240, RS2_FORMAT_Z16, 5, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
				tags.push_back({ RS2_STREAM_DEPTH, -1, 320, 242, RS2_FORMAT_Z16, 5, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
				tags.push_back({ RS2_STREAM_DEPTH, -1, 640, 480, RS2_FORMAT_Z16, 5, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
			}
			else
			{
				tags.push_back({ RS2_STREAM_DEPTH, -1, 320, 240, RS2_FORMAT_Z16, 5, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
				tags.push_back({ RS2_STREAM_DEPTH, -1, 320, 242, RS2_FORMAT_Z16, 5, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
				tags.push_back({ RS2_STREAM_DEPTH, -1, 640, 480, RS2_FORMAT_Z16, 5, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
			}
			return tags;
		};
	};


	std::shared_ptr<device_interface> sdc_info::create(std::shared_ptr<context> ctx,
		bool register_device_notifications) const
	{
		using namespace sdc;

		if (_depth.size() == 0) throw std::runtime_error("Depth Camera not found!");
		auto pid = _depth.front().pid;
		platform::backend_device_group group{ _depth, _hwm, _hid };

		switch (pid)
		{
			case SDC30_PID:
				return std::make_shared<sdc30_device>(ctx, group, register_device_notifications);
			default:
				throw std::runtime_error(to_string() << "Unsupported SDC30 model! 0x"
					<< std::hex << std::setw(4) << std::setfill('0') << (int)pid);
		}
	}

	std::vector<std::shared_ptr<device_info>> sdc_info::pick_sdc_devices(
		std::shared_ptr<context> ctx,
		platform::backend_device_group& group)
	{
		std::vector<platform::uvc_device_info> chosen;
		std::vector<std::shared_ptr<device_info>> results;

		auto valid_pid = filter_by_product(group.uvc_devices, sdc::sdc_sku_pid);
		auto group_devices = group_devices_and_hids_by_unique_id(group_devices_by_unique_id(valid_pid), group.hid_devices);

		for (auto& g : group_devices)
		{
			auto& devices = g.first;
			auto& hids = g.second;

			bool all_sensors_present = mi_present(devices, 0);

			// Device with multi sensors can be enabled only if all sensors (RGB + Depth) are present
			auto is_pid_of_multisensor_device = [](int pid) { return std::find(std::begin(sdc::multi_sensors_pid), std::end(sdc::multi_sensors_pid), pid) != std::end(sdc::multi_sensors_pid); };
			bool is_device_multisensor = false;
			for (auto&& uvc : devices)
			{
				if (is_pid_of_multisensor_device(uvc.pid))
					is_device_multisensor = true;
			}

			if (is_device_multisensor)
			{
				all_sensors_present = all_sensors_present && mi_present(devices, 0); //ToF only 0(interface 0); TOF and RGB 3(Inteface 3)//interface 2번이 존재하면
			}


			auto is_pid_of_hid_sensor_device = [](int pid) { return std::find(std::begin(sdc::hid_sensors_pid), std::end(sdc::hid_sensors_pid), pid) != std::end(sdc::hid_sensors_pid); };
			bool is_device_hid_sensor = false;
			for (auto&& uvc : devices)
			{
				if (is_pid_of_hid_sensor_device(uvc.pid))
					is_device_hid_sensor = true;
			}

			// Device with hids can be enabled only if both hids (gyro and accelerometer) are present
			// Assuming that hids amount of 2 and above guarantee that gyro and accelerometer are present
			if (is_device_hid_sensor)
			{
				all_sensors_present &= (hids.capacity() >= 2);
			}

			if (!devices.empty() && all_sensors_present)
			{
				platform::usb_device_info hwm;

				std::vector<platform::usb_device_info> hwm_devices;
				if (sdc::try_fetch_usb_device(group.usb_devices, devices.front(), hwm))
				{
					hwm_devices.push_back(hwm);
				}
				else
				{
					LOG_DEBUG("try_fetch_usb_device(...) failed.");
				}

				auto info = std::make_shared<sdc_info>(ctx, devices, hwm_devices, hids);
				chosen.insert(chosen.end(), devices.begin(), devices.end());
				results.push_back(info);

			}
			else
			{
				LOG_WARNING("SDC group_devices is empty.");
			}
		}

		trim_device_list(group.uvc_devices, chosen);

		return results;
	}


	inline std::shared_ptr<matcher> create_composite_matcher(std::vector<std::shared_ptr<matcher>> matchers)
	{
		return std::make_shared<timestamp_composite_matcher>(matchers);
	}


	std::shared_ptr<matcher> sdc30_device::create_matcher(const frame_holder& frame) const
	{
		std::vector<stream_interface*> streams = { _depth_stream.get() };

		if (frame.frame->supports_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER))
		{
			return matcher_factory::create(RS2_MATCHER_DLR_C, streams);
		}
		return matcher_factory::create(RS2_MATCHER_DEFAULT, streams);
	}

}
