// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved.

#include <mutex>
#include <chrono>
#include <vector>
#include <iterator>
#include <cstddef>
#include <string>

#include "device.h"
#include "context.h"
#include "image.h"
#include "metadata-parser.h"

#include "sdc-device.h"
#include "sdc-private.h"
#include "sdc-options.h"
#include "sdc-timestamp.h"
#include "stream.h"
#include "environment.h"
//#include "sdc-color.h"
#include "sdc-rolling-shutter.h"

#include "proc/decimation-filter.h"
#include "proc/threshold.h"
#include "proc/disparity-transform.h"
#include "proc/spatial-filter.h"
#include "proc/temporal-filter.h"
#include "proc/hole-filling-filter.h"

namespace librealsense
{
    sdc_auto_exposure_roi_method::sdc_auto_exposure_roi_method(
        const hw_monitor& hwm,
        sdc::fw_cmd cmd)
        : _hw_monitor(hwm), _cmd(cmd) {}

    void sdc_auto_exposure_roi_method::set(const region_of_interest& roi)
    {
        command cmd(_cmd);
        cmd.param1 = roi.min_y;
        cmd.param2 = roi.max_y;
        cmd.param3 = roi.min_x;
        cmd.param4 = roi.max_x;
        _hw_monitor.send(cmd);
    }

    region_of_interest sdc_auto_exposure_roi_method::get() const
    {
        region_of_interest roi;
        command cmd(_cmd + 1);
        auto res = _hw_monitor.send(cmd);

        if (res.size() < 4 * sizeof(uint16_t))
        {
            throw std::runtime_error("Invalid result size!");
        }

        auto words = reinterpret_cast<uint16_t*>(res.data());

        roi.min_y = words[0];
        roi.max_y = words[1];
        roi.min_x = words[2];
        roi.max_x = words[3];

        return roi;
    }

    std::vector<uint8_t> sdc_device::send_receive_raw_data(const std::vector<uint8_t>& input)
    {
        return _hw_monitor->send(input);
    }

    void sdc_device::hardware_reset()
    {
        command cmd(sdc::HWRST);
        _hw_monitor->send(cmd);
    }

    class sdc_depth_sensor : public uvc_sensor, public video_sensor_interface, public depth_stereo_sensor, public roi_sensor_base
    {
    public:
        explicit sdc_depth_sensor(sdc_device* owner,
            std::shared_ptr<platform::uvc_device> uvc_device,
            std::unique_ptr<frame_timestamp_reader> timestamp_reader)
            : uvc_sensor(sdc::DEPTH_STEREO, uvc_device, move(timestamp_reader), owner), _owner(owner), _depth_units(-1)
        {}

        processing_blocks get_recommended_processing_blocks() const override
        {
            return get_sdc_depth_recommended_proccesing_blocks();
        };

        rs2_intrinsics get_intrinsics(const stream_profile& profile) const override
        {
            return get_intrinsic_by_resolution(
                *_owner->_coefficients_table_raw,
                sdc::calibration_table_id::coefficients_table_id,
                profile.width, profile.height);
        }

        void open(const stream_profiles& requests) override
        {
            _depth_units = get_option(RS2_OPTION_DEPTH_UNITS).query();
            uvc_sensor::open(requests);
        }

        /*
        Infrared profiles are initialized with the following logic:
        - If device has color sensor (D415 / D435), infrared profile is chosen with Y8 format
        - If device does not have color sensor:
           * if it is a rolling shutter device (D400 / D410 / D415 / D405), infrared profile is chosen with RGB8 format
           * for other devices (D420 / D430), infrared profile is chosen with Y8 format
        */
        stream_profiles init_stream_profiles() override
        {
            auto lock = environment::get_instance().get_extrinsics_graph().lock();

            auto results = uvc_sensor::init_stream_profiles();

            //auto color_dev = dynamic_cast<const sdc_color*>(&get_device());
            auto rolling_shutter_dev = dynamic_cast<const sdc_rolling_shutter*>(&get_device());

            std::vector< video_stream_profile_interface*> depth_candidates;
            std::vector< video_stream_profile_interface*> infrared_candidates;

            auto candidate = [](video_stream_profile_interface* prof, platform::stream_profile tgt, rs2_stream stream, int streamindex) -> bool
            {
                return ((tgt.width == prof->get_width()) && (tgt.height == prof->get_height()) &&
                    (tgt.format == RS2_FORMAT_ANY || tgt.format == prof->get_format()) &&
                    (stream == RS2_STREAM_ANY || stream == prof->get_stream_type()) &&
                    (tgt.fps == prof->get_framerate()) && (streamindex == prof->get_stream_index()));
            };

            for (auto p : results)
            {
                // Register stream types
                if (p->get_stream_type() == RS2_STREAM_DEPTH)
                {
                    assign_stream(_owner->_depth_stream, p);
                }
				
				auto vid_profile = dynamic_cast<video_stream_profile_interface*>(p.get());

                // Register intrinsics
                if (p->get_format() != RS2_FORMAT_Y16) // Y16 format indicate unrectified images, no intrinsics are available for these
                {
                    auto profile = to_profile(p.get());
                    std::weak_ptr<sdc_depth_sensor> wp =
                        std::dynamic_pointer_cast<sdc_depth_sensor>(this->shared_from_this());
                    vid_profile->set_intrinsics([profile, wp]()
                    {
                        auto sp = wp.lock();
                        if (sp)
                            return sp->get_intrinsics(profile);
                        else
                            return rs2_intrinsics{};
                    });
                }
            }

            auto dev = dynamic_cast<const sdc_device*>(&get_device());
            auto dev_name = (dev) ? dev->get_info(RS2_CAMERA_INFO_NAME) : "";

            auto cmp = [](const video_stream_profile_interface* l, const video_stream_profile_interface* r) -> bool
            {
                return ((l->get_width() < r->get_width()) || (l->get_height() < r->get_height()));
            };

            return results;
        }

        float get_depth_scale() const override { if (_depth_units < 0) _depth_units = get_option(RS2_OPTION_DEPTH_UNITS).query(); return _depth_units; }

        void set_depth_scale(float val){ _depth_units = val; }

        float get_stereo_baseline_mm() const override { return _owner->get_stereo_baseline_mm(); }

        void create_snapshot(std::shared_ptr<depth_sensor>& snapshot) const override
        {
            snapshot = std::make_shared<depth_sensor_snapshot>(get_depth_scale());
        }

        void create_snapshot(std::shared_ptr<depth_stereo_sensor>& snapshot) const override
        {
            snapshot = std::make_shared<depth_stereo_sensor_snapshot>(get_depth_scale(), get_stereo_baseline_mm());
        }

        void enable_recording(std::function<void(const depth_sensor&)> recording_function) override
        {
            //does not change over time
        }

        void enable_recording(std::function<void(const depth_stereo_sensor&)> recording_function) override
        {
            //does not change over time
        }
    protected:
        const sdc_device* _owner;
        mutable std::atomic<float> _depth_units;
        float _stereo_baseline_mm;
    };

    class sdcu_depth_sensor : public sdc_depth_sensor
    {
    public:
        explicit sdcu_depth_sensor(sdcu_device* owner,
            std::shared_ptr<platform::uvc_device> uvc_device,
            std::unique_ptr<frame_timestamp_reader> timestamp_reader)
            : sdc_depth_sensor(owner, uvc_device, move(timestamp_reader)), _owner(owner)
        {}

        stream_profiles init_stream_profiles() override
        {
            auto lock = environment::get_instance().get_extrinsics_graph().lock();

            auto results = uvc_sensor::init_stream_profiles();

            for (auto p : results)
            {
                // Register stream types
                if (p->get_stream_type() == RS2_STREAM_DEPTH)
                {
                    assign_stream(_owner->_depth_stream, p);
                }

                auto video = dynamic_cast<video_stream_profile_interface*>(p.get());

                // Register intrinsics
                if (p->get_format() != RS2_FORMAT_Y16) // Y16 format indicate unrectified images, no intrinsics are available for these
                {
                    auto profile = to_profile(p.get());
                    std::weak_ptr<sdc_depth_sensor> wp = std::dynamic_pointer_cast<sdc_depth_sensor>(this->shared_from_this());
                    video->set_intrinsics([profile, wp]()
                    {
                        auto sp = wp.lock();
                        if (sp)
                            return sp->get_intrinsics(profile);
                        else
                            return rs2_intrinsics{};
                    });
                }
            }

            return results;
        }

    private:
        const sdcu_device* _owner;
    };

    bool sdc_device::is_camera_in_advanced_mode() const
    {
#if defined(USE_XU_UNIT) && USE_XU_UNIT
        command cmd(sdc::UAMG);
        assert(_hw_monitor);
        auto ret = _hw_monitor->send(cmd);
        if (ret.empty())
            throw invalid_value_exception("command result is empty!");

        return (0 != ret.front());
#else
		return 1;
#endif
    }

    float sdc_device::get_stereo_baseline_mm() const
    {
        using namespace sdc;
        auto table = check_calib<coefficients_table>(*_coefficients_table_raw);
        return fabs(table->baseline);
    }

	std::vector<uint8_t> sdc_device::get_raw_calibration_table(sdc::calibration_table_id table_id) const
	{
#if defined(USE_XU_UNIT) && USE_XU_UNIT
		command cmd(sdc::GETINTCAL, table_id);
		return _hw_monitor->send(cmd);
#else
		return {
			// GETINTCAL
			//
			//REALSENSE_XU_GETINTCAL, 0x00, 0x00, 0x00,
			///////////////////////////////////////////////////////
			// Header
			///////////////////////////////////////////////////////
			/* OFFSET 0   */ 0x03, 0x00,              // version 3.0 major.minor. Big-endian(특별히 요거만 Big endian이다.)
			/* OFFSET 2   */ 0x19, 0x00,              // table_type : ctCalibration =0x19(25)
			/* OFFSET 4   */ 0xf0, 0x01, 0x00, 0x00,  // full size including: TOC header + TOC + actual tables
			/* OFFSET 8   */ 0x6e, 0x0f, 0x2b, 0x04,  // This field content is defined ny table type
#if 0 //Original
			/* OFFSET 12  */ 0x21, 0xf5, 0x68, 0x47,  // crc of all the actual table data excluding header/CRC
#else
			/* OFFSET 12  */ 0xd5, 0xcc, 0x2d, 0x9e,  // crc of all the actual table data excluding header/CRC
#endif

	////////////////////////////////////////////////////////////////////
	// intrinsic_left 3x3, left camera intrinsic data, normilized
	////////////////////////////////////////////////////////////////////
	/* 0FFSET 16  */ 0x90, 0x4e, 0xff, 0x3e, 0x9f, 0x0e, 0x4c, 0x3f, 0xe1, 0xf8, 0x00, 0x3f,
	/* 0FFSET 28  */ 0x16, 0x1c, 0x06, 0x3f, 0x99, 0x1c, 0x67, 0xbd, 0x6c, 0x7d, 0x84, 0x3d,
	/* 0FFSET 40  */ 0x9f, 0x8d, 0x68, 0xb8, 0x5a, 0x2c, 0x85, 0xba, 0x2f, 0x35, 0xaa, 0xbc,

	////////////////////////////////////////////////////////////////////
	// intrinsic_right 3x3, right camera intrinsic data, normilized 
	////////////////////////////////////////////////////////////////////
	/* 0FFSET 52  */ 0xba, 0x02, 0x00, 0x3f, 0x3f, 0x9a, 0x4c, 0x3f, 0x66, 0x62, 0x03, 0x3f,
	/* 0FFSET 64  */ 0xe9, 0xb2, 0x04, 0x3f, 0x35, 0x08, 0x66, 0xbd, 0xc0, 0xab, 0x83, 0x3d,
	/* 0FFSET 76  */ 0x00, 0x87, 0xb1, 0x36, 0x07, 0x89, 0x2d, 0xba, 0x82, 0x27, 0xab, 0xbc,

	////////////////////////////////////////////////////////////////////
	// world2left_rot 3x3, the inverse rotation of the left camera
	////////////////////////////////////////////////////////////////////
	/* 0FFSET 88  */ 0x99, 0xfe, 0x7f, 0x3f, 0x58, 0x60, 0x97, 0xba, 0xe1, 0x0a, 0xd3, 0x3b,
	/* 0FFSET 100 */ 0x67, 0x49, 0x98, 0x3a, 0xeb, 0xff, 0x7f, 0x3f, 0xa2, 0xe0, 0x8c, 0xba,
	/* 0FFSET 112 */ 0x66, 0x00, 0xd3, 0xbb, 0xf2, 0xda, 0x8d, 0x3a, 0x9a, 0xfe, 0x7f, 0x3f,

	////////////////////////////////////////////////////////////////////
	// world2right_rot 3x3, the inverse rotation of the left camera
	////////////////////////////////////////////////////////////////////
	/* 0FFSET 124 */ 0x8b, 0xff, 0x7f, 0x3f, 0x9e, 0x7c, 0x74, 0xbb, 0xe8, 0xa0, 0x06, 0xb9,
	/* 0FFSET 136 */ 0xe7, 0x7e, 0x74, 0x3b, 0x81, 0xff, 0x7f, 0x3f, 0x03, 0x56, 0x8d, 0x3a,
	/* 0FFSET 148 */ 0xcf, 0x68, 0x02, 0x39, 0xd5, 0x65, 0x8d, 0xba, 0xf6, 0xff, 0x7f, 0x3f,

	////////////////////////////////////////////////////////////////////
	// baseline, the baseline between the cameras in mm units
	////////////////////////////////////////////////////////////////////
	/* 0FFSET 160 */ 0x8a, 0x5c, 0x47, 0xc2,

	////////////////////////////////////////////////////////////////////
	// brown_model,  Distortion model: 0 - DS distorion model, 1 - Brown model
	////////////////////////////////////////////////////////////////////
	/* 0FFSET 164 */ 0x01, 0x00, 0x00, 0x00,

	////////////////////////////////////////////////////////////////////
	// reserved1, 88bytes
	////////////////////////////////////////////////////////////////////
	/* 0FFSET 168 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 184 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 200 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 216 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 232 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 248 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	////////////////////////////////////////////////////////////////////
	// rect_params 16 x(fx,fy,fz,fw) --> reserved 2영역과 겹친다. 
	// fx  -- fx: focal length(pixel 폭(width) 사이즈로 얼마 만큼 떨어져 있는지)를 지정한다.
	// fy  -- fy: focal length(pixel 높이(height) 사이즈로 얼마 만큼 떨어져 있는지)를 지정한다.
	// ppx -- fz: 이미지 원점 좌표를 왼쪽에서 부터 얼마큼 떨어져 있는지 pixel 폭(width) 사이즈로 지정한다.
	// ppy -- fw: 이미지 원점 좌표를 윗쪽에서 부터 얼마큼 떨어져 있는지 pixel 높이(height) 사이즈로 지정한다.
	////////////////////////////////////////////////////////////////////
#if 0 //Original for DS435
	/* 0FFSET 256 0  1920x1080 (fx,fy,fz,fw)(959.835,959.835,978.241,563.419) */ 0x77, 0xf5, 0x6f, 0x44, 0x77, 0xf5, 0x6f, 0x44, 0x6d, 0x8f, 0x74, 0x44, 0xcf, 0xda, 0x0c, 0x44,
	/* 0FFSET 272 1  1280x720  (fx,fy,fz,fw)(639.890,639.890,652.161,375.613) */ 0xfa, 0xf8, 0x1f, 0x44, 0xfa, 0xf8, 0x1f, 0x44, 0x49, 0x0a, 0x23, 0x44, 0x6a, 0xce, 0xbb, 0x43,
	/* 0FFSET 288 2  640x480   (fx,fy,fz,fw)(383.934,383.934,327.296,249.368) */ 0x92, 0xf7, 0xbf, 0x43, 0x92, 0xf7, 0xbf, 0x43, 0xf1, 0xa5, 0xa3, 0x43, 0x19, 0x5e, 0x79, 0x43,
	/* 0FFSET 304 3  848x480   (fx,fy,fz,fw)(423.927,423.927,432.056,250.343) */ 0xb1, 0xf6, 0xd3, 0x43, 0xb1, 0xf6, 0xd3, 0x43, 0x3a, 0x07, 0xd8, 0x43, 0xe6, 0x57, 0x7a, 0x43,
	/* 0FFSET 320 4  640x360   (fx,fy,fz,fw)(299.945,299.945,326.080,187.806) */ 0xfa, 0xf8, 0x9f, 0x43, 0xfa, 0xf8, 0x9f, 0x43, 0x49, 0x0a, 0xa3, 0x43, 0x6a, 0xce, 0x3b, 0x43,
	/* 0FFSET 336 5  424x240   (fx,fy,fz,fw)(211.964,211.964,216.028,125.172) */ 0xb1, 0xf6, 0x53, 0x43, 0xb1, 0xf6, 0x53, 0x43, 0x3a, 0x07, 0x58, 0x43, 0xe6, 0x57, 0xfa, 0x42,
	/* 0FFSET 352 6  320x240   (fx,fy,fz,fw)(191.967,191.967,163.648,124.615) */ 0x92, 0xf7, 0x3f, 0x43, 0x92, 0xf7, 0x3f, 0x43, 0xf1, 0xa5, 0x23, 0x43, 0x3e, 0x3a, 0xf9, 0x42,
	/* 0FFSET 368 7  480x270   (fx,fy,fz,fw)(239.959,239.959,244.560,140.855) */ 0x77, 0xf5, 0x6f, 0x43, 0x77, 0xf5, 0x6f, 0x43, 0x6d, 0x8f, 0x74, 0x43, 0xcf, 0xda, 0x0c, 0x43,
	/* 0FFSET 384 8  1280x800  (fx,fy,fz,fw)(639.890,639.890,652.161,415.613) */ 0xfa, 0xf8, 0x1f, 0x44, 0xfa, 0xf8, 0x1f, 0x44, 0x49, 0x0a, 0x23, 0x44, 0x6a, 0xce, 0xcf, 0x43,
	/* 0FFSET 400 9  960x540   (fx,fy,fz,fw)(479.918,479.918,489.121,281.709) */ 0x77, 0xf5, 0xef, 0x43, 0x77, 0xf5, 0xef, 0x43, 0x6d, 0x8f, 0xf4, 0x43, 0xcf, 0xda, 0x8c, 0x43,
	/* 0FFSET 416 13 575x575   (fx,fy,fz,fw)(575.901,575.901,370.945,374.051) */ 0xae, 0xf9, 0x0f, 0x44, 0xae, 0xf9, 0x0f, 0x44, 0xea, 0x78, 0xb9, 0x43, 0x92, 0x06, 0xbb, 0x43,
	/* 0FFSET 432 ??           (fx,fy,fz,fw)(460.721,460.721,296.756,299.241) */ 0x49, 0x5c, 0xe6, 0x43, 0x49, 0x5c, 0xe6, 0x43, 0xbb, 0x60, 0x94, 0x43, 0xdc, 0x9e, 0x95, 0x43,
#else //for SDC30
	/* 0FFSET 256 0  320x240   (fx,fy,fz,fw)(191.967,191.967,163.648,124.615) */ 0x92, 0xf7, 0x3f, 0x43, 0x92, 0xf7, 0x3f, 0x43, 0xf1, 0xa5, 0x23, 0x43, 0x3e, 0x3a, 0xf9, 0x42,
	/* 0FFSET 272 1  320x242   (fx,fy,fz,fw)(191.967,191.967,163.648,124.615) */ 0x92, 0xf7, 0x3f, 0x43, 0x92, 0xf7, 0x3f, 0x43, 0xf1, 0xa5, 0x23, 0x43, 0x3e, 0x3a, 0xf9, 0x42,
	/* 0FFSET 288 2  320x246   (fx,fy,fz,fw)(191.967,191.967,163.648,124.615) */ 0x92, 0xf7, 0x3f, 0x43, 0x92, 0xf7, 0x3f, 0x43, 0xf1, 0xa5, 0x23, 0x43, 0x3e, 0x3a, 0xf9, 0x42,
	/* 0FFSET 304 3  Reserved 1(fx,fy,fz,fw)(423.927,423.927,432.056,250.343) */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 320 4  Reserved 2  (fx,fy,fz,fw)(299.945,299.945,326.080,187.806) */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 336 5  Reserved 3  (fx,fy,fz,fw)(211.964,211.964,216.028,125.172) */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 352 6  Reserved 4  (fx,fy,fz,fw)(191.967,191.967,163.648,124.615) */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 368 7  Reserved 5  (fx,fy,fz,fw)(239.959,239.959,244.560,140.855) */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 384 8  Reserved 6  (fx,fy,fz,fw)(639.890,639.890,652.161,415.613) */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 400 9  Reserved 7  (fx,fy,fz,fw)(479.918,479.918,489.121,281.709) */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 416 10 Reserved 8  (fx,fy,fz,fw)(575.901,575.901,370.945,374.051) */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 432 11             (fx,fy,fz,fw)(460.721,460.721,296.756,299.241) */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#endif
	////////////////////////////////////////////////////////////////////
	// reserved2 64bytes
	////////////////////////////////////////////////////////////////////
	/* 0FFSET 448 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 464 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 480 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* 0FFSET 496 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		};
#endif
	}

	sdc::sdc30_caps sdc_device::parse_device_capabilities() const
	{
#if defined(USE_XU_UNIT) && USE_XU_UNIT
		using namespace sdc;
		std::array<unsigned char, HW_MONITOR_BUFFER_SIZE> gvd_buf;
		_hw_monitor->get_gvd(gvd_buf.size(), gvd_buf.data(), GVD);

		// Opaque retrieval
		sdc30_caps val{ sdc30_caps::CAP_UNDEFINED };
		if (gvd_buf[active_projector])  // DepthActiveMode
			val |= sdc30_caps::CAP_ACTIVE_PROJECTOR;
		if (gvd_buf[rgb_sensor])                           // WithRGB
			val |= sdc30_caps::CAP_RGB_SENSOR;
		if (gvd_buf[imu_sensor])
			val |= sdc30_caps::CAP_IMU_SENSOR;
		if (0xFF != (gvd_buf[fisheye_sensor_lb] & gvd_buf[fisheye_sensor_hb]))
			val |= sdc30_caps::CAP_FISHEYE_SENSOR;
		if (0x1 == gvd_buf[depth_sensor_type])
			val |= sdc30_caps::CAP_ROLLING_SHUTTER;  // Standard depth
		if (0x2 == gvd_buf[depth_sensor_type])
			val |= sdc30_caps::CAP_GLOBAL_SHUTTER;   // Wide depth
#else
		using namespace sdc;
		// Opaque retrieval
		sdc30_caps val{ sdc30_caps::CAP_ACTIVE_PROJECTOR | sdc30_caps::CAP_GLOBAL_SHUTTER };
		
#endif
        return val;
    }

    std::shared_ptr<uvc_sensor> sdc_device::create_depth_device(std::shared_ptr<context> ctx,
                                                                const std::vector<platform::uvc_device_info>& all_device_infos)
    {
        using namespace sdc;

        auto&& backend = ctx->get_backend();

        std::vector<std::shared_ptr<platform::uvc_device>> depth_devices;
        for (auto&& info : filter_by_mi(all_device_infos, 0)) // Filter just mi=0, DEPTH
            depth_devices.push_back(backend.create_uvc_device(info));

        std::unique_ptr<frame_timestamp_reader> sdc_timestamp_reader_backup(new sdc_timestamp_reader(backend.create_time_service()));
        auto depth_ep = std::make_shared<sdc_depth_sensor>(this, std::make_shared<platform::multi_pins_uvc_device>(depth_devices),
                                                       std::unique_ptr<frame_timestamp_reader>(new sdc_timestamp_reader_from_metadata(std::move(sdc_timestamp_reader_backup))));
        depth_ep->register_xu(depth_xu); // make sure the XU is initialized every time we power the camera

        depth_ep->register_pixel_format(pf_z16); // Depth
        depth_ep->register_pixel_format(pf_y8); // Left Only - Luminance
        depth_ep->register_pixel_format(pf_yuyv); // Left Only

        return depth_ep;
    }

    sdc_device::sdc_device(std::shared_ptr<context> ctx,
                           const platform::backend_device_group& group)
        : device(ctx, group),
          _depth_stream(new stream(RS2_STREAM_DEPTH)),
          _device_capabilities(sdc::sdc30_caps::CAP_UNDEFINED),
          _depth_device_idx(add_sensor(create_depth_device(ctx, group.uvc_devices)))
    {
        init(ctx, group);
    }

    void sdc_device::init(std::shared_ptr<context> ctx,
        const platform::backend_device_group& group)
    {
        using namespace sdc;

        auto&& backend = ctx->get_backend();

        if (group.usb_devices.size() > 0)
        {
            _hw_monitor = std::make_shared<hw_monitor>(
                std::make_shared<locked_transfer>(
                    backend.create_usb_device(group.usb_devices.front()), get_depth_sensor()));
        }
        else
        {
#if defined(USE_XU_UNIT) && USE_XU_UNIT
            _hw_monitor = std::make_shared<hw_monitor>(
                std::make_shared<locked_transfer>(
                    std::make_shared<command_transfer_over_xu>(
                        get_depth_sensor(), depth_xu, SDC_HWMONITOR),
                    get_depth_sensor()));
#endif
        }

        // Define Left-to-Right extrinsics calculation (lazy)
        // Reference CS - Right-handed; positive [X,Y,Z] point to [Left,Up,Forward] accordingly.
#if defined(USE_XU_UNIT) && USE_XU_UNIT
        _left_right_extrinsics = std::make_shared<lazy<rs2_extrinsics>>([this]()
        {
            rs2_extrinsics ext = identity_matrix();
            auto table = check_calib<coefficients_table>(*_coefficients_table_raw);
            ext.translation[0] = 0.001f * table->baseline; // mm to meters
            return ext;
        });
#endif
		register_stream_to_extrinsic_group(*_depth_stream, 0);

		_coefficients_table_raw = [this]() { return get_raw_calibration_table(coefficients_table_id); };

		auto pid = group.uvc_devices.front().pid;
		std::string device_name = (rsSDC30_sku_names.end() != rsSDC30_sku_names.find(pid)) ? rsSDC30_sku_names.at(pid) : "SDCxx";
#if defined(USE_XU_UNIT) && USE_XU_UNIT
		_fw_version = firmware_version(_hw_monitor->get_firmware_version_string(GVD, camera_fw_version_offset));
#else
		_fw_version = firmware_version("19.11.7.0");
#endif
        _recommended_fw_version = firmware_version("19.11.7.0");
        if (_fw_version >= firmware_version("19.11.7.0"))
            _device_capabilities = parse_device_capabilities();
#if defined(USE_XU_UNIT) && USE_XU_UNIT
        auto serial = _hw_monitor->get_module_serial_string(GVD, module_serial_offset);
#else
		auto serial = "1910160000010000";
#endif

        auto& depth_ep = get_depth_sensor();
        auto advanced_mode = is_camera_in_advanced_mode();

        using namespace platform;
        auto _usb_mode = usb3_type;
        std::string usb_type_str(usb_spec_names.at(_usb_mode));
        bool usb_modality = (_fw_version >= firmware_version("19.11.7.0"));
        if (usb_modality)
        {
            _usb_mode = depth_ep.get_usb_specification();
            if (usb_spec_names.count(_usb_mode) && (usb_undefined != _usb_mode))
                usb_type_str = usb_spec_names.at(_usb_mode);
            else  // Backend fails to provide USB descriptor  - occurs with RS3 build. Requires further work
                usb_modality = false;
        }

        auto pid_hex_str = hexify(pid);

#if defined(USE_XU_UNIT) && USE_XU_UNIT
        if ((pid != SDC30_PID) && _fw_version >= firmware_version("19.11.7.0"))
        {
            depth_ep.register_option(RS2_OPTION_HARDWARE_PRESET,
                std::make_shared<uvc_xu_option<uint8_t>>(depth_ep, depth_xu, SDC_HARDWARE_PRESET,
                    "Hardware pipe configuration"));
        }
#endif

        std::string is_camera_locked{ "" };
        if (_fw_version >= firmware_version("19.11.7.0"))
        {
#if defined(USE_XU_UNIT) && USE_XU_UNIT
            auto is_locked = _hw_monitor->is_camera_locked(GVD, is_camera_locked_offset);
            is_camera_locked = (is_locked) ? "YES" : "NO";
#else
			is_camera_locked = "YES";
#endif

#if defined(USE_XU_UNIT) && USE_XU_UNIT
#ifdef HWM_OVER_XU
            //if hw_monitor was created by usb replace it with xu
            // D400_IMU will remain using USB interface due to HW limitations
            if ((group.usb_devices.size() > 0) && (SDC30_PID == pid))
            {
                _hw_monitor = std::make_shared<hw_monitor>(
                    std::make_shared<locked_transfer>(
                        std::make_shared<command_transfer_over_xu>(
                            get_depth_sensor(), depth_xu, SDC_HWMONITOR),
                        get_depth_sensor()));
            }
#endif
#endif
#if defined(USE_SDC30_TPG) && USE_SDC30_TPG
			depth_ep.register_pu(RS2_OPTION_BACKGROUND_OFFSET);
			depth_ep.register_pu(RS2_OPTION_PHASE_ALIGNMENT);
#endif

			if (_fw_version >= firmware_version("20.11.7.0"))
			{
				auto exposure_option = std::make_shared<uvc_xu_option<uint32_t>>(depth_ep,
					depth_xu,
					SDC_EXPOSURE,
					"Depth Exposure (usec)");
				depth_ep.register_option(RS2_OPTION_EXPOSURE, exposure_option);

				auto enable_auto_exposure = std::make_shared<uvc_xu_option<uint8_t>>(depth_ep,
					depth_xu,
					SDC_ENABLE_AUTO_EXPOSURE,
					"Enable Auto Exposure");
				depth_ep.register_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, enable_auto_exposure);


				depth_ep.register_option(RS2_OPTION_EXPOSURE,
					std::make_shared<auto_disabling_control>(
					exposure_option,
					enable_auto_exposure));
			}
        }

        if (_fw_version >= firmware_version("20.11.7.0"))
        {
            depth_ep.register_option(RS2_OPTION_OUTPUT_TRIGGER_ENABLED,
                std::make_shared<uvc_xu_option<uint8_t>>(depth_ep, depth_xu, SDC_EXT_TRIGGER,
                    "Generate trigger from the camera to external device once per frame"));

            auto error_control = std::unique_ptr<uvc_xu_option<uint8_t>>(new uvc_xu_option<uint8_t>(depth_ep, depth_xu, SDC_ERROR_REPORTING, "Error reporting"));

            _polling_error_handler = std::unique_ptr<polling_error_handler>(
                new polling_error_handler(1000,
                    std::move(error_control),
                    depth_ep.get_notifications_processor(),
                    std::unique_ptr<notification_decoder>(new sdc_notification_decoder())));

            depth_ep.register_option(RS2_OPTION_ERROR_POLLING_ENABLED, std::make_shared<polling_errors_disable>(_polling_error_handler.get()));

			depth_ep.register_option(RS2_OPTION_ASIC_TEMPERATURE,
				std::make_shared<sdc_asic_and_projector_temperature_options>(depth_ep,
				RS2_OPTION_ASIC_TEMPERATURE));
        }

        // Alternating laser pattern is applicable for global shutter/active SKUs
        auto mask = sdc30_caps::CAP_GLOBAL_SHUTTER | sdc30_caps::CAP_ACTIVE_PROJECTOR;
        if ((_fw_version >= firmware_version("20.11.7.0")) && ((_device_capabilities & mask) == mask))
        {
            depth_ep.register_option(RS2_OPTION_EMITTER_ON_OFF, std::make_shared<sdc_alternating_emitter_option>(*_hw_monitor, &depth_ep));
        }
        else if (_fw_version >= firmware_version("20.11.7.0") &&
            _fw_version.experimental()) // Not yet available in production firmware
        {
            depth_ep.register_option(RS2_OPTION_EMITTER_ON_OFF, std::make_shared<sdc_emitter_on_and_off_option>(*_hw_monitor, &depth_ep));
        }

        if (_fw_version >= firmware_version("20.11.7.0"))
        {
            get_depth_sensor().register_option(RS2_OPTION_INTER_CAM_SYNC_MODE,
                std::make_shared<sdc_external_sync_mode>(*_hw_monitor));
        }

        roi_sensor_interface* roi_sensor;
        if ((roi_sensor = dynamic_cast<roi_sensor_interface*>(&depth_ep)))
            roi_sensor->set_roi_method(std::make_shared<sdc_auto_exposure_roi_method>(*_hw_monitor));

		if (_fw_version >= firmware_version("20.11.7.0"))
		{
			depth_ep.register_option(RS2_OPTION_STEREO_BASELINE, std::make_shared<const_value_option>("Distance in mm between the stereo imagers",
				lazy<float>([this]() { return get_stereo_baseline_mm(); })));
		}

        if (advanced_mode && _fw_version >= firmware_version("20.11.7.0"))
        {
            auto depth_scale = std::make_shared<sdc_depth_scale_option>(*_hw_monitor);
            auto depth_sensor = As<sdc_depth_sensor, uvc_sensor>(&depth_ep);
            assert(depth_sensor);

            depth_scale->add_observer([depth_sensor](float val)
            {
                depth_sensor->set_depth_scale(val);
            });

            depth_ep.register_option(RS2_OPTION_DEPTH_UNITS, depth_scale);
        }
        else
            depth_ep.register_option(RS2_OPTION_DEPTH_UNITS, std::make_shared<const_value_option>("Number of meters represented by a single depth unit",
                lazy<float>([]() { return 0.001f; })));

        // Metadata registration
        depth_ep.register_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP, make_uvc_header_parser(&uvc_header::timestamp));

        // attributes of md_capture_timing
        auto md_prop_offset = offsetof(metadata_raw, mode) +
            offsetof(md_depth_mode, depth_y_mode) +
            offsetof(md_depth_y_normal_mode, intel_capture_timing);

        depth_ep.register_metadata(RS2_FRAME_METADATA_FRAME_COUNTER, make_attribute_parser(&md_capture_timing::frame_counter, md_capture_timing_attributes::frame_counter_attribute, md_prop_offset));
        depth_ep.register_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP, make_rs400_sensor_ts_parser(make_uvc_header_parser(&uvc_header::timestamp),
            make_attribute_parser(&md_capture_timing::sensor_timestamp, md_capture_timing_attributes::sensor_timestamp_attribute, md_prop_offset)));

        // attributes of md_capture_stats
        md_prop_offset = offsetof(metadata_raw, mode) +
            offsetof(md_depth_mode, depth_y_mode) +
            offsetof(md_depth_y_normal_mode, intel_capture_stats);

        depth_ep.register_metadata(RS2_FRAME_METADATA_WHITE_BALANCE, make_attribute_parser(&md_capture_stats::white_balance, md_capture_stat_attributes::white_balance_attribute, md_prop_offset));

        // attributes of md_depth_control
        md_prop_offset = offsetof(metadata_raw, mode) +
            offsetof(md_depth_mode, depth_y_mode) +
            offsetof(md_depth_y_normal_mode, intel_depth_control);

        depth_ep.register_metadata(RS2_FRAME_METADATA_GAIN_LEVEL, make_attribute_parser(&md_depth_control::manual_gain, md_depth_control_attributes::gain_attribute, md_prop_offset));
        depth_ep.register_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE, make_attribute_parser(&md_depth_control::manual_exposure, md_depth_control_attributes::exposure_attribute, md_prop_offset));
        depth_ep.register_metadata(RS2_FRAME_METADATA_AUTO_EXPOSURE, make_attribute_parser(&md_depth_control::auto_exposure_mode, md_depth_control_attributes::ae_mode_attribute, md_prop_offset));

        depth_ep.register_metadata(RS2_FRAME_METADATA_FRAME_LASER_POWER, make_attribute_parser(&md_depth_control::laser_power, md_depth_control_attributes::laser_pwr_attribute, md_prop_offset));
        depth_ep.register_metadata(RS2_FRAME_METADATA_FRAME_LASER_POWER_MODE, make_attribute_parser(&md_depth_control::laserPowerMode, md_depth_control_attributes::laser_pwr_attribute, md_prop_offset));
        depth_ep.register_metadata(RS2_FRAME_METADATA_EXPOSURE_PRIORITY, make_attribute_parser(&md_depth_control::exposure_priority, md_depth_control_attributes::exposure_priority_attribute, md_prop_offset));
        depth_ep.register_metadata(RS2_FRAME_METADATA_EXPOSURE_ROI_LEFT, make_attribute_parser(&md_depth_control::exposure_roi_left, md_depth_control_attributes::roi_attribute, md_prop_offset));
        depth_ep.register_metadata(RS2_FRAME_METADATA_EXPOSURE_ROI_RIGHT, make_attribute_parser(&md_depth_control::exposure_roi_right, md_depth_control_attributes::roi_attribute, md_prop_offset));
        depth_ep.register_metadata(RS2_FRAME_METADATA_EXPOSURE_ROI_TOP, make_attribute_parser(&md_depth_control::exposure_roi_top, md_depth_control_attributes::roi_attribute, md_prop_offset));
        depth_ep.register_metadata(RS2_FRAME_METADATA_EXPOSURE_ROI_BOTTOM, make_attribute_parser(&md_depth_control::exposure_roi_bottom, md_depth_control_attributes::roi_attribute, md_prop_offset));

        // md_configuration - will be used for internal validation only
        md_prop_offset = offsetof(metadata_raw, mode) + offsetof(md_depth_mode, depth_y_mode) + offsetof(md_depth_y_normal_mode, intel_configuration);

        depth_ep.register_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_HW_TYPE, make_attribute_parser(&md_configuration::hw_type, md_configuration_attributes::hw_type_attribute, md_prop_offset));
        depth_ep.register_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_SKU_ID, make_attribute_parser(&md_configuration::sku_id, md_configuration_attributes::sku_id_attribute, md_prop_offset));
        depth_ep.register_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_FORMAT, make_attribute_parser(&md_configuration::format, md_configuration_attributes::format_attribute, md_prop_offset));
        depth_ep.register_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_WIDTH, make_attribute_parser(&md_configuration::width, md_configuration_attributes::width_attribute, md_prop_offset));
        depth_ep.register_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_HEIGHT, make_attribute_parser(&md_configuration::height, md_configuration_attributes::height_attribute, md_prop_offset));
        depth_ep.register_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_ACTUAL_FPS,  std::make_shared<sdc_md_attribute_actual_fps> ());

        register_info(RS2_CAMERA_INFO_NAME, device_name);
        register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, serial);
        register_info(RS2_CAMERA_INFO_FIRMWARE_VERSION, _fw_version);
        register_info(RS2_CAMERA_INFO_PHYSICAL_PORT, group.uvc_devices.front().device_path);
#if defined(USE_XU_UNIT) && USE_XU_UNIT
        register_info(RS2_CAMERA_INFO_DEBUG_OP_CODE, std::to_string(static_cast<int>(fw_cmd::GLD)));
#endif
        register_info(RS2_CAMERA_INFO_ADVANCED_MODE, ((advanced_mode) ? "YES" : "NO"));
        register_info(RS2_CAMERA_INFO_PRODUCT_ID, pid_hex_str);
        register_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION, _recommended_fw_version);

        if (usb_modality)
            register_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR, usb_type_str);

        std::string curr_version= _fw_version;
        std::string minimal_version = _recommended_fw_version;

        if (_fw_version < _recommended_fw_version)
        {
            std::weak_ptr<notifications_processor> weak = depth_ep.get_notifications_processor();
            std::thread notification_thread = std::thread([weak, curr_version, minimal_version]()
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                while (true)
                {
                    auto ptr = weak.lock();
                    if (ptr)
                    {
                        std::string msg = "Current firmware version: " + curr_version + "\nMinimal firmware version: " + minimal_version +"\n";
                        notification n(RS2_NOTIFICATION_CATEGORY_FIRMWARE_UPDATE_RECOMMENDED, 0, RS2_LOG_SEVERITY_INFO, msg);
                        ptr->raise_notification(n);
                    }
                    else
                    {
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::hours(8));
                }
            });
            notification_thread.detach();
        }
    }
	
    notification sdc_notification_decoder::decode(int value)
    {
        if (sdc::sdc_fw_error_report.find(static_cast<uint8_t>(value)) != sdc::sdc_fw_error_report.end())
            return{ RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR, value, RS2_LOG_SEVERITY_ERROR, sdc::sdc_fw_error_report.at(static_cast<uint8_t>(value)) };

        return{ RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR, value, RS2_LOG_SEVERITY_WARN, (to_string() << "D400 HW report - unresolved type " << value) };
    }

    void sdc_device::create_snapshot(std::shared_ptr<debug_interface>& snapshot) const
    {
        //TODO: Implement
    }
    void sdc_device::enable_recording(std::function<void(const debug_interface&)> record_action)
    {
        //TODO: Implement
    }

    platform::usb_spec sdc_device::get_usb_spec() const
    {
        if(!supports_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
            return platform::usb_undefined;
        auto str = get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
        for (auto u : platform::usb_spec_names)
        {
            if (u.second.compare(str) == 0)
                return u.first;
        }
        return platform::usb_undefined;
    }

    std::shared_ptr<uvc_sensor> sdcu_device::create_sdcu_depth_device(std::shared_ptr<context> ctx,
        const std::vector<platform::uvc_device_info>& all_device_infos)
    {
        using namespace sdc;

        auto&& backend = ctx->get_backend();

        std::vector<std::shared_ptr<platform::uvc_device>> depth_devices;
        for (auto&& info : filter_by_mi(all_device_infos, 0)) // Filter just mi=0, DEPTH
            depth_devices.push_back(backend.create_uvc_device(info));

        std::unique_ptr<frame_timestamp_reader> sdc_timestamp_reader_backup(new sdc_timestamp_reader(backend.create_time_service()));
        auto depth_ep = std::make_shared<sdcu_depth_sensor>(this, std::make_shared<platform::multi_pins_uvc_device>(depth_devices),
                            std::unique_ptr<frame_timestamp_reader>(new sdc_timestamp_reader_from_metadata(std::move(sdc_timestamp_reader_backup))));
        depth_ep->register_xu(depth_xu); // make sure the XU is initialized every time we power the camera

        depth_ep->register_pixel_format(pf_z16); // Depth

        // Support SDCU-specific pixel format
        depth_ep->register_pixel_format(pf_w10);
        depth_ep->register_pixel_format(pf_uyvyl);

        return depth_ep;
    }

    sdcu_device::sdcu_device(std::shared_ptr<context> ctx,
        const platform::backend_device_group& group)
        : sdc_device(ctx, group), device(ctx, group)
    {
        using namespace sdc;

        // Override the basic sdc sensor with the development version
        _depth_device_idx = assign_sensor(create_sdcu_depth_device(ctx, group.uvc_devices), _depth_device_idx);

        init(ctx, group);

        auto& depth_ep = get_depth_sensor();

        if (!is_camera_in_advanced_mode())
        {
            depth_ep.remove_pixel_format(pf_y8i); // L+R
            depth_ep.remove_pixel_format(pf_y12i); // L+R
        }

        // Inhibit specific unresolved options
		if (_fw_version >= firmware_version("20.11.7.0"))
		{
			depth_ep.unregister_option(RS2_OPTION_OUTPUT_TRIGGER_ENABLED);
			depth_ep.unregister_option(RS2_OPTION_ERROR_POLLING_ENABLED);
			depth_ep.unregister_option(RS2_OPTION_ASIC_TEMPERATURE);
			depth_ep.unregister_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE);
		}

        // Enable laser etc.
        auto pid = group.uvc_devices.front().pid;
        if (pid == SDC30_PID)
        {
            auto& depth_ep = get_depth_sensor();
            auto emitter_enabled = std::make_shared<sdc_emitter_option>(depth_ep);
            depth_ep.register_option(RS2_OPTION_EMITTER_ENABLED, emitter_enabled);

			if (_fw_version >= firmware_version("20.11.7.0"))
			{
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

    processing_blocks get_sdc_depth_recommended_proccesing_blocks()
    {
        auto res = get_depth_recommended_proccesing_blocks();
        res.push_back(std::make_shared<threshold>());
        res.push_back(std::make_shared<disparity_transform>(true));
        res.push_back(std::make_shared<spatial_filter>());
        res.push_back(std::make_shared<temporal_filter>());
        res.push_back(std::make_shared<hole_filling_filter>());
        res.push_back(std::make_shared<disparity_transform>(false));
        return res;
    }

}
