// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved.

#include "sdc-options.h"

namespace librealsense
{
    const char* sdc_emitter_option::get_value_description(float val) const
    {
        switch (static_cast<int>(val))
        {
            case 0:
            {
                return "Off";
            }
            case 1:
            {
                return "On";
            }
            case 2:
            {
                return "Auto";
            }
            default:
                throw invalid_value_exception("value not found");
        }
    }

    sdc_emitter_option::sdc_emitter_option(uvc_sensor& ep)
        : uvc_xu_option(ep, sdc::depth_xu, sdc::SDC_DEPTH_EMITTER_ENABLED,
                        "Power Control for SDC30 Projector, 0-off, 1-on, (2-deprecated)")
    {}

    float sdc_asic_and_projector_temperature_options::query() const
    {
        if (!is_enabled())
            throw wrong_api_call_sequence_exception("query option is allow only in streaming!");

        #pragma pack(push, 1)
        struct temperature
        {
            uint8_t is_projector_valid;
            uint8_t is_asic_valid;
            int8_t projector_temperature;
            int8_t asic_temperature;
        };
        #pragma pack(pop)

        auto temperature_data = static_cast<temperature>(_ep.invoke_powered(
            [this](platform::uvc_device& dev)
            {
                temperature temp{};
                if (!dev.get_xu(sdc::depth_xu,
                                sdc::SDC_ASIC_AND_PROJECTOR_TEMPERATURES,
                                reinterpret_cast<uint8_t*>(&temp),
                                sizeof(temperature)))
                 {
                        throw invalid_value_exception(to_string() << "get_xu(ctrl=SDC_ASIC_AND_PROJECTOR_TEMPERATURES) failed!" << " Last Error: " << strerror(errno));
                 }

                return temp;
            }));

        int8_t temperature::* field;
        uint8_t temperature::* is_valid_field;

        switch (_option)
        {
        case RS2_OPTION_ASIC_TEMPERATURE:
            field = &temperature::asic_temperature;
            is_valid_field = &temperature::is_asic_valid;
            break;
        case RS2_OPTION_PROJECTOR_TEMPERATURE:
            field = &temperature::projector_temperature;
            is_valid_field = &temperature::is_projector_valid;
            break;
        default:
            throw invalid_value_exception(to_string() << _ep.get_option_name(_option) << " is not temperature option!");
        }

        if (0 == temperature_data.*is_valid_field)
            LOG_ERROR(_ep.get_option_name(_option) << " value is not valid!");

        return temperature_data.*field;
    }

    option_range sdc_asic_and_projector_temperature_options::get_range() const
    {
        return option_range { -40, 125, 0, 0 };
    }

    bool sdc_asic_and_projector_temperature_options::is_enabled() const
    {
        return _ep.is_streaming();
    }

    const char* sdc_asic_and_projector_temperature_options::get_description() const
    {
        switch (_option)
        {
        case RS2_OPTION_ASIC_TEMPERATURE:
            return "Current Asic Temperature (degree celsius)";
        case RS2_OPTION_PROJECTOR_TEMPERATURE:
            return "Current Projector Temperature (degree celsius)";
        default:
            throw invalid_value_exception(to_string() << _ep.get_option_name(_option) << " is not temperature option!");
        }
    }

	sdc_asic_and_projector_temperature_options::sdc_asic_and_projector_temperature_options(uvc_sensor& ep, rs2_option opt)
        : _option(opt), _ep(ep)
        {}

    float sdc_motion_module_temperature_option::query() const
    {
        if (!is_enabled())
            throw wrong_api_call_sequence_exception("query option is allow only in streaming!");

        static const auto report_field = platform::custom_sensor_report_field::value;
        auto data = _ep.get_custom_report_data(custom_sensor_name, report_name, report_field);
        if (data.empty())
            throw invalid_value_exception("query() motion_module_temperature_option failed! Empty buffer arrived.");

        auto data_str = std::string(reinterpret_cast<char const*>(data.data()));
        return std::stof(data_str);
    }

    option_range sdc_motion_module_temperature_option::get_range() const
    {
        if (!is_enabled())
            throw wrong_api_call_sequence_exception("get option range is allow only in streaming!");

        static const auto min_report_field = platform::custom_sensor_report_field::minimum;
        static const auto max_report_field = platform::custom_sensor_report_field::maximum;
        auto min_data = _ep.get_custom_report_data(custom_sensor_name, report_name, min_report_field);
        auto max_data = _ep.get_custom_report_data(custom_sensor_name, report_name, max_report_field);
        if (min_data.empty() || max_data.empty())
            throw invalid_value_exception("get_range() motion_module_temperature_option failed! Empty buffer arrived.");

        auto min_str = std::string(reinterpret_cast<char const*>(min_data.data()));
        auto max_str = std::string(reinterpret_cast<char const*>(max_data.data()));

        return option_range{std::stof(min_str),
                            std::stof(max_str),
                            0, 0};
    }

    bool sdc_motion_module_temperature_option::is_enabled() const
    {
        return _ep.is_streaming();
    }

    const char* sdc_motion_module_temperature_option::get_description() const
    {
        return "Current Motion-Module Temperature (degree celsius)";
    }

	sdc_motion_module_temperature_option::sdc_motion_module_temperature_option(hid_sensor& ep)
        : _ep(ep)
    {}

    void sdc_enable_motion_correction::set(float value)
    {
        if (!is_valid(value))
            throw invalid_value_exception(to_string() << "set(enable_motion_correction) failed! Given value " << value << " is out of range.");

        _is_enabled = value > _opt_range.min;
        _recording_function(*this);
    }

    float sdc_enable_motion_correction::query() const
    {
        auto is_enabled = _is_enabled.load();
        return is_enabled ? _opt_range.max : _opt_range.min;
    }

	sdc_enable_motion_correction::sdc_enable_motion_correction(sensor_base* mm_ep,
                                                       const sdc::imu_intrinsic& accel,
                                                       const sdc::imu_intrinsic& gyro,
                                                       std::shared_ptr<librealsense::lazy<rs2_extrinsics>> depth_to_imu,
                                                       on_before_frame_callback frame_callback,
                                                       const option_range& opt_range)
        : option_base(opt_range), _is_enabled(true), _accel(accel), _gyro(gyro), _depth_to_imu(**depth_to_imu)
    {
        mm_ep->register_on_before_frame_callback(
            [this, frame_callback](rs2_stream stream, frame_interface* fr, callback_invocation_holder callback)
            {
                if (_is_enabled.load() && fr->get_stream()->get_format() == RS2_FORMAT_MOTION_XYZ32F)
                {
                    auto xyz = (float3*)(fr->get_frame_data());

                    if (stream == RS2_STREAM_ACCEL)
                        *xyz = (_accel.sensitivity * (*xyz)) - _accel.bias;

                    if (stream == RS2_STREAM_GYRO)
                        *xyz = _gyro.sensitivity * (*xyz) - _gyro.bias;
                }

                // Align IMU axes to the established Coordinates System
                if (frame_callback)
                    frame_callback(stream, fr, std::move(callback));
            });
    }

    void sdc_enable_auto_exposure_option::set(float value)
    {
        if (!is_valid(value))
            throw invalid_value_exception("set(enable_auto_exposure) failed! Invalid Auto-Exposure mode request " + std::to_string(value));

        auto auto_exposure_prev_state = _auto_exposure_state->get_enable_auto_exposure();
        _auto_exposure_state->set_enable_auto_exposure(0.f < std::fabs(value));

        if (_auto_exposure_state->get_enable_auto_exposure()) // auto_exposure current value
        {
            if (!auto_exposure_prev_state) // auto_exposure previous value
            {
                _to_add_frames = true; // auto_exposure moved from disable to enable
            }
        }
        else
        {
            if (auto_exposure_prev_state)
            {
                _to_add_frames = false; // auto_exposure moved from enable to disable
            }
        }
        _recording_function(*this);
    }

    float sdc_enable_auto_exposure_option::query() const
    {
        return _auto_exposure_state->get_enable_auto_exposure();
    }

	sdc_enable_auto_exposure_option::sdc_enable_auto_exposure_option(uvc_sensor* fisheye_ep,
                                                             std::shared_ptr<auto_exposure_mechanism> auto_exposure,
                                                             std::shared_ptr<auto_exposure_state> auto_exposure_state,
                                                             const option_range& opt_range)
        : option_base(opt_range),
          _auto_exposure_state(auto_exposure_state),
          _to_add_frames((_auto_exposure_state->get_enable_auto_exposure())),
          _auto_exposure(auto_exposure)
    {
        fisheye_ep->register_on_before_frame_callback(
                    [this](rs2_stream stream, frame_interface* f, callback_invocation_holder callback)
        {
            if (!_to_add_frames || stream != RS2_STREAM_FISHEYE)
                return;

            ((frame*)f)->additional_data.fisheye_ae_mode = true;

            f->acquire();
            _auto_exposure->add_frame(f, std::move(callback));
        });
    }

	sdc_auto_exposure_mode_option::sdc_auto_exposure_mode_option(std::shared_ptr<auto_exposure_mechanism> auto_exposure,
                                                         std::shared_ptr<auto_exposure_state> auto_exposure_state,
                                                         const option_range& opt_range,
                                                         const std::map<float, std::string>& description_per_value)
        : option_base(opt_range),
          _auto_exposure_state(auto_exposure_state),
          _auto_exposure(auto_exposure),
          _description_per_value(description_per_value)
    {}

    void sdc_auto_exposure_mode_option::set(float value)
    {
        if (!is_valid(value))
            throw invalid_value_exception(to_string() << "set(auto_exposure_mode_option) failed! Given value " << value << " is out of range.");

        _auto_exposure_state->set_auto_exposure_mode(static_cast<auto_exposure_modes>((int)value));
        _auto_exposure->update_auto_exposure_state(*_auto_exposure_state);
        _recording_function(*this);
    }

    float sdc_auto_exposure_mode_option::query() const
    {
        return static_cast<float>(_auto_exposure_state->get_auto_exposure_mode());
    }

    const char* sdc_auto_exposure_mode_option::get_value_description(float val) const
    {
        try{
            return _description_per_value.at(val).c_str();
        }
        catch(std::out_of_range)
        {
            throw invalid_value_exception(to_string() << "auto_exposure_mode: get_value_description(...) failed! Description of value " << val << " is not found.");
        }
    }

	sdc_auto_exposure_step_option::sdc_auto_exposure_step_option(std::shared_ptr<auto_exposure_mechanism> auto_exposure,
        std::shared_ptr<auto_exposure_state> auto_exposure_state,
        const option_range& opt_range)
        : option_base(opt_range),
        _auto_exposure_state(auto_exposure_state),
        _auto_exposure(auto_exposure)
    {}

    void sdc_auto_exposure_step_option::set(float value)
    {
        if (!std::isnormal(_opt_range.step) || ((value < _opt_range.min) || (value > _opt_range.max)))
            throw invalid_value_exception(to_string() << "set(auto_exposure_step_option) failed! Given value " << value << " is out of range.");

        _auto_exposure_state->set_auto_exposure_step(value);
        _auto_exposure->update_auto_exposure_state(*_auto_exposure_state);
        _recording_function(*this);
    }

    float sdc_auto_exposure_step_option::query() const
    {
        return static_cast<float>(_auto_exposure_state->get_auto_exposure_step());
    }

	sdc_auto_exposure_antiflicker_rate_option::sdc_auto_exposure_antiflicker_rate_option(std::shared_ptr<auto_exposure_mechanism> auto_exposure,
                                                                                 std::shared_ptr<auto_exposure_state> auto_exposure_state,
                                                                                 const option_range& opt_range,
                                                                                 const std::map<float, std::string>& description_per_value)
        : option_base(opt_range),
          _auto_exposure_state(auto_exposure_state),
          _auto_exposure(auto_exposure),
          _description_per_value(description_per_value)
    {}

    void sdc_auto_exposure_antiflicker_rate_option::set(float value)
    {
        if (!is_valid(value))
            throw invalid_value_exception(to_string() << "set(auto_exposure_antiflicker_rate_option) failed! Given value " << value << " is out of range.");

        _auto_exposure_state->set_auto_exposure_antiflicker_rate(static_cast<uint32_t>(value));
        _auto_exposure->update_auto_exposure_state(*_auto_exposure_state);
        _recording_function(*this);
    }

    float sdc_auto_exposure_antiflicker_rate_option::query() const
    {
        return static_cast<float>(_auto_exposure_state->get_auto_exposure_antiflicker_rate());
    }

    const char* sdc_auto_exposure_antiflicker_rate_option::get_value_description(float val) const
    {
        try{
            return _description_per_value.at(val).c_str();
        }
        catch(std::out_of_range)
        {
            throw invalid_value_exception(to_string() << "antiflicker_rate: get_value_description(...) failed! Description of value " << val << " is not found.");
        }
    }

    sdc::depth_table_control sdc_depth_scale_option::get_depth_table(sdc::advanced_query_mode mode) const
    {
        command cmd(sdc::GET_ADV);
        cmd.param1 = sdc::etDepthTableControl;
        cmd.param2 = mode;
        auto res = _hwm.send(cmd);

        if (res.size() < sizeof(sdc::depth_table_control))
            throw std::runtime_error("Not enough bytes returned from the firmware!");

        auto table = (const sdc::depth_table_control*)res.data();
        return *table;
    }

	sdc_depth_scale_option::sdc_depth_scale_option(hw_monitor& hwm)
        : _hwm(hwm)
    {
        _range = [this]()
        {
            auto min = get_depth_table(sdc::GET_MIN);
            auto max = get_depth_table(sdc::GET_MAX);
            return option_range{ (float)(0.000001 * min.depth_units),
                                 (float)(0.000001 * max.depth_units),
                                 0.000001f, 0.001f };
        };
    }

    void sdc_depth_scale_option::set(float value)
    {
        command cmd(sdc::SET_ADV);
        cmd.param1 = sdc::etDepthTableControl;

        auto depth_table = get_depth_table(sdc::GET_VAL);
        depth_table.depth_units = static_cast<uint32_t>(1000000 * value);
        auto ptr = (uint8_t*)(&depth_table);
        cmd.data = std::vector<uint8_t>(ptr, ptr + sizeof(sdc::depth_table_control));

        _hwm.send(cmd);
        _record_action(*this);
        notify(value);
    }

    float sdc_depth_scale_option::query() const
    {
        auto table = get_depth_table(sdc::GET_VAL);
        return (float)(0.000001 * (float)table.depth_units);
    }

    option_range sdc_depth_scale_option::get_range() const
    {
        return *_range;
    }

	sdc_external_sync_mode::sdc_external_sync_mode(hw_monitor& hwm)
        : _hwm(hwm)
    {
        _range = [this]()
        {
            return option_range{ sdc::inter_cam_sync_mode::INTERCAM_SYNC_DEFAULT,
                                 sdc::inter_cam_sync_mode::INTERCAM_SYNC_MAX - 1,
                                 sdc::inter_cam_sync_mode::INTERCAM_SYNC_DEFAULT, 1 };
        };
    }

    void sdc_external_sync_mode::set(float value)
    {
        command cmd(sdc::SET_CAM_SYNC);
        cmd.param1 = static_cast<int>(value);

        _hwm.send(cmd);
        _record_action(*this);
    }

    float sdc_external_sync_mode::query() const
    {
        command cmd(sdc::GET_CAM_SYNC);
        auto res = _hwm.send(cmd);
        if (res.empty())
            throw invalid_value_exception("external_sync_mode::query result is empty!");

        return (res.front());
    }

    option_range sdc_external_sync_mode::get_range() const
    {
        return *_range;
    }

	sdc_emitter_on_and_off_option::sdc_emitter_on_and_off_option(hw_monitor& hwm, sensor_base* ep)
        : _hwm(hwm), _sensor(ep)
    {
        _range = [this]()
        {
            return option_range{ 0, 1, 1, 0 };
        };
    }

    void sdc_emitter_on_and_off_option::set(float value)
    {
        if (_sensor->is_streaming())
            throw std::runtime_error("Cannot change Emitter On/Off option while streaming!");

        command cmd(sdc::SET_PWM_ON_OFF);
        cmd.param1 = static_cast<int>(value);

        _hwm.send(cmd);
        _record_action(*this);
    }

    float sdc_emitter_on_and_off_option::query() const
    {
        command cmd(sdc::GET_PWM_ON_OFF);
        auto res = _hwm.send(cmd);
        if (res.empty())
            throw invalid_value_exception("emitter_on_and_off_option::query result is empty!");

        return (res.front());
    }

    option_range sdc_emitter_on_and_off_option::get_range() const
    {
        return *_range;
    }

	sdc_alternating_emitter_option::sdc_alternating_emitter_option(hw_monitor& hwm, sensor_base* ep)
        : _hwm(hwm), _sensor(ep)
    {
        _range = [this]()
        {
            return option_range{ 0, 1, 1, 0 };
        };
    }

    void sdc_alternating_emitter_option::set(float value)
    {
        std::vector<uint8_t> pattern{};
        if (static_cast<int>(value))
            pattern = sdc::alternating_emitter_pattern;

        command cmd(sdc::SETSUBPRESET, static_cast<int>(pattern.size()));
        cmd.data = pattern;
        auto res = _hwm.send(cmd);
        _record_action(*this);
    }

    float sdc_alternating_emitter_option::query() const
    {
        command cmd(sdc::GETSUBPRESETNAME);
        auto res = _hwm.send(cmd);
        if (res.size()>20)
            throw invalid_value_exception("HWMON::GETSUBPRESETNAME invalid size");

        static std::vector<uint8_t> alt_emitter_name(sdc::alternating_emitter_pattern.begin()+2,sdc::alternating_emitter_pattern.begin()+22);
        return (alt_emitter_name == res);
    }
}
