// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once
#include "../../../include/librealsense2/h/rs_advanced_mode_command.h"

namespace librealsense
{
    typedef struct
    {
        float laser_power;
        bool was_set = false;
    }sdc_laser_power_control;

    typedef struct
    {
        int laser_state;
        bool was_set = false;
    }sdc_laser_state_control;

    typedef struct
    {
        float exposure;
        bool was_set = false;
    }sdc_exposure_control;

    typedef struct
    {
        int auto_exposure;
        bool was_set = false;
    }sdc_auto_exposure_control;

    typedef struct
    {
        float gain;
        bool was_set = false;
    }sdc_gain_control;

    typedef struct
    {
        int backlight_compensation;
        bool was_set = false;
    }sdc_backlight_compensation_control;

    typedef struct
    {
        float brightness;
        bool was_set = false;
    }sdc_brightness_control;

    typedef struct
    {
        float contrast;
        bool was_set = false;
    }sdc_contrast_control;

    typedef struct
    {
        float gamma;
        bool was_set = false;
    }sdc_gamma_control;

    typedef struct
    {
        float hue;
        bool was_set = false;
    }sdc_hue_control;

    typedef struct
    {
        float saturation;
        bool was_set = false;
    }sdc_saturation_control;

    typedef struct
    {
        float sharpness;
        bool was_set = false;
    }sdc_sharpness_control;

    typedef struct
    {
        float white_balance;
        bool was_set = false;
    }sdc_white_balance_control;

    typedef struct
    {
        int auto_white_balance;
        bool was_set = false;
    }sdc_auto_white_balance_control;

    typedef struct
    {
        int power_line_frequency;
        bool was_set = false;
    }sdc_power_line_frequency_control;

    struct sdc_preset{
        STDepthControlGroup                depth_controls;
        STRsm                              rsm;
        STRauSupportVectorControl          rsvc;
        STColorControl                     color_control;
        STRauColorThresholdsControl        rctc;
        STSloColorThresholdsControl        sctc;
        STSloPenaltyControl                spc;
        STHdad                             hdad;
        STColorCorrection                  cc;
        STDepthTableControl                depth_table;
        STAEControl                        ae;
        STCensusRadius                     census;
        sdc_laser_state_control            laser_state;
        sdc_laser_power_control            laser_power;
        sdc_exposure_control               depth_exposure;
        sdc_auto_exposure_control          depth_auto_exposure;
        sdc_gain_control                   depth_gain;
        sdc_auto_white_balance_control     depth_auto_white_balance;
        sdc_exposure_control               color_exposure;
        sdc_auto_exposure_control          color_auto_exposure;
        sdc_backlight_compensation_control color_backlight_compensation;
        sdc_brightness_control             color_brightness;
        sdc_contrast_control               color_contrast;
        sdc_gain_control                   color_gain;
        sdc_gamma_control                  color_gamma;
        sdc_hue_control                    color_hue;
        sdc_saturation_control             color_saturation;
        sdc_sharpness_control              color_sharpness;
        sdc_white_balance_control          color_white_balance;
        sdc_auto_white_balance_control     color_auto_white_balance;
        sdc_power_line_frequency_control   color_power_line_frequency;
    };

    void sdc_default(sdc_preset& p);
    void sdc_high_res_high_accuracy(sdc_preset& p);
    void sdc_high_res_high_density(sdc_preset& p);
    void sdc_high_res_mid_density(sdc_preset& p);
    void sdc_low_res_high_accuracy(sdc_preset& p);
    void sdc_low_res_high_density(sdc_preset& p);
    void sdc_low_res_mid_density(sdc_preset& p);
    void sdc_mid_res_high_accuracy(sdc_preset& p);
    void sdc_mid_res_high_density(sdc_preset& p);
    void sdc_mid_res_mid_density(sdc_preset& p);
    void sdc_hand_gesture(sdc_preset& p);
}
