#pragma once

#include "kf_gins_types.h"
#include <common/angle.h>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

class ParamLoader {
public:
    ParamLoader() = default;

    bool load(const std::string &filename);

    std::string imu_topic;
    std::string gnss_origin_topic;
    std::string gnss_pv_topic;
    std::string gi_topic;
    std::string mag_topic;
    //bora
    std::string baro_topic;

    //range
    std::string range_topic;

    int ca_timelen;
    double gnss_timelen;
    double calc_timelen;

    int nav_pub_rate;
    int precisionEchoEpoch;
    int imudatarate;

    int zupt_timelength;
    Eigen::Vector3d velstd;
    //baro
    double baro_std;
    //range
    Eigen::Vector3d range_std;
    double omega_threshold;
    // gnss
    Eigen::Vector3d gnss_pos_std;
    //range ID Eigen::Vector3i 
    Eigen::Vector3d range_id_1_3;
    Eigen::Vector3d range_id_4_6;
    Eigen::Vector3d range_id_7_9;
    Eigen::Vector3d range_id_10_12;
    Eigen::Vector3d anchor_id_1_3;
    
    bool is_baro_using;
    bool is_baro_using_kf;
    bool is_range_using;
    bool is_mag_yaw_using;
};

extern std::shared_ptr<ParamLoader> pload;
