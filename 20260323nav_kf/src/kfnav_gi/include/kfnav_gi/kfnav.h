#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <chrono>
#include <deque>
#include <iomanip>
#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fileio/filebase.h>
#include <fileio/fileloader.h>
#include <fileio/filesaver.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <common/angle.h>
#include <common/earth.h>
#include <mutex>
#include <sema_deque.h>

#include <kf_gins_types.h>

#include <pi_engine.h>
#include <inspv_stekf_15.h>

#include <datagram.h>
#include <socket_send.h>

#include <coarse_align_i0.h>
#include <coarse_align_compass.h>
#include <cmath>


class KfNav {
public:
    static const uint64_t IMU_LOG_MAX = 100000; // 200*500, 允许存储500S的惯性数据
    using NavPubPtr = ros::Publisher;

    explicit KfNav();
    ~KfNav() = default;

    void gnss_ins_thread();
    void realtime_ins_thread();

    void callback_inertial(std::vector<double> data);
    auto waitIMU_main() -> IMU;
    auto waitIMU_rt() -> IMU;

    void callback_gnss_origin(std::vector<double> data);
    auto waitGNSS_Origin() -> GNSS;

    void callback_mag(std::vector<double> data);
    auto magYawInWindow(uint64_t start_time, uint64_t end_time, const Eigen::Vector3d &roll_pitch_yaw,
                        double &yaw) -> bool;

    void callback_baro(std::vector<double> data);
    auto waitBARO() -> BARO;

    //range
    void callback_range(std::vector<double> data);
    auto waitRANGE() -> RANGE;

    void publishNavState(const NavState &nav_state);
    void dispVector(std::string title, const Eigen::Vector3d &vec);
    void dispNavState(std::string title, const NavState &nav_state);

protected:
    void writeNavResult(uint64_t update_cnt, uint64_t predict_cnt, const NavState &navstate, const GNSS &gnss,
                        const Eigen::MatrixXd &cov, double omega_rms);

    void sendNavState(const NavState &nav_state);
   

private:
    YAML::Node config;

    ImuCalib imu_calib;

    uint64_t align_start_time = UINT64_MAX;
    uint64_t ca_end_time = 0;

    boost::interprocess::interprocess_semaphore state0_sem;
    std::mutex state0_mutex;
    NavState state0;
    uint64_t state0_update_cnt;

    NavPubPtr gi_pub;
    socket_send context;

    double m_time_last;
    int is_redirect_from_file;
    bool is_gnss_using;
    FileSaver fs; // 用于保存导航轨迹的文件

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // IMU
    SemaDeque<IMU> main_imu_buf;
    SemaDeque<IMU> rt_imu_buf;
    ros::Subscriber imu_sub;

    //baro
    SemaDeque<BARO> baro_buf;
    ros::Subscriber baro_sub;

    // Magnetometer
    SemaDeque<MAG> mag_buf;
    ros::Subscriber mag_sub;

    //range
    SemaDeque<RANGE> range_buf;
    ros::Subscriber range_sub;
    

    // GNSS Origin
    SemaDeque<GNSS> gnss_origin_buf;
    ros::Subscriber gnss_origin_sub;
};
