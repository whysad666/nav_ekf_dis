#pragma once

#include <Eigen/Dense>
#include <common/rotation.h>
#include <kf_gins_types.h>
#include <vector>

// #define TWO_SUBSAMPLE

class CoarseAlignI0 {
public:
    CoarseAlignI0() {
        init();
    }
    CoarseAlignI0(const double lati, const double alti, const double sampt0) {
        init();
        set(lati, alti, sampt0);
    }
    ~CoarseAlignI0() = default;

    void init() {
        cnt = 0;
        vel_cnt = 0;
        vib0_vec.clear();
        // 初始化累计状态
        qbib0 = Eigen::Quaterniond::Identity();
        Cbib0 = Eigen::Matrix3d::Identity();
        vib0 = Eigen::Vector3d::Zero();
    }

    void set(const double lati, const double alti, const double sampt0) {
        Lati = lati;
        Alti = alti;
        Sampt0 = sampt0;
        is_ready = true;
    }

    // 计算初始姿态
    auto calc() -> Attitude;

    //单子样惯导更新
    void update(const IMU &imu1);
    //双子样惯导更新
    void update(const IMU &imu1, const IMU &imu2);

    auto dv2att(const Eigen::Vector3d &vn1, const Eigen::Vector3d &vn2, const Eigen::Vector3d &vd1,
                const Eigen::Vector3d &vd2) -> Eigen::Matrix3d;

private:
    bool is_ready{false};
    auto Vi_calc(const double t) -> Eigen::Vector3d;
    // 参数
    double Lati, Alti;
    double Sampt0;
    // 状态
    Eigen::Quaterniond qbib0;
    Eigen::Matrix3d Cbib0;
    Eigen::Vector3d vib0;
    // 计数器
    uint64_t cnt; // 计算过的惯导帧数
    uint64_t vel_cnt;
    // 累积速度积分量
    std::vector<Eigen::Vector3d> vib0_vec;
};
