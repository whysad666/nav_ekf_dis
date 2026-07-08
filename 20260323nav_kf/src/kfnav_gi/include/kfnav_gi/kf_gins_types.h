/*
 * KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Liqiang Wang
 *    Contact : wlq@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef KF_GINS_TYPES_H
#define KF_GINS_TYPES_H

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <param_common.h>
#include <yaml-cpp/yaml.h>

#include "common/angle.h"
#include "common/earth.h"

const double g0 = 9.80665;

using Attitude = struct Attitude {
    Eigen::Quaterniond qbn;
    Eigen::Matrix3d cbn;
    Eigen::Vector3d euler;
};

using PVA = struct PVA {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Attitude att;
};

using ImuError = struct ImuError {
    Eigen::Vector3d gyrbias;
    Eigen::Vector3d accbias;
    Eigen::Vector3d gyrscale;
    Eigen::Vector3d accscale;

    ImuError() = default;
    ImuError(YAML::Node node) {
        // gyro bias
        gyrbias = ParamCommon::getVector3d(node["gyro_bias"], Eigen::Vector3d::Zero()) * DH2RS;
        // accl bias
        accbias = ParamCommon::getVector3d(node["accl_bias"], Eigen::Vector3d::Zero()) * g0 * 1e-6;
        // gyro scale
        gyrscale = ParamCommon::getVector3d(node["gyro_scale"], Eigen::Vector3d::Zero()) / 1e6;
        // accl scale
        accscale = ParamCommon::getVector3d(node["accl_scale"], Eigen::Vector3d::Zero()) / 1e6;
    }
    void expr() {
        std::cout << '\t' << "- gyrbias : " << gyrbias.transpose() * RS2DH << " [deg/h] " << std::endl;
        std::cout << '\t' << "- accbias : " << accbias.transpose() / (g0 * 1e-6) << " [ug] " << std::endl;
        std::cout << '\t' << "- gyrscale: " << gyrscale.transpose() * 1.0e6 << " [ppm] " << std::endl;
        std::cout << '\t' << "- accscale: " << accscale.transpose() * 1.0e6 << " [ppm] " << std::endl;
    }
};

using ImuCalib = struct ImuCalib {
    Eigen::Matrix3d Kgyro;
    Eigen::Matrix3d Kaccl;
    Eigen::Vector3d bias_gyro;
    Eigen::Vector3d bias_accl;

    ImuCalib() = default;
    ImuCalib(YAML::Node node) {
        Kgyro = ParamCommon::getMatrix3d(node["k_g"], Eigen::Matrix3d::Identity());
        Kaccl = ParamCommon::getMatrix3d(node["k_a"], Eigen::Matrix3d::Identity());
        bias_gyro = ParamCommon::getVector3d(node["b_g"], Eigen::Vector3d::Zero());
        bias_accl = ParamCommon::getVector3d(node["b_a"], Eigen::Vector3d::Zero());
    }
    void expr() {
        std::cout << " --   K_g               :\n" << Kgyro << std::endl;
        std::cout << " --   K_a               :\n" << Kaccl << std::endl;
        std::cout << " --   bias_gyro         :" << bias_gyro.transpose() << std::endl;
        std::cout << " --   bias_accl         :" << bias_accl.transpose() << std::endl;
        std::cout << std::endl;
    }
};

using NavState = struct NavState {
    // timestamp 表示导航状态使用的最后一帧IMU数据的时间戳
    uint64_t timestamp;
    // utc_ns 表示导航状态使用的最后一帧GNSS数据的UTC时间戳
    uint64_t utc_ns;
    // raw_sn 表示导航状态使用的最后一帧IMU数据的原始时间戳*200，相当于惯导序列号
    uint64_t raw_sn; // 原始时间戳*200，作为序列号使用
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d euler;

    ImuError imuerror;

    NavState() = default;
    NavState(YAML::Node node) {
        // nav state
        euler = ParamCommon::getVector3d(node["euler"], Eigen::Vector3d::Zero());
        vel = ParamCommon::getVector3d(node["velocity"], Eigen::Vector3d::Zero());
        pos = ParamCommon::getVector3d(node["position"], Eigen::Vector3d::Zero());
        imuerror = ImuError(node["imu_error"]);
    }
};

using ImuRw = struct ImuRw {
    Eigen::Vector3d gyr_arw;
    Eigen::Vector3d acc_vrw;
    Eigen::Vector3d bias_gyr_rw;
    Eigen::Vector3d bias_acc_rw;
    Eigen::Vector3d scale_gyr_rw;
    Eigen::Vector3d scale_acc_rw;
    double corr_time;

    ImuRw() = default;
    ImuRw(YAML::Node node) {
        // angular random walk
        // [deg/sqrt(hr)] -> [rad/sqrt(s)]
        gyr_arw = ParamCommon::getVector3d(node["gyr_arw"], Eigen::Vector3d::Zero()) * D2R / 60;
        // velocity random walk
        // [μg/sqrt(Hz)] -> [m/s/sqrt(s)]
        acc_vrw = ParamCommon::getVector3d(node["acc_vrw"], Eigen::Vector3d::Zero()) * g0 * 1.0e-6;

        bias_gyr_rw = ParamCommon::getVector3d(node["bias_gyr_rw"], Eigen::Vector3d::Zero()) * DH2RS / 60;
        bias_acc_rw = ParamCommon::getVector3d(node["bias_acc_rw"], Eigen::Vector3d::Zero()) * g0 * 1e-6 / 60;
        scale_gyr_rw = ParamCommon::getVector3d(node["scale_gyr_rw"], Eigen::Vector3d::Zero()) / 1e6 / 60;
        scale_acc_rw = ParamCommon::getVector3d(node["scale_acc_rw"], Eigen::Vector3d::Zero()) / 1e6 / 60;
        corr_time = ParamCommon::getDouble(node["corr_time"], 36000.0);
    }
    void expr() {
        std::cout << " -- gyr_arw       :" << gyr_arw.transpose() * R2D * 60 << "[deg/sqrt(hr)]" << std::endl;
        std::cout << " -- acc_vrw       :" << acc_vrw.transpose() / g0 * 1e6 << "[μg/sqrt(Hz)]" << std::endl;
        std::cout << " -- bias_gyr_rw   :" << bias_gyr_rw.transpose() * RS2DH * 60 << "[deg/hr/sqrt(hr)]" << std::endl;
        std::cout << " -- bias_acc_rw   :" << bias_acc_rw.transpose() / g0 * 1e6 * 60 << "[μg/sqrt(hr)]" << std::endl;
        std::cout << " -- scale_gyr_rw  :" << scale_gyr_rw.transpose() * 1e6 * 60 << "[ppm/sqrt(hr)]" << std::endl;
        std::cout << " -- scale_acc_rw  :" << scale_acc_rw.transpose() * 1e6 * 60 << "[ppm/sqrt(hr)]" << std::endl;
        std::cout << " -- corr_time     :" << corr_time << "[s]" << std::endl;
        std::cout << std::endl;
    }
};

using MeasureNoise = struct MeasureNoise {
    Eigen::Vector3d phi;
    Eigen::Vector3d vel;
    Eigen::Vector3d pos;
};

using KfNoise = struct KfNoise {
    ImuRw prop_noise;
    MeasureNoise meas_noise;
};

using KFOptions = struct KFOptions {

    // 初始状态和状态标准差
    // initial state and state standard deviation
    NavState initstate;
    NavState initstate_std;

    ImuRw prop_noise;

    void print_options() {
        std::cout << "---------------KF-GINS Options:---------------" << std::endl;

        // 打印初始状态
        // print initial state
        std::cout << " - Initial State: " << std::endl;
        std::cout << '\t' << "- initial position: ";
        std::cout << std::setprecision(12) << initstate.pos[0] * R2D << "  ";
        std::cout << std::setprecision(12) << initstate.pos[1] * R2D << "  ";
        std::cout << std::setprecision(6) << initstate.pos[2] << " [deg, deg, m] " << std::endl;
        std::cout << '\t' << "- initial velocity: " << initstate.vel.transpose() << " [m/s] " << std::endl;
        std::cout << '\t' << "- initial attitude: " << initstate.euler.transpose() * R2D << " [deg] " << std::endl;
        initstate.imuerror.expr();

        // 打印初始状态标准差
        // print initial state STD
        std::cout << " - Initial State STD: " << std::endl;
        std::cout << '\t' << "- initial position std: " << initstate_std.pos.transpose() << " [m] " << std::endl;
        std::cout << '\t' << "- initial velocity std: " << initstate_std.vel.transpose() << " [m/s] " << std::endl;
        std::cout << '\t' << "- initial attitude std: " << initstate_std.euler.transpose() << " [deg] " << std::endl;
        initstate_std.imuerror.expr();

        // 打印IMU噪声参数
        // print IMU noise parameters
        std::cout << " - IMU Random Walk: " << std::endl;
        prop_noise.expr();

        std::cout << std::endl;
    }
};

#endif // KF_GINS_TYPES_H
