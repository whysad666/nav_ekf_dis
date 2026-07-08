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

#pragma once

#include <Eigen/Dense>
#include <vector>

#include "common/types.h"

#include "kf_gins_types.h"

class PIEngine {

public:
    explicit PIEngine(const NavState &initstate);
    PIEngine() = default;

    ~PIEngine() = default;

    /**
     * @brief 初始化系统状态和协方差
     *        initialize state and state covariance
     * @param [in] initstate     初始状态
     *                           initial state
     * @param [in] initstate_std 初始状态标准差
     *                           initial state std
     * */
    void initialize(const NavState &initstate);

    /**
     * @brief 处理新的IMU数据
     *        process new imudata
     * */
    NavState newImuProcess(const IMU &imu1_in, const IMU &imu2_in);

    /**
     * @brief 获取当前时间
     *        get current time
     * */
    uint64_t timestamp() const {
        return timestamp_;
    }
    uint64_t raw_imu_sn() const {
        return raw_sn;
    }

    /**
     * @brief 获取当前IMU状态
     *        get current navigation state
     * */
    NavState getNavState();

private:
    /**
     * @brief 进行INS状态更新(IMU机械编排算法), 并计算IMU状态转移矩阵和噪声阵
     *        do INS state update(INS mechanization), and compute state transition matrix and noise matrix
     * @param [in,out] imupre 前一时刻IMU数据
     *                        imudata at the previous epoch
     * @param [in,out] imucur 当前时刻IMU数据
     *                        imudata at the current epoch
     * */
    void insPropagation(const IMU &imu1_in, const IMU &imu2_in);

private:
    uint64_t timestamp_;
    uint64_t raw_sn; // 原始时间戳*200，作为序列号使用

    // IMU状态（位置、速度、姿态和IMU误差）
    // imu state (position, velocity, attitude and imu error)
    PVA pvacur_;
    ImuError imuerror_;
};
