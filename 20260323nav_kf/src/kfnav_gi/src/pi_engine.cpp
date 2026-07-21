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

#include "common/earth.h"
#include "common/rotation.h"
#include "imu_common.h"

#include "inscalc.h"
#include "pi_engine.h"

PIEngine::PIEngine(const NavState &initstate) {
    // 设置系统状态(位置、速度、姿态和IMU误差)初值和初始协方差
    // set initial state (position, velocity, attitude and IMU error) and covariance
    initialize(initstate);
}

void PIEngine::initialize(const NavState &initstate) {
    raw_sn = initstate.raw_sn;
    timestamp_ = initstate.timestamp;

    // 初始化位置、速度、姿态
    // initialize position, velocity and attitude
    pvacur_.pos = initstate.pos;
    pvacur_.vel = initstate.vel;
    pvacur_.att.euler = initstate.euler;
    pvacur_.att.cbn = Rotation::euler2matrix(pvacur_.att.euler);
    pvacur_.att.qbn = Rotation::euler2quaternion(pvacur_.att.euler);
    // 初始化IMU误差
    // initialize imu error
    imuerror_ = initstate.imuerror;
}

NavState PIEngine::newImuProcess(const IMU &imu1_in, const IMU &imu2_in) {
    timestamp_ = std::max(imu1_in.time, imu2_in.time);
    raw_sn = std::max(imu1_in.raw_sn, imu2_in.raw_sn);

    // 只传播导航状态
    // only propagate navigation state
    insPropagation(imu2_in, imu2_in);

    return getNavState();
}

void PIEngine::insPropagation(const IMU &imu1_in, const IMU &imu2_in) {

    IMU imu1, imu2;
    // 补偿IMU零偏
    // compensate the imu bias
    imu1 = ImuCommon::imuCompensate(imu1_in, imuerror_);
    imu2 = ImuCommon::imuCompensate(imu2_in, imuerror_);

    // IMU状态更新(机械编排算法)
    // update imustate(mechanization)
    INSCalc::calc(pvacur_, imu1, imu2, 0.5 * (imu1.dt + imu2.dt), INSCalc::INSMode::Normal);
}

NavState PIEngine::getNavState() {

    NavState state;

    state.pos = pvacur_.pos;
    state.vel = pvacur_.vel;
    state.euler = pvacur_.att.euler;
    state.imuerror = imuerror_;

    return state;
}
