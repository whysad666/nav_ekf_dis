#pragma once

#include <Eigen/Dense>
#include <vector>

#include "common/types.h"

#include "kf_gins_types.h"

namespace ImuCommon {
IMU imuCalibration(const IMU &imu, const ImuCalib &imucalib_);

IMU imuCompensate(const IMU &imu, const ImuError &imuerror_);

IMU imuSummation(const IMU &imu1, const IMU &imu2);
} // namespace ImuCommon
