#include <imu_common.h>

namespace ImuCommon {
IMU imuCalibration(const IMU &imu, const ImuCalib &imucalib_) {
    // 补偿标定参数
    IMU ret = imu;

    ret.dvel = imucalib_.Kaccl * imu.dvel - imucalib_.bias_accl * imu.dt;
    ret.dtheta = imucalib_.Kgyro * (imu.dtheta - imucalib_.bias_gyro * imu.dt);
    return ret;
}

IMU imuCompensate(const IMU &imu, const ImuError &imuerror_) {
    // 补偿滤波结果
    IMU ret = imu;

    // 补偿IMU零偏
    // compensate the imu bias
    ret.dtheta = imu.dtheta - imuerror_.gyrbias * imu.dt;
    ret.dvel = imu.dvel - imuerror_.accbias * imu.dt;

    // 补偿IMU比例因子
    // compensate the imu scale
    Eigen::Vector3d gyrscale, accscale;
    gyrscale = Eigen::Vector3d::Ones() - imuerror_.gyrscale;
    accscale = Eigen::Vector3d::Ones() - imuerror_.accscale;
    ret.dtheta = ret.dtheta.cwiseProduct(gyrscale);
    ret.dvel = ret.dvel.cwiseProduct(accscale);
    return ret;
}

IMU imuSummation(const IMU &imu1, const IMU &imu2) {
    IMU ret = imu2;
    ret.time = std::max(imu1.time, imu2.time);
    ret.dt = imu1.dt + imu2.dt;
    ret.frame_delta = imu1.frame_delta + imu2.frame_delta;
    ret.lost_frames = imu1.lost_frames + imu2.lost_frames;
    ret.has_gap = imu1.has_gap || imu2.has_gap;
    ret.dtheta = imu1.dtheta + imu2.dtheta;
    ret.dvel = imu1.dvel + imu2.dvel;
    return ret;
}
} // namespace ImuCommon
