#include <imu_common.h>

namespace ImuCommon {
IMU imuCalibration(const IMU &imu, const ImuCalib &imucalib_) {
    // 补偿标定参数
    IMU ret;
    ret.time = imu.time;
    ret.dt = imu.dt;
    ret.raw_sn = imu.raw_sn;

    ret.dvel = imucalib_.Kaccl * imu.dvel - imucalib_.bias_accl * imu.dt;
    ret.dtheta = imucalib_.Kgyro * (imu.dtheta - imucalib_.bias_gyro);
    return ret;
}

IMU imuCompensate(const IMU &imu, const ImuError &imuerror_) {
    // 补偿滤波结果
    IMU ret;
    ret.time = imu.time;
    ret.dt = imu.dt;
    ret.raw_sn = imu.raw_sn;

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
    IMU ret;
    ret.time = std::max(imu1.time, imu2.time);
    ret.dt = imu1.dt + imu2.dt;
    ret.dtheta = imu1.dtheta + imu2.dtheta;
    ret.dvel = imu1.dvel + imu2.dvel;
    return ret;
}
} // namespace ImuCommon
