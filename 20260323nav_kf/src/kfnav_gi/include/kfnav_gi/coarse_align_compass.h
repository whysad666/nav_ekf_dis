#pragma once

#include <Eigen/Dense>
#include <vector>

#include "common/rotation.h"
#include "common/types.h"

#include "kf_gins_types.h"

class CoarseAlignCompass {
public:
    CoarseAlignCompass() {
        cumsum.dt     = 0.0;
        cumsum.dtheta = Eigen::Vector3d::Zero();
        cumsum.dvel   = Eigen::Vector3d::Zero();
        cnt           = 0;
    }
    ~CoarseAlignCompass() {
    }

    /**
     * @brief 添加新的IMU数据，(不)补偿IMU误差
     *        add new imudata, do (not) compensate imu error
     * @param [in] imu        新的IMU原始数据
     *                        new raw imudata
     * @param [in] compensate 是否补偿IMU误差
     *                        if compensate imu error to new imudata
     * */
    void addImuData(const IMU &imu) {
        // 累加IMU数据
        cumsum.dt += imu.dt;
        cumsum.dtheta += imu.dtheta;
        cumsum.dvel += imu.dvel;
        cnt++;
    }

    double timeLength() {
        return cumsum.dt;
    }

    uint64_t count() {
        return cnt;
    }

    Eigen::Vector3d getEuler(bool compensate = false) {
        Eigen::Vector3d f0, wibb;
        if (compensate) {
            // 补偿IMU比例因子
            // compensate the imu scale
            Eigen::Vector3d gyrscale, accscale;
            gyrscale = Eigen::Vector3d::Ones() + imuerror_.gyrscale;
            accscale = Eigen::Vector3d::Ones() + imuerror_.accscale;
            // 补偿IMU零偏
            // compensate the imu bias
            wibb = (cumsum.dtheta / cumsum.dt - imuerror_.gyrbias).cwiseProduct(gyrscale.cwiseInverse());
            f0   = (cumsum.dvel / cumsum.dt - imuerror_.accbias).cwiseProduct(accscale.cwiseInverse());
        } else {
            wibb = cumsum.dtheta / cumsum.dt;
            f0   = cumsum.dvel / cumsum.dt;
        }
        // 调试输出
        // debug output
        if (0) {
            std::cout << "wibb: " << wibb.transpose() << ", f0: " << f0.transpose() << std::endl;
        }

        double roll  = atan2(-f0[1], -f0[2]);
        double pitch = asin(f0[0] / f0.norm());
        // double pitch = atan2( f0[ 0 ], sqrt( f0[ 1 ] * f0[ 1 ] + f0[ 2 ] * f0[ 2 ] ) );
        // std::cout << "diff pitch" << ( pitch - pitch2 ) * R2D << std::endl; // debug
        double yaw = atan2(-wibb[1] * cos(roll) + wibb[2] * sin(roll),
                           wibb[0] * cos(pitch) + wibb[1] * sin(roll) * sin(pitch) + wibb[2] * cos(roll) * sin(pitch));
        return Eigen::Vector3d(roll, pitch, yaw);
    }

    Attitude getAttitude(bool compensate = false) {
        Attitude ret;
        ret.euler = getEuler(compensate);
        ret.qbn   = Rotation::euler2quaternion(ret.euler);
        ret.cbn   = Rotation::quaternion2matrix(ret.qbn);
        return ret;
    }

private:
    uint64_t cnt;
    IMU cumsum;
    ImuError imuerror_;
};
