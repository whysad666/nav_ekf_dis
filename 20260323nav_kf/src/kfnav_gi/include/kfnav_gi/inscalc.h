#pragma once

#include "common/types.h"

#include "kf_gins_types.h"

class INSCalc {

public:
    enum INSMode { Normal = 0, VdZeroed = 1, NoPosUpdate = 2 };

    /**
     * @brief INS机械编排算法, 利用IMU数据进行速度、位置和姿态更新
     *        INS Mechanization, update velocity, position and attitude using imudata
     * @param [in]     pvapre 上一时刻状态
     *                        the last imustate
     * @param [in,out] pvacur 输出当前时刻状态
     *                        output the current imustate
     * @param [in]     imu1, imu2 imudata
     * */
    static void calc(PVA &pvacur, const IMU &imu1, const IMU &imu2, const double sampt0,
                       const INSMode mode = Normal);

};
