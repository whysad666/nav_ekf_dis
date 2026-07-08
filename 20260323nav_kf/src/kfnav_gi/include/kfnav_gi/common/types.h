/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
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

#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

using GNSS = struct GNSS {
    // 对于读文件运行，time == UINT64_MAX表示文件已经结束
    uint64_t time;     // ns
    uint64_t utc_time; // ns
    Vector3d blh;
    Vector3d vel;
    // 对于无效的测量，将其std设为+INF
    //默认值：0.5m,默认值：0.5m/s
    Vector3d std=Vector3d(0.5,0.5,0.5);
    Vector3d vstd=Vector3d(0.5,0.5,0.5);

    bool flag_pos, flag_vel, flag_utc;
};

using BARO = struct BARO {
    // 对于读文件运行，time == UINT64_MAX表示文件已经结束
    uint64_t time;     // ns
    uint64_t utc_time; // ns
    double pressure; // m
    // 对于无效的测量，将其std设为+INF
    double std=0.5;
    bool flag_baro, flag_utc;
};

using MAG = struct MAG {
    // 对于读文件运行，time == UINT64_MAX表示文件已经结束
    uint64_t time; // ns
    Vector3d mag;  // microtesla, body frame aligned with prepared IMU
};

//range
struct rangeAnchorData
{
    int tag;               // 基站的标签号
    Eigen::Matrix<double, 3, 1> position; // 基站的坐标
    double range;       // 无人机与基站的距离
    double time;      // 基站数据的时间戳

    // 打印基站数据
    friend std::ostream& operator<<(std::ostream& os, const rangeAnchorData& anchor) {
        os << "Anchor - Tag: " << anchor.tag;
        os << ", Position: " << anchor.position(0) << ", " << anchor.position(1) << ", " << anchor.position(2); // 设置矩阵输出格式
        os << ", range: " << anchor.range;
        os << ", time: " << anchor.time;
        return os;
    }
};
struct RANGE
{
    double time = -1; // 时间戳，用于判断RANGE数据是否为空或者有效
    double empty = 0; // 用于判断RANGE数据是否为空或者有效
    std::vector<rangeAnchorData> anchors; // 包含不同基站的信息    
    // 打印range数据
    friend std::ostream& operator<<(std::ostream& os, const RANGE& RANGE) {
        os << "RANGE - time: " << RANGE.time;
        os << ", Anchors: [";
        for (const auto& anchor : RANGE.anchors) {
            os << anchor;
            if (&anchor != &RANGE.anchors.back()) os << ", ";
        }
        os << "]";
        return os;
    }
      bool operator<(const RANGE& other) const {
        return time < other.time;
  }
};




using IMU = struct IMU {
    // 对于读文件运行，time == UINT64_MAX表示文件已经结束
    uint64_t time; // ns
    double dt;     // s
    uint64_t raw_sn; // 原始时间戳*200，相当于惯导序列号

    Vector3d dtheta;
    Vector3d dvel;
};

using Pose = struct Pose {
    Matrix3d R;
    Vector3d t;
};

#endif // TYPES_H
