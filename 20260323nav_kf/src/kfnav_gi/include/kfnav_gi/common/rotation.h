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

#ifndef ROTATION_H
#define ROTATION_H

#include <Eigen/Geometry>
#include <iostream>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

class Rotation {

public:
    static auto matrix2quaternion(const Matrix3d &matrix) -> Quaterniond {
        return Quaterniond(matrix);
    }

    static auto quaternion2matrix(const Quaterniond &quaternion) -> Matrix3d {
        return quaternion.toRotationMatrix();
    }

    // ZYX旋转顺序, 前右下的IMU, 输出RPY
    static auto matrix2euler(const Eigen::Matrix3d &dcm) -> Vector3d {
        Vector3d euler;

        euler[1] = atan(-dcm(2, 0) / sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));

        if (dcm(2, 0) <= -0.999) {
            euler[0] = 0;
            euler[2] = atan2((dcm(1, 2) - dcm(0, 1)), (dcm(0, 2) + dcm(1, 1)));
            std::cout << "[WARNING] Rotation::matrix2euler: Singular Euler Angle! Set the roll angle to 0!"
                      << std::endl;
        } else if (dcm(2, 0) >= 0.999) {
            euler[0] = 0;
            euler[2] = M_PI + atan2((dcm(1, 2) + dcm(0, 1)), (dcm(0, 2) - dcm(1, 1)));
            std::cout << "[WARNING] Rotation::matrix2euler: Singular Euler Angle! Set the roll angle to 0!"
                      << std::endl;
        } else {
            euler[0] = atan2(dcm(2, 1), dcm(2, 2));
            euler[2] = atan2(dcm(1, 0), dcm(0, 0));
        }

        return euler;
    }

    static auto quaternion2euler(const Quaterniond &quaternion) -> Vector3d {
        return matrix2euler(quaternion.toRotationMatrix());
    }

    static auto rotvec2quaternion(const Vector3d &rotvec) -> Quaterniond {
        double angle = rotvec.norm();
        Vector3d vec = rotvec.normalized();
        return Quaterniond(Eigen::AngleAxisd(angle, vec));
    }

    static auto quaternion2vector(const Quaterniond &quaternion) -> Vector3d {
        Eigen::AngleAxisd axisd(quaternion);
        return axisd.angle() * axisd.axis();
    }

    // RPY --> C_b^n, ZYX顺序
    static auto euler2matrix(const Vector3d &euler) -> Matrix3d {
        return Matrix3d(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                        Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
    }

    static auto euler2quaternion(const Vector3d &euler) -> Quaterniond {
        return Quaterniond(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                           Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
    }

    static auto skewSymmetric(const Vector3d &vector) -> Matrix3d {
        Matrix3d mat;
        mat << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1), vector(0), 0;
        return mat;
    }

    static auto quaternionleft(const Quaterniond &q) -> Eigen::Matrix4d {
        Eigen::Matrix4d ans;
        ans(0, 0)             = q.w();
        ans.block<1, 3>(0, 1) = -q.vec().transpose();
        ans.block<3, 1>(1, 0) = q.vec();
        ans.block<3, 3>(1, 1) = q.w() * Eigen::Matrix3d::Identity() + skewSymmetric(q.vec());
        return ans;
    }

    static auto quaternionright(const Quaterniond &p) -> Eigen::Matrix4d {
        Eigen::Matrix4d ans;
        ans(0, 0)             = p.w();
        ans.block<1, 3>(0, 1) = -p.vec().transpose();
        ans.block<3, 1>(1, 0) = p.vec();
        ans.block<3, 3>(1, 1) = p.w() * Eigen::Matrix3d::Identity() - skewSymmetric(p.vec());
        return ans;
    }
};

#endif // ROTATION_H
