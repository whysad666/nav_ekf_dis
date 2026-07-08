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

#ifndef ANGLE_H
#define ANGLE_H
#include <Eigen/Dense>

const double D2R = (M_PI / 180.0);
const double R2D = (180.0 / M_PI);
const double RS2DH = R2D * 3600;
const double DH2RS = D2R / 3600;

class Angle {

public:
    static auto rad2deg(double rad) -> double {
        return rad * R2D;
    }

    static auto deg2rad(double deg) -> double {
        return deg * D2R;
    }

    static auto rad2deg(float rad) -> float {
        return rad * R2D;
    }

    static auto deg2rad(float deg) -> float {
        return deg * D2R;
    }

    template <typename T, int Rows, int Cols>
    static auto rad2deg(const Eigen::Matrix<T, Rows, Cols> &array) -> Eigen::Matrix<T, Rows, Cols> {
        return array * R2D;
    }

    template <typename T, int Rows, int Cols>
    static auto deg2rad(const Eigen::Matrix<T, Rows, Cols> &array) -> Eigen::Matrix<T, Rows, Cols> {
        return array * D2R;
    }
};

#endif // ANGLE_H
