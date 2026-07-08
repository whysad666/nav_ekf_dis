#include "common/earth.h"
#include "common/rotation.h"

#include "inscalc.h"

void INSCalc::calc(PVA &pvacur, const IMU &imu1, const IMU &imu2, const double sampt0, const INSMode mode) {

    Eigen::Vector3d gl;
    gl << 0, 0, Earth::gravity(pvacur.pos);
    // 计算地球自转角速度投影到n系, n系相对于e系转动角速度投影到n系
    // calculate  earth rotational angular velocity projected to n-frame,
    // rotational angular velocity of n-frame to e-frame projected to n-frame
    Eigen::Vector3d wie_n = Earth::wien(pvacur.pos[0]);
    Eigen::Vector2d rnre = Earth::meridianPrimeVerticalRadius(pvacur.pos[0]);
    Eigen::Vector3d wen_n = Earth::wenn(rnre, pvacur.pos, pvacur.vel);
    Eigen::Vector3d win_n = wie_n + wen_n;
    Eigen::Vector3d winb = pvacur.att.cbn.transpose() * win_n;

    // std::cout << std::scientific << std::setprecision(15) << "wie_n: " << wie_n.transpose() << std::endl;
    // std::cout << std::scientific << std::setprecision(15) << "wen_n: " << wen_n.transpose() << std::endl;
    // std::cout << std::scientific << std::setprecision(15) << "winn: " << win_n.transpose() << std::endl;
    // std::cout << std::scientific << std::setprecision(15) << "winb: " << winb.transpose() << std::endl;

    auto ang1 = imu1.dtheta - winb * sampt0;
    auto ang2 = imu2.dtheta - winb * sampt0;
    auto angle = ang1 + ang2;

    // b系比力积分项
    // velocity increment due to the specific force
    Eigen::Vector3d d_vfb;
    if (1) {
        // 使用wnbb作为划摇补偿的角速度
        d_vfb = imu1.dvel +
                imu2.dvel
                // 旋转效应
                // rotational motion
                + 0.5 * angle.cross(imu1.dvel + imu2.dvel)
                // 双子样划桨效应
                // sculling motion
                + 2.0 / 3.0 * (imu1.dvel.cross(ang2) + ang1.cross(imu2.dvel));
    } else {
        // 使用wibb作为划摇补偿的角速度
        d_vfb = imu1.dvel +
                imu2.dvel
                // 旋转效应
                // rotational motion
                + 0.5 * (imu1.dtheta + imu2.dtheta).cross(imu1.dvel + imu2.dvel)
                // 双子样划桨效应
                // sculling motion
                + 2.0 / 3.0 * (imu1.dvel.cross(imu2.dtheta) + imu1.dtheta.cross(imu2.dvel));
    }

    // 比力积分项投影到n系
    // velocity increment dut to the specfic force projected to the n-frame
    Eigen::Vector3d d_vfn = pvacur.att.cbn * d_vfb;

    // 计算重力/哥式积分项
    // velocity increment due to the gravity and Coriolis force
    Eigen::Vector3d d_vgn = (gl - (2 * wie_n + wen_n).cross(pvacur.vel)) * 2 * sampt0;

    // std::cout << std::scientific << std::setprecision(15) << "dvfb: " << d_vfb.transpose() << std::endl;
    // std::cout << std::scientific << std::setprecision(15) << "dvfn: " << d_vfn.transpose() << std::endl;
    // std::cout << std::scientific << std::setprecision(15) << "dvgn: " << d_vgn.transpose() << std::endl;

    // 速度更新完成
    // velocity update finish
    // std::cout << std::scientific << std::setprecision(15) << "vel pre: " << pvacur.vel.transpose() << std::endl;
    pvacur.vel = pvacur.vel + d_vfn + d_vgn;
    // std::cout << std::scientific << std::setprecision(15) << "vel update: " << pvacur.vel.transpose() << std::endl;

    // 地向速度置零
    // set the down velocity to zero
    if (mode == INSMode::VdZeroed) {
        pvacur.vel[2] = 0;
    }

    // pos update
    if (mode != INSMode::NoPosUpdate) {
        if (1) {
            // 使用新的速度计算wenn
            wen_n = Earth::wenn(rnre, pvacur.pos, pvacur.vel);

            Eigen::Quaterniond qne, qee, qnn;

            // 重新计算 k时刻到k-1时刻 n系旋转矢量
            // recompute n-frame rotation vector (n(k) with respect to n(k-1)-frame)
            qnn = Rotation::rotvec2quaternion((wie_n + wen_n) * 2 * sampt0);

            // e系转动等效旋转矢量 (k-1时刻k时刻，所以取负号)
            // e-frame rotation vector (e(k-1) with respect to e(k)-frame)
            qee = Rotation::rotvec2quaternion(Eigen::Vector3d(0, 0, -WGS84_WIE * 2 * sampt0));

            // 位置更新完成
            // position update finish
            qne = Earth::qne(pvacur.pos);
            qne = qee * qne * qnn;
            double height = pvacur.pos[2] - pvacur.vel[2] * 2 * sampt0;
            pvacur.pos = Earth::blh(qne, height);
        } else {
            pvacur.pos(2) = pvacur.pos(2) - 2.0 * sampt0 * pvacur.vel(2);
            pvacur.pos(0) = pvacur.pos(0) + 2.0 * sampt0 * pvacur.vel(0) / (rnre(0) + pvacur.pos(2));
            pvacur.pos(1) =
                pvacur.pos(1) + 2.0 * sampt0 * pvacur.vel(1) / ((rnre(1) + pvacur.pos(2)) * cos(pvacur.pos(0)));
        }
    }

    rnre = Earth::meridianPrimeVerticalRadius(pvacur.pos[0]);
    gl.setZero();
    gl << 0, 0, Earth::gravity(pvacur.pos);

    // 最后计算姿态更新
    if (0) {
        // some difference
        // 计算n系的旋转四元数 k-1时刻到k时刻变换
        // n-frame rotation vector (n(k-1) with respect to n(k)-frame)
        Eigen::Quaterniond qnn = Rotation::rotvec2quaternion(-(wie_n + wen_n) * 2 * sampt0);

        // 计算b系旋转四元数
        // b-frame rotation vector (b(k) with respect to b(k-1)-frame)
        Eigen::Vector3d temp1;
        temp1 = imu1.dtheta +
                imu2.dtheta
                // 补偿二阶圆锥误差
                // compensate the second-order coning correction term.
                + 2.3 / 3.0 * imu1.dtheta.cross(imu2.dtheta);
        Eigen::Quaterniond qbb = Rotation::rotvec2quaternion(temp1);

        // 姿态更新完成
        // attitude update finish
        pvacur.att.qbn = qnn * pvacur.att.qbn * qbb;
    } else {
        // fine
        auto rv = angle + 2.0 / 3.0 * ang1.cross(ang2);
        // std::cout << std::scientific << std::setprecision(15) << "rotvec: " << rv.transpose() << std::endl;
        // fine
        Eigen::Quaterniond dq = Rotation::rotvec2quaternion(rv);
        pvacur.att.qbn = pvacur.att.qbn * dq;
    }
    pvacur.att.cbn = Rotation::quaternion2matrix(pvacur.att.qbn);
    pvacur.att.euler = Rotation::matrix2euler(pvacur.att.cbn);
}
