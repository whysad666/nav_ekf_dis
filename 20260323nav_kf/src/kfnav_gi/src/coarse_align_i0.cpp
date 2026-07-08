#include <coarse_align_i0.h>

void CoarseAlignI0::update(const IMU &imu1) {
    // b系比力积分项
    // velocity increment due to the specific force
    auto dv_b = imu1.dvel + 0.5 * imu1.dtheta.cross(imu1.dvel);

    // 速度更新完成
    // velocity update finish
    vib0 += Cbib0 * dv_b;
    // 保存速度
    // save velocity
    vib0_vec.push_back(vib0);
    // std::cout<<"vib0: "<<vib0.transpose()<<std::endl;

    // 计算b系旋转四元数
    // b-frame rotation vector (b(k) with respect to b(k-1)-frame)
    auto rotvec = imu1.dtheta;
    auto qbb = Rotation::rotvec2quaternion(rotvec);
    // 姿态更新完成
    // attitude update finish
    qbib0 = qbib0 * qbb;
    // std::cout<<"qbib0: "<<qbib0<<std::endl;
    Cbib0 = Rotation::quaternion2matrix(qbib0);

    cnt += 1;
    vel_cnt++;
}

void CoarseAlignI0::update(const IMU &imu1, const IMU &imu2) {
    // b系比力积分项
    // velocity increment due to the specific force
    auto dv_b = imu1.dvel +
                imu2.dvel
                // 旋转效应
                // rotational motion
                + 0.5 * (imu1.dtheta + imu2.dtheta).cross(imu1.dvel + imu2.dvel)
                // 双子样划桨效应
                // sculling motion
                + 2.0 / 3.0 * (imu1.dvel.cross(imu2.dtheta) + imu1.dtheta.cross(imu2.dvel));

    // 速度更新完成
    // velocity update finish
    vib0 += Cbib0 * dv_b;
    // 保存速度
    // save velocity
    vib0_vec.push_back(vib0);

    // 计算b系旋转四元数
    // b-frame rotation vector (b(k) with respect to b(k-1)-frame)
    auto rotvec = imu1.dtheta +
                  imu2.dtheta
                  // 补偿二阶圆锥误差
                  // compensate the second-order coning correction term.
                  + 2.3 / 3.0 * imu1.dtheta.cross(imu2.dtheta);
    auto qbb = Rotation::rotvec2quaternion(rotvec);
    // 姿态更新完成
    // attitude update finish
    qbib0 = qbib0 * qbb;
    Cbib0 = Rotation::quaternion2matrix(qbib0);

    cnt += 2;
    vel_cnt++;
}

auto CoarseAlignI0::calc() -> Attitude {
    // 如果计算对准数据之前未准备好参数，则停止程序
    assert(is_ready == true);
    // 从初始时刻以来经历的惯导采样周期数
    auto n2 = cnt;
    auto n1 = n2 / 2;
    // 从初始时刻以来经过的时间
    auto t2 = n2 * Sampt0;
    auto t1 = n1 * Sampt0;
    // 上述时刻对应的速度记录索引
    auto index2 = vib0_vec.size() - 1;
    auto index1 = (index2 - 1) / 2;
    // 索引对应关系：
    // n1 == n2/2
    // n2 == index2+1
    // n1 == index1+1
    // 索引对应表：
    // n2, index2,  n1, index1
    //  2,     1,    1,    0
    //  3,     2,    1,    0
    //  4,     3,    2,    1
    //  5,     4,    2,    1
    //  6,     5,    3,    2
    //  7,     6,    3,    2

    // ib0系中的比力积分
    auto Vib0_1 = vib0_vec.at(index1);
    auto Vib0_2 = vib0_vec.at(index2);

    // i系中的负重力加速度积分
    Eigen::Vector3d Vi_1 = Vi_calc(t1);
    Eigen::Vector3d Vi_2 = Vi_calc(t2);

    // 双矢量定姿
    Eigen::Matrix3d Cib0i = dv2att(Vi_1, Vi_2, Vib0_1, Vib0_2);

    auto Cen = Earth::cne(Eigen::Vector3d(Lati, 0, Alti)).transpose();
    auto Cb0n0 = Cen * Cib0i;

    // 返回值
    Attitude att;
    att.cbn = Cb0n0;
    att.qbn = Rotation::matrix2quaternion(att.cbn);
    att.euler = Rotation::matrix2euler(att.cbn);
    return att;
}

auto CoarseAlignI0::dv2att(const Eigen::Vector3d &vn1, const Eigen::Vector3d &vn2, const Eigen::Vector3d &vb1,
                           const Eigen::Vector3d &vb2) -> Eigen::Matrix3d {
    auto vn_c12 = vn1.cross(vn2);
    auto vn_c121 = vn_c12.cross(vn1);
    auto vb_c12 = vb1.cross(vb2);
    auto vb_c121 = vb_c12.cross(vb1);

    // 计算姿态矩阵
    // calculate attitude matrix
    Eigen::Matrix3d matn, matb;
    matn << vn1 / vn1.norm(), vn_c12 / vn_c12.norm(), vn_c121 / vn_c121.norm();
    matb << vb1 / vb1.norm(), vb_c12 / vb_c12.norm(), vb_c121 / vb_c121.norm();

    auto cbn = matn * matb.transpose();

    return cbn;
}

auto CoarseAlignI0::Vi_calc(const double t) -> Eigen::Vector3d {
    // 地球参数
    auto gu = Earth::gravity(Eigen::Vector3d(Lati, 0, Alti));
    auto WIE = WGS84_WIE;
    Eigen::Vector3d Vi;
    Vi << gu * cos(Lati) * sin(WIE * t) / WIE, gu * cos(Lati) * (1 - cos(WIE * t)) / WIE, gu * sin(Lati) * t;
    return Vi;
}
