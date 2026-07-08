#include "common/earth.h"
#include "common/rotation.h"
#include <vector>

#include "imu_common.h"
#include "inscalc.h"
#include <inspv_stekf_15.h>
#include <unsupported/Eigen/MatrixFunctions>

InsPvStekf15states::InsPvStekf15states(KFOptions &options)
{

    this->options_ = options;
    options_.print_options();
    timestamp_ = 0;
    utc_ns_ = 0;
    raw_sn = 0;

    // 设置协方差矩阵，系统噪声阵和系统误差状态矩阵大小
    // resize covariance matrix, system noise matrix, and system error state matrix
    Cov_.resize(StateDim, StateDim);
    Qc_.resize(NoiseDim, NoiseDim);
    dx_.resize(StateDim, 1);
    Cov_.setZero();
    Qc_.setZero();
    dx_.setZero();

    // 初始化系统噪声阵
    // initialize noise matrix
    auto imunoise = options_.prop_noise;
    Qc_.block<3, 3>(ARW_ID, ARW_ID) = imunoise.gyr_arw.cwiseProduct(imunoise.gyr_arw).asDiagonal();
    Qc_.block<3, 3>(VRW_ID, VRW_ID) = imunoise.acc_vrw.cwiseProduct(imunoise.acc_vrw).asDiagonal();
    Qc_.block<3, 3>(BGRW_ID, BGRW_ID) = imunoise.bias_gyr_rw.cwiseProduct(imunoise.bias_gyr_rw).asDiagonal();
    Qc_.block<3, 3>(BARW_ID, BARW_ID) = imunoise.bias_acc_rw.cwiseProduct(imunoise.bias_acc_rw).asDiagonal();

    // 设置系统状态(位置、速度、姿态和IMU误差)初值和初始协方差
    // set initial state (position, velocity, attitude and IMU error) and covariance
    initialize(options_.initstate, options_.initstate_std);
}

void InsPvStekf15states::initialize(const NavState &initstate, const NavState &initstate_std)
{

    // 初始化位置、速度、姿态
    // initialize position, velocity and attitude
    pvacur_.pos = initstate.pos;
    pvacur_.vel = initstate.vel;
    pvacur_.att.euler = initstate.euler;
    pvacur_.att.cbn = Rotation::euler2matrix(pvacur_.att.euler);
    pvacur_.att.qbn = Rotation::euler2quaternion(pvacur_.att.euler);
    // 初始化IMU误差
    // initialize imu error
    imuerror_ = initstate.imuerror;

    Eigen::Vector2d rnre = Earth::meridianPrimeVerticalRadius(initstate.pos(0));
    Eigen::Vector2d rnhreh = rnre + Eigen::Vector2d::Ones() * initstate.pos[2];
    // 初始化协方差
    // initialize covariance
    ImuError imuerror_std = initstate_std.imuerror;
    auto phi_std_si = initstate_std.euler * D2R;
    Cov_.block<3, 3>(PHI_ID, PHI_ID) = phi_std_si.cwiseProduct(phi_std_si).asDiagonal();
    auto pos_std_si = initstate_std.pos.cwiseProduct(
        Eigen::Vector3d(1.0 / rnhreh(0), 1.0 / (rnhreh(1) * cos(initstate.pos(0))), 1.0));
    Cov_.block<3, 3>(P_ID, P_ID) = pos_std_si.cwiseProduct(pos_std_si).asDiagonal();
    Cov_.block<3, 3>(V_ID, V_ID) = initstate_std.vel.cwiseProduct(initstate_std.vel).asDiagonal();
    Cov_.block<3, 3>(BG_ID, BG_ID) = imuerror_std.gyrbias.cwiseProduct(imuerror_std.gyrbias).asDiagonal();
    Cov_.block<3, 3>(BA_ID, BA_ID) = imuerror_std.accbias.cwiseProduct(imuerror_std.accbias).asDiagonal();
}

void InsPvStekf15states::insPropagation(const IMU &imu1_in, const IMU &imu2_in, const double dt)
{

    // 当前IMU时间作为系统当前状态时间,
    // set current IMU time as the current state time
    timestamp_ = std::max(imu1_in.time, imu2_in.time);
    //printf("time now: %f \n",(imu1_in.time-imu2_in.time)/1e9); 
    // 当前IMU序列号作为系统当前状态序列号
    // set current IMU sequence number as the current state sequence number
    raw_sn = std::max(imu1_in.raw_sn, imu2_in.raw_sn);
    //printf("after theta: %lf ,vel=%lf\n",imu1_in.dtheta(1) ,imu1_in.dvel(1)); 
    IMU imu1, imu2;
    // 补偿IMU零偏
    // compensate the imu bias
    imu1 = ImuCommon::imuCompensate(imu1_in, imuerror_);
    imu2 = ImuCommon::imuCompensate(imu2_in, imuerror_);
    //printf("bias:%f %f %f \n",imuerror_.accbias(0),imuerror_.accbias(1),imuerror_.accbias(2));
    //printf("bias:%f %f %f \n",imuerror_.gyrbias(0),imuerror_.gyrbias(1),imuerror_.gyrbias(2));
    
    // IMU状态更新(机械编排算法)
    // update imustate(mechanization)
    INSCalc::calc(pvacur_, imu1, imu2, dt, INSCalc::INSMode::Normal);

    // 系统噪声传播，姿态误差采用phi角误差模型
    // system noise propagate, phi-angle error model for attitude error
    Eigen::MatrixXd Phi, F, Qd, G;

    // 初始化Phi阵(状态转移矩阵)，F阵，Qd阵(传播噪声阵)，G阵(噪声驱动阵)
    // initialize Phi (state transition), F matrix, Qd(propagation noise) and G(noise driven) matrix
    F.resize(StateDim, StateDim);
    G.resize(StateDim, NoiseDim);
    F.setZero();
    G.setZero();

    // 使用上一历元状态计算状态转移矩阵
    // compute state transition matrix using the previous state
    Eigen::Matrix3d cbn = pvacur_.att.cbn;
    Eigen::Vector3d vel = pvacur_.vel;
    Eigen::Vector3d pos = pvacur_.pos;
    double lati, clati, slati, tlati, c2lati;
    {
        lati = pos[0];
        clati = cos(lati);
        slati = sin(lati);
        tlati = tan(lati);
        c2lati = clati * clati;
    }

    Eigen::Vector2d rnre;
    Eigen::Vector3d wie_n, wen_n, gn;
    {
        double gravity;
        rnre = Earth::meridianPrimeVerticalRadius(lati);
        gravity = Earth::gravity(pos);
        gn << 0, 0, gravity;
        wie_n = Earth::wien(lati);
        wen_n = Earth::wenn(rnre, pos, vel);
    }

    double vn, ve;
    {
        vn = vel[0];
        ve = vel[1];
    }

    double rnh, reh, rnh2, reh2;
    {
        rnh = rnre[0] + pvacur_.pos[2];
        reh = rnre[1] + pvacur_.pos[2];
        rnh2 = rnh * rnh;
        reh2 = reh * reh;
    }

    Eigen::Matrix3d Fer, Fev, Frr, Frv, deltaOmega;
    {
        Fer.setZero();
        Fer << -WGS84_WIE * slati, 0, -ve / reh2, 0, 0, vn / rnh2, -WGS84_WIE * clati - ve / (reh * c2lati), 0,
            ve * tlati / reh2;
        Fev.setZero();
        Fev << 0, 1 / reh, 0, -1 / rnh, 0, 0, 0, -tlati / reh, 0;
        Frr.setZero();
        Frr << 0, 0, -vn / rnh2, ve * tlati / (reh * clati), 0, -ve / (reh2 * clati), 0, 0, 0;
        Frv.setZero();
        Frv << 1 / rnh, 0, 0, 0, 1 / (reh * clati), 0, 0, 0, -1;

        deltaOmega.setZero();
        deltaOmega << -WGS84_WIE * slati, 0, 0, 0, 0, 0, -WGS84_WIE * clati, 0, 0;
    }

    // 姿态误差
    // attitude error
    F.block<3, 3>(PHI_ID, PHI_ID) = -Rotation::skewSymmetric(wie_n + wen_n) + Fev * Rotation::skewSymmetric(vel);
    F.block<3, 3>(PHI_ID, V_ID) = Fev;
    F.block<3, 3>(PHI_ID, P_ID) = Fer;
    F.block<3, 3>(PHI_ID, BG_ID) = -cbn;

    // 速度误差
    // velocity error
    F.block<3, 3>(V_ID, PHI_ID) =
        -Rotation::skewSymmetric(gn) - Rotation::skewSymmetric(vel) * Rotation::skewSymmetric(wie_n);
    F.block<3, 3>(V_ID, V_ID) = -Rotation::skewSymmetric(2 * wie_n + wen_n);
    F.block<3, 3>(V_ID, P_ID) = Rotation::skewSymmetric(vel) * deltaOmega;
    F.block<3, 3>(V_ID, BG_ID) = Rotation::skewSymmetric(vel) * cbn;
    F.block<3, 3>(V_ID, BA_ID) = cbn;

    // 位置误差
    // position error
    F.block<3, 3>(P_ID, PHI_ID) = Frv * Rotation::skewSymmetric(vel);
    F.block<3, 3>(P_ID, V_ID) = Frv;
    F.block<3, 3>(P_ID, P_ID) = Frr;

    // IMU零偏误差和比例因子误差，建模成一阶高斯-马尔科夫过程
    // imu bias error and scale error, modeled as the first-order Gauss-Markov process
    F.block<3, 3>(BG_ID, BG_ID) = -1 / options_.prop_noise.corr_time * Eigen::Matrix3d::Identity();
    F.block<3, 3>(BA_ID, BA_ID) = -1 / options_.prop_noise.corr_time * Eigen::Matrix3d::Identity();

    // 系统噪声驱动矩阵
    // system noise driven matrix
    G.block<3, 3>(PHI_ID, ARW_ID) = -cbn;
    G.block<3, 3>(V_ID, ARW_ID) = Rotation::skewSymmetric(vel) * cbn;
    G.block<3, 3>(V_ID, VRW_ID) = cbn;
    G.block<3, 3>(BG_ID, BGRW_ID) = Eigen::Matrix3d::Identity();
    G.block<3, 3>(BA_ID, BARW_ID) = Eigen::Matrix3d::Identity();

    // 状态转移矩阵
    // compute the state transition matrix
    auto ft = F * dt * 2;
    Phi = ft.exp();

    // 计算系统传播噪声
    // compute system propagation noise
    Qd = G * Qc_ * G.transpose() * dt * 2;
    // Qd = (Phi * Qd * Phi.transpose() + Qd) / 2;

    // EKF预测传播系统协方差和系统误差状态
    // do EKF predict to propagate covariance and error state
    EKFPredict(Phi, Qd);

    // 检查协方差矩阵对角线元素
    // check diagonal elements of current covariance matrix
    checkCov();
}

void InsPvStekf15states::velUpdate(const Eigen::Vector3d &vel_measure, const Eigen::Vector3d &vel_cov)
{
    const uint64_t dim_z = 3;
    // 观测：
    Eigen::MatrixXd Z;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;

    // 测量新息
    // compute innovation
    Z.resize(dim_z, 1);
    Z.block<3, 1>(0, 0) = pvacur_.vel - vel_measure;

    // 构造GNSS位置观测矩阵
    // construct GNSS position measurement matrix
    H.resize(dim_z, StateDim);
    H.setZero();
    H.block<3, 3>(0, PHI_ID) = Rotation::skewSymmetric(pvacur_.vel);
    H.block<3, 3>(0, V_ID) = Eigen::Matrix3d::Identity();

    // 位置速度标准差
    Eigen::MatrixXd measure_std(dim_z, 1);
    measure_std.block<3, 1>(0, 0) = vel_cov;
    R = measure_std.cwiseProduct(measure_std).asDiagonal();

    // EKF更新协方差和误差状态
    // do EKF update to update covariance and error state
    EKFUpdate(Z, H, R);

    // 检查协方差矩阵对角线元素
    // check diagonal elements of current covariance matrix
    checkCov();

    // 将观测得到的误差状态反馈到导航状态中
    // feedback error state to navigation state
    stateFeedback();
}

// range_update,
// measure the distance between the current position and the anchor position
// input: the range, the anchor position, the anchor position standard deviation
void InsPvStekf15states::rangeUpdate(const double &range_measure, const Eigen::Vector3d &range_pos,
                                    const Eigen::Vector3d &range_pos_cov)
{
    const uint64_t dim_z = 1;
    // 观测：
    Eigen::MatrixXd Z;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;

    // 将pvacur_.pos和range_pos输入的值为纬度经度高度，从BLH坐标系转换为ECEF坐标系
    //  convert pvacur_.pos and range_pos from BLH coordinate system to ECEF coordinate system
    Eigen::Vector3d range_pos_R;
    // range_pos_R[0] = range_pos[0] / 180 * M_PI;
    // range_pos_R[1] = range_pos[1] / 180 * M_PI;
    range_pos_R[0] = range_pos[0];
    range_pos_R[1] = range_pos[1];
    range_pos_R[2] = range_pos[2];
    Eigen::Vector3d pvacur_ecef = Earth::blh2ecef(pvacur_.pos);
    Eigen::Vector3d range_pos_ecef = Earth::blh2ecef(range_pos_R);

    // 在ECEF坐标系下计算距离矩阵
    //  compute distance in ECEF coordinate system
    Eigen::Vector3d range_pos_ecef_ = range_pos_ecef - pvacur_ecef;
    // 计算欧氏距离
    //  compute Euclidean distance
    double range_measure_ecef = range_pos_ecef_.norm();

    // 测量新息
    // compute innovation
    Z.resize(dim_z, 1);
    Z(0, 0) = range_measure_ecef - (range_measure);
    // 如果Z(0,0)的绝对值大于30，则不进行更新
    // if the absolute value of Z(0,0) is greater than 30, do not update
    
    if (fabs(Z(0, 0))>range_pos_cov[1])
    {
        return;
    }
    else
    {
    //printf pvacur_.pos and range_pos in BLH coordinate system
    //printf("pvacur_.pos: %f, %f, %f\n", pvacur_.pos[0]*180/M_PI, pvacur_.pos[1]*180/M_PI, pvacur_.pos[2]);
    //printf("range_pos: %f, %f, %f\n", range_pos[0], range_pos[1], range_pos[2]);
    //printf("range_measure_ecef: %f || range: %f || d_range: %f\n", range_measure_ecef, range_measure, range_measure_ecef - range_measure);

    // 构造GNSS位置观测矩阵,BLH坐标系
    // construct GNSS position measurement matrix
    H.resize(dim_z, StateDim);

    H.setZero();
    // 计算给定纬度下的地球子午圈半径和卯酉圈半径
    Eigen::Vector2d rnre = Earth::meridianPrimeVerticalRadius(pvacur_.pos(0));
    Eigen::Vector2d rnhreh = rnre + Eigen::Vector2d::Ones() * pvacur_.pos[2];
    // 分别计算纬度、经度、高度三个方向的观测量，并构造观测矩阵
    H(0, P_ID) = (pvacur_.pos(0) - range_pos_R(0)) * (rnhreh[0] * rnhreh[0]) / (range_measure );
    H(0, P_ID + 1) = (pvacur_.pos(1) - range_pos_R(1)) * ((cos(pvacur_.pos[0]) * rnhreh[1]) * (cos(pvacur_.pos[0]) * rnhreh[1])) / (range_measure);
    H(0, P_ID + 2) = (pvacur_.pos(2) - range_pos_R(2)) / (range_measure);

    // 位置标准差
    Eigen::MatrixXd measure_std(1, 1);
    measure_std(0, 0) = range_pos_cov[0];
    R = measure_std.cwiseProduct(measure_std).asDiagonal();

    // EKF更新协方差和误差状态
    // do EKF update to update covariance and error state
    EKFUpdate(Z, H, R);
    // 检查协方差矩阵对角线元素
    // check diagonal elements of current covariance matrix
    checkCov();

    // 将观测得到的误差状态反馈到导航状态中
    // feedback error state to navigation state
    stateFeedback();
    }
}

// 批量测距更新
void InsPvStekf15states::batchRangeUpdate(const std::vector<rangeAnchorData> &anchors, const double &range_pos_cov)
{
    const uint64_t dim_z = anchors.size(); // 观测维度

    // 观测向量
    Eigen::MatrixXd Z(dim_z, 1);
    Z.setZero();

    // 观测矩阵
    Eigen::MatrixXd H(dim_z, StateDim); // 状态维度 StateDim = 15,常量

    H.setZero();

    // 观测噪声协方差矩阵
    Eigen::MatrixXd R(dim_z, dim_z);
    R.setZero();

    // 将BLH坐标系转换为ECEF坐标系
    Eigen::Vector3d pvacur_ecef = Earth::blh2ecef(pvacur_.pos);

    for (size_t i = 0; i < anchors.size(); ++i)
    {
        const auto &anchor = anchors[i];

        // 将锚点位置从BLH转换为ECEF
        Eigen::Vector3d anchor_pos_R;
        // 对锚点的位置的单位进行判断，如果单位是度，则转换为弧度；如果单位是弧度，则直接使用
        if (anchor.position(0) < M_PI && anchor.position(1) < M_PI)
        {
            anchor_pos_R[0] = anchor.position(0);
            anchor_pos_R[1] = anchor.position(1);
            anchor_pos_R[2] = anchor.position(2);
        }
        else
        {
            anchor_pos_R[0] = anchor.position(0) / 180 * M_PI;
            anchor_pos_R[1] = anchor.position(1) / 180 * M_PI;
            anchor_pos_R[2] = anchor.position(2);
        }
        Eigen::Vector3d anchor_ecef = Earth::blh2ecef(anchor_pos_R);

        // 计算ECEF坐标系下的距离
        Eigen::Vector3d range_ecef = anchor_ecef - pvacur_ecef;
        double range_ecef_norm = range_ecef.norm();

        // 计算新息
        Z(i, 0) = range_ecef_norm - anchor.range;
        // 判断新息是否大于一定值，如果大于则重置为0
        if (Z(i, 0) > 200)
        {
            Z(i, 0) = 0;
        }
        printf("range_ecef_norm: %f || range: %f || d_range: %f\n", range_ecef_norm, anchor.range, range_ecef_norm - anchor.range);

        // 构造观测矩阵
        Eigen::Vector2d rnre = Earth::meridianPrimeVerticalRadius(pvacur_.pos(0));
        Eigen::Vector2d rnhreh = rnre + Eigen::Vector2d::Ones() * pvacur_.pos[2];

        H(i, P_ID) = (pvacur_.pos(0) - anchor_pos_R(0)) * (rnhreh[0] * rnhreh[0]) / range_ecef_norm;
        H(i, P_ID + 1) = (pvacur_.pos(1) - anchor_pos_R(1)) * ((cos(pvacur_.pos[0]) * rnhreh[1]) * (cos(pvacur_.pos[0]) * rnhreh[1])) / range_ecef_norm;
        H(i, P_ID + 2) = (pvacur_.pos(2) - anchor_pos_R(2)) / range_ecef_norm;

        // 构造观测噪声协方差矩阵
        R(i, i) = range_pos_cov * range_pos_cov;
    }
    // 打印矩阵Z
    //  std::cout << "Z:  " << Z << std::endl;
    //  //打印矩阵H
    //  std::cout << "H:  " << H << std::endl;
    //  //打印矩阵R
    //  std::cout << "R:  " << R << std::endl;
    //  EKF更新
    EKFUpdate(Z, H, R);

    // 检查协方差矩阵
    checkCov();

    // 将观测得到的误差状态反馈到导航状态中
    stateFeedback();
}

void InsPvStekf15states::posUpdate(const Eigen::Vector3d &pos_measure, const Eigen::Vector3d &pos_cov)
{
    const uint64_t dim_z = 3;
    // 观测：

    Eigen::MatrixXd Z;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;

    // 测量新息
    // compute innovation
    Z.resize(dim_z, 1);
    Z.block<3, 1>(0, 0) = pvacur_.pos - pos_measure;

    // 构造GNSS位置观测矩阵
    // construct GNSS position measurement matrix
    H.resize(dim_z, StateDim);
    H.setZero();
    H.block<3, 3>(0, P_ID) = Eigen::Matrix3d::Identity();

    Eigen::Vector2d rnre = Earth::meridianPrimeVerticalRadius(pvacur_.pos(0));
    Eigen::Vector2d rnhreh = rnre + Eigen::Vector2d::Ones() * pvacur_.pos[2];
    Eigen::Vector3d pos_std_blh =
        pos_cov.cwiseProduct(Eigen::Vector3d(1 / rnhreh[0], 1 / (cos(pvacur_.pos[0]) * rnhreh[1]), 1));
    // 位置速度标准差
    Eigen::MatrixXd measure_std(dim_z, 1);
    measure_std.block<3, 1>(0, 0) = pos_std_blh;
    R = measure_std.cwiseProduct(measure_std).asDiagonal();

    // EKF更新协方差和误差状态
    // do EKF update to update covariance and error state
    EKFUpdate(Z, H, R);

    // 检查协方差矩阵对角线元素
    // check diagonal elements of current covariance matrix
    checkCov();

    // 将观测得到的误差状态反馈到导航状态中
    // feedback error state to navigation state
    stateFeedback();
}

void InsPvStekf15states::latiUpdate(const double lati_measure, const double lati_cov)
{
    // 观测：

    Eigen::MatrixXd Z;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;

    // 测量新息
    // compute innovation
    Z.resize(1, 1);
    Z(0, 0) = pvacur_.pos(0) - lati_measure;

    // 构造GNSS位置观测矩阵
    // construct GNSS position measurement matrix
    H.resize(1, StateDim);
    H.setZero();
    H(0, P_ID + 0) = 1;

    // 位置速度标准差
    Eigen::Vector2d rnre = Earth::meridianPrimeVerticalRadius(pvacur_.pos(0));
    Eigen::Vector2d rnhreh = rnre + Eigen::Vector2d::Ones() * pvacur_.pos[2];
    Eigen::MatrixXd measure_std(1, 1);
    measure_std(0, 0) = lati_cov / rnhreh[0];
    R = measure_std.cwiseProduct(measure_std).asDiagonal();

    // EKF更新协方差和误差状态
    // do EKF update to update covariance and error state
    EKFUpdate(Z, H, R);

    // 检查协方差矩阵对角线元素
    // check diagonal elements of current covariance matrix
    checkCov();

    // 将观测得到的误差状态反馈到导航状态中
    // feedback error state to navigation state
    stateFeedback();
}

void InsPvStekf15states::longiUpdate(const double longi_measure, const double longi_cov)
{
    // 观测：

    Eigen::MatrixXd Z;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;

    // 测量新息
    // compute innovation
    Z.resize(1, 1);
    Z(0, 0) = pvacur_.pos(1) - longi_measure;

    // 构造GNSS位置观测矩阵
    // construct GNSS position measurement matrix
    H.resize(1, StateDim);
    H.setZero();
    H(0, P_ID + 1) = 1;

    // 位置速度标准差
    Eigen::Vector2d rnre = Earth::meridianPrimeVerticalRadius(pvacur_.pos(0));
    Eigen::Vector2d rnhreh = rnre + Eigen::Vector2d::Ones() * pvacur_.pos[2];
    Eigen::MatrixXd measure_std(1, 1);
    measure_std(0, 0) = longi_cov / (cos(pvacur_.pos[0]) * rnhreh[1]);
    R = measure_std.cwiseProduct(measure_std).asDiagonal();

    // EKF更新协方差和误差状态
    // do EKF update to update covariance and error state
    EKFUpdate(Z, H, R);

    // 检查协方差矩阵对角线元素
    // check diagonal elements of current covariance matrix
    checkCov();

    // 将观测得到的误差状态反馈到导航状态中
    // feedback error state to navigation state
    stateFeedback();
}

void InsPvStekf15states::altiUpdate(const double alti_measure, const double alti_cov)
{
    // 观测：

    Eigen::MatrixXd Z;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;

    // 测量新息
    // compute innovation
    Z.resize(1, 1);
    Z(0, 0) = pvacur_.pos(2) - alti_measure;

    //打印输出矩阵Z
    //  std::cout << "Z:  " << Z << std::endl;

    // 构造GNSS位置观测矩阵
    // construct GNSS position measurement matrix
    H.resize(1, StateDim);
    H.setZero();
    H(0, P_ID + 2) = 1;

    // 位置标准差
    Eigen::MatrixXd measure_std(1, 1);
    measure_std(0, 0) = alti_cov;
    R = measure_std.cwiseProduct(measure_std).asDiagonal();

    // EKF更新协方差和误差状态
    // do EKF update to update covariance and error state
    EKFUpdate(Z, H, R);

    // 检查协方差矩阵对角线元素
    // check diagonal elements of current covariance matrix
    checkCov();

    // 将观测得到的误差状态反馈到导航状态中
    // feedback error state to navigation state
    stateFeedback();
}

// 直接赋值更新高度、
void InsPvStekf15states::setAltitude(const double alti_measure)
{

    // //直接赋值更新高度
    // pvacur_.pos(2) = alti_measure;

    Eigen::MatrixXd Z;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;

    // 测量新息
    // compute innovation
    Z.resize(1, 1);
    Z(0, 0) = pvacur_.pos(2) - alti_measure;

    // 构造GNSS位置观测矩阵
    // construct GNSS position measurement matrix
    H.resize(1, StateDim);
    H.setZero();
    H(0, P_ID + 2) = 1;

    // 位置标准差
    Eigen::MatrixXd measure_std(1, 1);
    measure_std(0, 0) = 2;
    R = measure_std.cwiseProduct(measure_std).asDiagonal();

    // EKF更新协方差和误差状态
    // do EKF update to update covariance and error state
    assert(H.cols() == Cov_.rows());
    assert(Z.rows() == H.rows());
    assert(Z.rows() == R.rows());
    assert(Z.cols() == 1);

    // 计算Kalman增益
    // Compute Kalman Gain
    auto temp = H * Cov_ * H.transpose() + R;
    Eigen::MatrixXd K = Cov_ * H.transpose() * temp.inverse();

    // 更新系统误差状态和协方差
    // update system error state and covariance
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(StateDim, StateDim) - K * H;
    // 如果每次更新后都进行状态反馈，则更新前dx_一直为0，下式可以简化为：dx_ = K * dz;
    // if state feedback is performed after every update, dx_ is always zero before the update
    // the following formula can be simplified as : dx_ = K * dz;
    dx_ = dx_ + K * (Z - H * dx_);
    // Cov_ = I * Cov_ * I.transpose() + K * R * K.transpose();

    // 检查协方差矩阵对角线元素
    // check diagonal elements of current covariance matrix
    checkCov();

    // 将观测得到的误差状态反馈到导航状态中
    // feedback error state to navigation state
    // 速度误差反馈
    // velocity error feedback
    pvacur_.vel -= dx_.block<3, 1>(V_ID, 0) + Rotation::skewSymmetric(pvacur_.vel) * dx_.block<3, 1>(PHI_ID, 0);

    // 位置误差反馈
    // position error feedback
    pvacur_.pos -= dx_.block<3, 1>(P_ID, 0);

    // 姿态误差反馈
    // attitude error feedback
    Eigen::Quaterniond delta_q = Rotation::rotvec2quaternion(dx_.block<3, 1>(PHI_ID, 0));
    pvacur_.att.qbn = delta_q * pvacur_.att.qbn;
    pvacur_.att.cbn = Rotation::quaternion2matrix(pvacur_.att.qbn);
    pvacur_.att.euler = Rotation::matrix2euler(pvacur_.att.cbn);

    // IMU零偏误差反馈
    // // IMU bias error feedback
    imuerror_.gyrbias += dx_.block<3, 1>(BG_ID, 0);
    imuerror_.accbias += dx_.block<3, 1>(BA_ID, 0);

    // 误差状态反馈到系统状态后,将误差状态清零
    // set 'dx' to zero after feedback error state to system state
    dx_.setZero();
}

void InsPvStekf15states::EKFPredict(const Eigen::MatrixXd &Phi, const Eigen::MatrixXd &Qd)
{

    assert(Phi.rows() == Cov_.rows());
    assert(Qd.rows() == Cov_.rows());

    // 传播系统协方差和误差状态
    // propagate system covariance and error state
    Cov_ = Phi * Cov_ * Phi.transpose() + Qd;
    dx_ = Phi * dx_;
}

void InsPvStekf15states::EKFUpdate(const Eigen::MatrixXd &dz, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R)
{

    assert(H.cols() == Cov_.rows());
    assert(dz.rows() == H.rows());
    assert(dz.rows() == R.rows());
    assert(dz.cols() == 1);

    // 计算Kalman增益
    // Compute Kalman Gain
    auto temp = H * Cov_ * H.transpose() + R;
    Eigen::MatrixXd K = Cov_ * H.transpose() * temp.inverse();

    // 更新系统误差状态和协方差
    // update system error state and covariance
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(StateDim, StateDim) - K * H;
    // 如果每次更新后都进行状态反馈，则更新前dx_一直为0，下式可以简化为：dx_ = K * dz;
    // if state feedback is performed after every update, dx_ is always zero before the update
    // the following formula can be simplified as : dx_ = K * dz;
    dx_ = dx_ + K * (dz - H * dx_);
    Cov_ = I * Cov_ * I.transpose() + K * R * K.transpose();
}

void InsPvStekf15states::stateFeedback()
{

    // 速度误差反馈
    // velocity error feedback
    pvacur_.vel -= dx_.block<3, 1>(V_ID, 0) + Rotation::skewSymmetric(pvacur_.vel) * dx_.block<3, 1>(PHI_ID, 0);

    // 位置误差反馈
    // position error feedback
    pvacur_.pos -= dx_.block<3, 1>(P_ID, 0);

    // 姿态误差反馈
    // attitude error feedback
    Eigen::Quaterniond delta_q = Rotation::rotvec2quaternion(dx_.block<3, 1>(PHI_ID, 0));
    pvacur_.att.qbn = delta_q * pvacur_.att.qbn;
    pvacur_.att.cbn = Rotation::quaternion2matrix(pvacur_.att.qbn);
    pvacur_.att.euler = Rotation::matrix2euler(pvacur_.att.cbn);

    // IMU零偏误差反馈
    // IMU bias error feedback
    imuerror_.gyrbias += dx_.block<3, 1>(BG_ID, 0);
    imuerror_.accbias += dx_.block<3, 1>(BA_ID, 0);

    // 误差状态反馈到系统状态后,将误差状态清零
    // set 'dx' to zero after feedback error state to system state
    dx_.setZero();
}

auto InsPvStekf15states::getNavState() const -> NavState
{

    NavState state;

    state.timestamp = timestamp_;
    state.utc_ns = utc_ns_;
    state.raw_sn = raw_sn;

    state.pos = pvacur_.pos;
    state.vel = pvacur_.vel;
    state.euler = pvacur_.att.euler;
    state.imuerror = imuerror_;

    return state;
}
