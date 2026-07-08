#pragma once

#include <Eigen/Dense>

#include "common/types.h"

#include "kf_gins_types.h"

class InsPvStekf15states {

public:
    static constexpr int NoiseDim{12}, StateDim{15};

    // 状态ID和噪声ID
    // state ID and noise ID
    enum StateID { PHI_ID = 0, V_ID = 3, P_ID = 6, BG_ID = 9, BA_ID = 12 };
    enum NoiseID { ARW_ID = 0, VRW_ID = 3, BGRW_ID = 6, BARW_ID = 9 };

    explicit InsPvStekf15states(KFOptions &options);

    ~InsPvStekf15states() = default;

    /**
     * @brief 进行INS状态更新(IMU机械编排算法), 并计算IMU状态转移矩阵和噪声阵
     *        do INS state update(INS mechanization), and compute state transition matrix and noise matrix
     * @param [in,out] imupre 前一时刻IMU数据
     *                        imudata at the previous epoch
     * @param [in,out] imucur 当前时刻IMU数据
     *                        imudata at the current epoch
     * */
    void insPropagation(const IMU &imu1_in, const IMU &imu2_in, const double dt);

    /**
     * @brief 使用GNSS位置观测更新系统状态
     *        update state using gnss position
     * @param [in,out] gnssdata
     * */
    void altiUpdate(const double alti_measure, const double alti_cov);
    void latiUpdate(const double lati_measure, const double lati_cov);
    void longiUpdate(const double longi_measure, const double longi_cov);
    void velUpdate(const Eigen::Vector3d &vel_measure, const Eigen::Vector3d &vel_cov);
    void posUpdate(const Eigen::Vector3d &pos_measure, const Eigen::Vector3d &pos_cov);
    //range
    void rangeUpdate(const double &range_measure, const Eigen::Vector3d &range_pos,
        const Eigen::Vector3d &range_pos_cov);
    void batchRangeUpdate(const std::vector<rangeAnchorData> &anchors, const double &range_pos_cov); 
    void setAltitude(const double alti_measure);

    

    /**
     * @brief 获取当前时间
     *        get current time
     * */
    auto timestamp() const -> uint64_t {
        return timestamp_;
    }

    void setUtcNs(uint64_t utc_ns_param) {
        utc_ns_ = utc_ns_param;
    }

    auto utc() const -> uint64_t {
        return utc_ns_;
    }

    auto raw_imu_sn() const -> uint64_t {
        return raw_sn;
    }

    /**
     * @brief 获取当前IMU状态
     *        get current navigation state
     * */
    auto getNavState() const -> NavState;

    auto getPVA() const -> PVA {
        return pvacur_;
    }

    /**
     * @brief 获取当前状态协方差
     *        get current state covariance
     * */
    auto getCovariance() const -> Eigen::MatrixXd {
        return Cov_;
    }

private:
    /**
     * @brief 初始化系统状态和协方差
     *        initialize state and state covariance
     * @param [in] initstate     初始状态
     *                           initial state
     * @param [in] initstate_std 初始状态标准差
     *                           initial state std
     * */
    void initialize(const NavState &initstate, const NavState &initstate_std);

    /**
     * @brief Kalman 预测,
     *        Kalman Filter Predict process
     * @param [in,out] Phi 状态转移矩阵
     *                     state transition matrix
     * @param [in,out] Qd  传播噪声矩阵
     *                     propagation noise matrix
     * */
    void EKFPredict(const Eigen::MatrixXd &Phi, const Eigen::MatrixXd &Qd);

    /**
     * @brief Kalman 更新
     *        Kalman Filter Update process
     * @param [in] dz 观测新息
     *                measurement innovation
     * @param [in] H  观测矩阵
     *                measurement matrix
     * @param [in] R  观测噪声阵
     *                measurement noise matrix
     * */
    void EKFUpdate(const Eigen::MatrixXd &dz, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R);

    /**
     * @brief 反馈误差状态到当前状态
     *        feedback error state to the current state
     * */
    void stateFeedback();

    /**
     * @brief 检查协方差对角线元素是否都为正
     *        Check if covariance diagonal elements are all positive
     * */
    void checkCov() {

        for (int i = 0; i < StateDim; i++) {
            if (Cov_(i, i) < 0) {
                std::cout << "Covariance is negative at " << std::setprecision(10) << timestamp_ << " !" << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
    }

private:
    KFOptions options_;

    uint64_t timestamp_;
    uint64_t utc_ns_;
    uint64_t raw_sn; // 原始时间戳*200，作为序列号使用

    // IMU状态（位置、速度、姿态和IMU误差）
    // imu state (position, velocity, attitude and imu error)
    PVA pvacur_;
    ImuError imuerror_;

    // Kalman滤波相关
    // ekf variables
    Eigen::MatrixXd Cov_;
    Eigen::MatrixXd Qc_;
    Eigen::MatrixXd dx_;
};
