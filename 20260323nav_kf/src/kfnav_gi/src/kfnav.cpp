#include "UtilityTool.h"
#include <imu_common.h>
#include <kfnav.h>
#include <param_common.h>
#include <param_loader.h>
#include <cmath>
#include <Eigen/Core>
#include <numeric>
#include <limits>

const uint64_t u1e9 = 1000000000L;
const double replay_rate = 0; // File replay rate, 0 disables delay

const static uint64_t secSigniDigit = 2;    // Precision for seconds
const static uint64_t meterSigniDigit = 4;  // Precision for meters
const static uint64_t degreeSigniDigit = 8; // Precision for degrees

KfNav::KfNav()
    : nh_(), private_nh_("~"), state0_sem(0)
      , m_time_last(-1)
{
    ROS_INFO(" -- KF-NAV : GNSS-INS STEKF loosely coupled kalman filter -- ");

    std::string param_file;
    private_nh_.param<std::string>("param_file", param_file, "");

    if (param_file.empty())
    {
        ROS_ERROR("no ROS parameter:[param_file] found");
        ros::shutdown();
        return;
    }
    
    ROS_INFO_STREAM("[param_file]: " << param_file);
    pload->load(param_file);
    config = YAML::LoadFile(param_file);

    std::cout << " #####\nCalib:              :" << std::endl;
    imu_calib = ImuCalib(config["calib"]);
    imu_calib.expr();

    private_nh_.param<int>("redirect_from_file", is_redirect_from_file, 0);

    if (is_redirect_from_file != 0)
    {
        printf("#####file start\n");
        std::string track_name;
        private_nh_.param<std::string>("track", track_name, "");

        if (track_name.empty())
        {
            ROS_ERROR("no ROS parameter:[track] found");
            ros::shutdown();
            return;
        }
        
        ROS_INFO_STREAM("[track]: " << track_name);
        
        FileLoader fimu(track_name + "/imu.txt", 20);
        FileLoader fmag(track_name + "/mag_origin.txt", 4);
        FileLoader fgnss_origin(track_name + "/gnss_origin.txt", 9);
        FileLoader fbaro(track_name + "/baro.txt", 2);
        FileLoader frange(track_name + "/range.txt", 20);

        while (!fimu.isEof())
        {
            bool valid;
            auto data = fimu.load(valid);
            if (valid)
            {
                callback_inertial(data);
            }
            else
            {
                LOG_INFO("IMU: Empty line");
            }
        }

        if (fmag.isOpen())
        {
            while (!fmag.isEof())
            {
                bool valid;
                auto data = fmag.load(valid);
                if (valid)
                {
                    callback_mag(data);
                }
                else
                {
                    LOG_INFO("MAG: Empty line");
                }
            }
        }

        while (!fgnss_origin.isEof())
        {
            bool valid;
            auto data = fgnss_origin.load(valid);
            if (valid)
            {
                callback_gnss_origin(data);
            }
            else
            {
                LOG_INFO("GNSS Origin: Empty line");
            }
        }
        while (!fbaro.isEof())
        {
            bool valid;
            auto data = fbaro.load(valid);
            if (valid)
            {
                callback_baro(data);
            }
            else
            {
                LOG_INFO("Baro: Empty line");
            }
        }

        while (!frange.isEof())
        {
            bool valid;
            auto data = frange.load(valid);
            if (valid)
            {
                callback_range(data);
            }
            else
            {
                LOG_INFO("Range: Empty line");
            }
        }
    }
    else
    {
        printf("#####ros start\n");
        imu_sub = nh_.subscribe<std_msgs::Float64MultiArray>(
            pload->imu_topic, 100,
            [this](const std_msgs::Float64MultiArray::ConstPtr &msg)
            { callback_inertial(msg->data); });
        
        gnss_origin_sub = nh_.subscribe<std_msgs::Float64MultiArray>(
            pload->gnss_origin_topic, 100,
            [this](const std_msgs::Float64MultiArray::ConstPtr &msg)
            { callback_gnss_origin(msg->data); });

        baro_sub = nh_.subscribe<std_msgs::Float64MultiArray>(
            pload->baro_topic, 100,
            [this](const std_msgs::Float64MultiArray::ConstPtr &msg)
            { callback_baro(msg->data); });

        range_sub = nh_.subscribe<std_msgs::Float64MultiArray>(
            pload->range_topic, 100,
            [this](const std_msgs::Float64MultiArray::ConstPtr &msg)
            { callback_range(msg->data); });

        if (pload->is_mag_yaw_using)
        {
            mag_sub = nh_.subscribe<std_msgs::Float64MultiArray>(
                pload->mag_topic, 100,
                [this](const std_msgs::Float64MultiArray::ConstPtr &msg)
                { callback_mag(msg->data); });
        }
    }

    gi_pub = nh_.advertise<std_msgs::Float64MultiArray>(pload->gi_topic, 100);

    private_nh_.param<bool>("is_gnss_using", is_gnss_using, true);


    std::string dumpdir;
    private_nh_.param<std::string>("dump_root", dumpdir, "");

    if (dumpdir.empty())
    {
        ROS_ERROR("no ROS parameter:[dump_root] found");
        ros::shutdown();
        return;
    }
    
    ROS_INFO_STREAM("dump_root" << dumpdir);
    fs.open(dumpdir +
                std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
                                   std::chrono::system_clock::now().time_since_epoch())
                                   .count()) +
                "nav.txt",
            0, FileSaver::TEXT);

    std::thread thread_ins_pv(&KfNav::gnss_ins_thread, this);
    thread_ins_pv.detach();
}

void KfNav::callback_inertial(std::vector<double> data)
{
    if (data.size() == 0)
    {
        IMU message;
        message.time = UINT64_MAX;
        main_imu_buf.push(message);
        ROS_INFO_STREAM_ONCE("File redirect end. IMU data received: " << main_imu_buf.getPushCnt());
        return;
    }

    auto imu_steady_time = data[1];
    m_time_last = imu_steady_time;
    //printf("last imu time %.3f \n",data[1]);
    IMU message;
    message.time = data[0] * 1e9;
    //rintf("imu message time %.5f \n",data[0]*1e9);
    message.dt = 1.0 / pload->imudatarate;
    message.raw_sn = std::round(data[1] * 200);

    message.dtheta << data[2], data[3], data[4];
    message.dvel << data[5], data[6], data[7];
    //printf("push before %lf vel:%lf\n",data[2],data[5]);
    main_imu_buf.push(message);
    if (main_imu_buf.getPushCnt() == static_cast<uint64_t>(pload->imudatarate))
    {
        ROS_INFO_STREAM("One second of IMU data received. ");
    }
}

auto KfNav::waitIMU_main() -> IMU
{
    auto imu = main_imu_buf.pop();
    if (imu.time == UINT64_MAX)
    {
        ROS_INFO_STREAM("IMU data end (file mode).");
        imu.time = 0;
        return imu;
    }
    return ImuCommon::imuCalibration(imu, imu_calib);
}

auto KfNav::waitIMU_rt() -> IMU
{
    auto imu = rt_imu_buf.pop();
    if (imu.time == UINT64_MAX)
    {
        ROS_INFO_STREAM("IMU data end.");
        ros::shutdown();
        return imu;
    }
    return imu;
}

void KfNav::callback_gnss_origin(std::vector<double> data)
{
    if (data.size() == 0)
    {
        GNSS message;
        message.time = UINT64_MAX;
        gnss_origin_buf.push(message);
        ROS_INFO_STREAM_ONCE("File redirect end. GNSS Origin data received: " << gnss_origin_buf.getPushCnt());
        return;
    }

    GNSS message;
    message.time = data[0] * u1e9;

    message.flag_utc = false;
    message.utc_time = data[6] * u1e9;

    message.flag_pos = true;
    message.blh << data[3] * D2R, data[2] * D2R, data[4];
    message.std << 1, 1, 3;

    message.flag_vel = false;
    message.vel << 0, 0, 0;
    message.vstd << INFINITY, INFINITY, INFINITY;

    gnss_origin_buf.push(message);
    ROS_INFO_STREAM_ONCE("One GNSS Origin data received. ");
}

auto KfNav::waitGNSS_Origin() -> GNSS
{
    auto gnss = gnss_origin_buf.pop();
    if (gnss.time == UINT64_MAX)
    {
        ROS_INFO_STREAM("GNSS Origin data end.");
        ros::shutdown();
        return gnss;
    }
    return gnss;
}

void KfNav::callback_mag(std::vector<double> data)
{
    if (data.size() == 0)
    {
        MAG message;
        message.time = UINT64_MAX;
        mag_buf.push(message);
        ROS_INFO_STREAM_ONCE("File redirect end. MAG data received: " << mag_buf.getPushCnt());
        return;
    }
    if (data.size() < 4)
    {
        return;
    }

    MAG message;
    message.time = data[0] * u1e9;
    message.mag << data[1], data[2], data[3];
    if (std::isfinite(message.mag.x()) && std::isfinite(message.mag.y()) && std::isfinite(message.mag.z()))
    {
        mag_buf.push(message);
        ROS_INFO_STREAM_ONCE("One MAG data received. ");
    }
}

auto KfNav::magYawInWindow(uint64_t start_time, uint64_t end_time, const Eigen::Vector3d &roll_pitch_yaw,
                           double &yaw) -> bool
{
    if (!pload->is_mag_yaw_using || mag_buf.empty())
    {
        return false;
    }

    Eigen::Vector3d mag_sum = Eigen::Vector3d::Zero();
    size_t mag_count = 0;
    while (!mag_buf.empty() && mag_buf.front().time < start_time)
    {
        mag_buf.pop();
    }
    for (size_t i = 0; i < mag_buf.size(); i++)
    {
        const auto mag = mag_buf.at(i);
        if (mag.time == UINT64_MAX)
        {
            break;
        }
        if (mag.time > end_time)
        {
            break;
        }
        mag_sum += mag.mag;
        mag_count++;
    }
    if (mag_count == 0)
    {
        return false;
    }

    const Eigen::Vector3d mag_mean = mag_sum / static_cast<double>(mag_count);
    if (mag_mean.norm() < std::numeric_limits<double>::epsilon())
    {
        return false;
    }

    const double roll = roll_pitch_yaw[0];
    const double pitch = roll_pitch_yaw[1];
    const double mx = mag_mean[0];
    const double my = mag_mean[1];
    const double mz = mag_mean[2];

    const double mx_h = mx * cos(pitch) + my * sin(roll) * sin(pitch) + mz * cos(roll) * sin(pitch);
    const double my_h = my * cos(roll) - mz * sin(roll);
    yaw = atan2(my_h, mx_h);

    ROS_INFO_STREAM("MAG yaw used in initial alignment. samples: " << mag_count
                    << ", mean[uT]: " << mag_mean.transpose()
                    << ", yaw[deg]: " << yaw * R2D);
    return true;
}

void KfNav::callback_baro(std::vector<double> data)
{
    if (data.size() == 0)
    {
        ROS_INFO_STREAM_ONCE("File redirect end. Baro data received: " << baro_buf.getPushCnt());
        return;
    }
    else
    {
        BARO message;
        message.time = data[0] * 1e9;
        message.pressure = data[1];
        bool valid = true;
        for (double value : data)
        {
            if (value < -100)
            {
                valid = false;
                break;
            }
        }
        if (valid)
        {
            baro_buf.push(message);
            ROS_INFO_STREAM_ONCE("One Baro data received. ");
        }
        else
        {
            ROS_WARN("Received invalid baro data with negative values.");
        }
        return;
    }
}

void KfNav::callback_range(std::vector<double> data)
{
    if (data.size() == 0)
    {
        ROS_INFO_STREAM_ONCE("File redirect end. Range data received: " << range_buf.getPushCnt());
        return;
    }

    RANGE message;
    message.time = data[0] * u1e9;

    std::vector<int> anchor_ids = {
        pload->range_id_1_3[0], pload->range_id_1_3[1], pload->range_id_1_3[2],
        pload->range_id_4_6[0], pload->range_id_4_6[1], pload->range_id_4_6[2],
        pload->range_id_7_9[0], pload->range_id_7_9[1], pload->range_id_7_9[2],
        pload->range_id_10_12[0], pload->range_id_10_12[1], pload->range_id_10_12[2],
        pload->anchor_id_1_3[0], pload->anchor_id_1_3[1], pload->anchor_id_1_3[2]};

    std::vector<bool> flags(anchor_ids.size(), false);

    for (int i = 1; i < data.size();)
    {
        for (int j = 0; j < anchor_ids.size(); ++j)
        {
            if (data[i] == anchor_ids[j] && !flags[j])
            {
                rangeAnchorData anchor;
                anchor.tag = anchor_ids[j];
                anchor.position = Eigen::Vector3d(data[i + 1], data[i + 2], data[i + 3]);
                anchor.range = data[i + 4];
                message.anchors.push_back(anchor);
                flags[j] = true;
                i += 4;
                break;
            }
        }
        ++i;
    }

    range_buf.push(message);
    ROS_INFO_STREAM_ONCE("One Range data received. ");
}

auto KfNav::waitBARO() -> BARO
{
    auto baro = baro_buf.pop();
    if (baro.time == UINT64_MAX)
    {
        ROS_INFO_STREAM("Baro data end (file mode).");
        baro.time = 0;
        return baro;
    }
    return baro;
}

auto KfNav::waitRANGE() -> RANGE
{
    auto range = range_buf.pop();
    if (range.time == UINT64_MAX)
    {
        ROS_INFO_STREAM("Range data end (file mode).");
        range.time = 0;
        return range;
    }
    return range;
}

void KfNav::realtime_ins_thread()
{
    PIEngine pi;
    uint64_t pub_sn;
    uint64_t last_state0_update_cnt;
    state0_sem.wait();
    {
        std::lock_guard<std::mutex> lock(state0_mutex);
        pub_sn = state0.raw_sn;
        pi.initialize(state0);
        last_state0_update_cnt = state0_update_cnt;
        assert(pub_sn % 2 == 0);
    }
    IMU imu1, imu2;
    std::deque<IMU> imu_log;
    imu_log.clear();
    while (ros::ok())
    {
        imu1 = waitIMU_rt();
        imu2 = waitIMU_rt();

        {
            std::lock_guard<std::mutex> lock(state0_mutex);
            if (state0_update_cnt > last_state0_update_cnt)
            {
                pi.initialize(state0);
                last_state0_update_cnt = state0_update_cnt;
                while (imu_log.front().raw_sn <= pi.raw_imu_sn())
                {
                    imu_log.pop_front();
                }
                IMU oldimu1, oldimu2;
                for (auto ptr = imu_log.cbegin(); ptr != imu_log.cend();)
                {
                    oldimu1 = *ptr;
                    ptr++;
                    oldimu2 = *ptr;
                    ptr++;
                    pi.newImuProcess(oldimu1, oldimu2);
                }
                assert(pi.raw_imu_sn() == pub_sn);
            }
        }

        pi.newImuProcess(imu1, imu2);
        imu_log.push_back(imu1);
        imu_log.push_back(imu2);
        if (1)
        {
        }
        pub_sn = pi.raw_imu_sn();
        assert(imu_log.back().raw_sn == pi.raw_imu_sn());
        while (imu_log.size() > IMU_LOG_MAX)
        {
            LOG_WARN("IMU LOG size over limit, pop front.");
            imu_log.pop_front();
        }
    }
}

void KfNav::gnss_ins_thread()
{
    assert(pload->ca_timelen > 0);

    IMU imu1, imu2;
    GNSS gnss;
    BARO baro;
    RANGE range;

    CoarseAlignI0 ca;
    CoarseAlignCompass ca_compass;
    const auto num_ca_sample = static_cast<size_t>(pload->ca_timelen * pload->imudatarate);
    while (main_imu_buf.size() < num_ca_sample)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    LOG_INFO("IMU data enough, start initial alignment.");
    for (size_t i = 0; i < num_ca_sample; i++)
    {
        imu1 = ImuCommon::imuCalibration(main_imu_buf.at(i), imu_calib);
        ROS_INFO_ONCE("Init IMU data used.");
        align_start_time = std::min(align_start_time, imu1.time);
        ca_end_time = std::max(ca_end_time, imu1.time);
        ca.update(imu1);
        ca_compass.addImuData(imu1);
    }

    Eigen::Vector3d pos_mean;
    {
        std::vector<GNSS> gnssvec;
        gnssvec.clear();
        printf("###Wait for GNSS data ready to calculate the initial position...\n");
        while (gnss_origin_buf.empty() || gnss_origin_buf.back().time < ca_end_time)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        while (!gnss_origin_buf.empty() && gnss_origin_buf.front().time < align_start_time)
        {
            waitGNSS_Origin();
        }
        for (size_t i = 0; i < gnss_origin_buf.size() && gnss_origin_buf.at(i).time <= ca_end_time; i++)
        {
            gnss = gnss_origin_buf.at(i);
            ROS_INFO_ONCE("Init GNSS data used.");
            //printf("blh:%f %f %f",gnss.blh(0),gnss.blh(1),gnss.blh(2));
            gnssvec.push_back(gnss);
        }
        LOG_INFO("GNSS during CA num: " << gnssvec.size());

        Eigen::Vector3d pos_sum = Eigen::Vector3d::Zero();
        for (auto gnss_item : gnssvec)
        {
            pos_sum += gnss_item.blh;
        }
        pos_mean = pos_sum / gnssvec.size();
    }
    dispVector("Init Position mean: ", pos_mean.cwiseProduct(Eigen::Vector3d(R2D, R2D, 1.0)));

    ca.set(pos_mean[0], pos_mean[2], 1.0 / pload->imudatarate);
    auto cai0_atti = ca.calc();
    const auto compass_euler = ca_compass.getEuler(false);
    const auto i0_euler = cai0_atti.euler;
    cai0_atti.euler[0] = compass_euler[0];
    cai0_atti.euler[1] = compass_euler[1];
    cai0_atti.cbn = Rotation::euler2matrix(cai0_atti.euler);
    cai0_atti.qbn = Rotation::euler2quaternion(cai0_atti.euler);
    ROS_INFO_STREAM("Initial roll/pitch replaced by CoarseAlignCompass. I0 roll/pitch[deg]: "
                    << i0_euler[0] * R2D << ", " << i0_euler[1] * R2D
                    << "; compass roll/pitch[deg]: " << compass_euler[0] * R2D << ", "
                    << compass_euler[1] * R2D);
    if (pload->is_mag_yaw_using)
    {
        int mag_wait_cnt = 0;
        while ((mag_buf.empty() || mag_buf.back().time < ca_end_time) && ros::ok() && mag_wait_cnt < 100)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            mag_wait_cnt++;
        }
    }
    double mag_yaw = 0;
    if (magYawInWindow(align_start_time, ca_end_time, cai0_atti.euler, mag_yaw))
    {
        const double coarse_yaw = cai0_atti.euler[2];
        cai0_atti.euler[2] = mag_yaw;
        cai0_atti.cbn = Rotation::euler2matrix(cai0_atti.euler);
        cai0_atti.qbn = Rotation::euler2quaternion(cai0_atti.euler);
        ROS_INFO_STREAM("Initial yaw replaced by magnetometer. coarse yaw[deg]: "
                        << coarse_yaw * R2D << ", mag yaw[deg]: " << mag_yaw * R2D);
    }
    else if (pload->is_mag_yaw_using)
    {
        ROS_WARN("Magnetometer yaw requested, but no valid MAG samples were available in the CA window. "
                 "Using coarse alignment yaw.");
    }
    dispVector("CA Euler: ", cai0_atti.euler * R2D);
    std::cout << "CA Duration: [ " << align_start_time << ", " << ca_end_time << "]" << std::endl;

    NavState initstate(config["initstate"]);
    {
        initstate.euler = cai0_atti.euler;
        initstate.vel = Eigen::Vector3d::Zero();
        initstate.pos = pos_mean;
        initstate.imuerror.gyrbias = Eigen::Vector3d::Zero();
        initstate.imuerror.accbias = Eigen::Vector3d::Zero();
    }

    KFOptions initoptions;
    {
        initoptions.initstate = initstate;
        initoptions.initstate_std = NavState(config["initstd"]);
        initoptions.prop_noise = ImuRw(config["prop_noise"]);
    }
    InsPvStekf15states gi_kf(initoptions);
    NavState gistate;
    uint64_t update_cnt = 0;
    double rms_norm_omega;
    std::vector<IMU> imuvec;
    Eigen::Vector2d rnre = Earth::meridianPrimeVerticalRadius(pos_mean(0));
    Eigen::Vector2d rnhreh = rnre + Eigen::Vector2d::Ones() * pos_mean(2);
    double radiusMeridian = rnhreh(0);
    double radiusLatiCircle = rnhreh(1) * cos(pos_mean(0));
    std::cout << "local radiusMeridian: " << radiusMeridian << std::endl;
    std::cout << "local radiusLatiCircle: " << radiusLatiCircle << std::endl;
    imuvec.clear();
    //清空 main_imu_buf 中的所有元素
    //通过获取当前大小然后弹出相应数量的元素来模拟清空操作
    size_t current_size_imu = main_imu_buf.size();
    for (size_t i = 0; i < current_size_imu; ++i) {
        main_imu_buf.pop();
    }
    size_t current_size_gnss = gnss_origin_buf.size();
    for (size_t i = 0; i < current_size_gnss; ++i) {
            gnss_origin_buf.pop();
    }
    //main_imu_buf.clear();
    bool is_updating = true;
    bool is_gnss_update = true;
    bool is_baro_using = pload->is_baro_using;
    bool is_baro_using_kf = pload->is_baro_using_kf;
    bool is_baro_update = true;
    bool is_range_using = pload->is_range_using;
    bool is_range_update = true;
    bool is_gnss_using = true;
    uint64_t skipped_gnss_pv_cnt = 0;
    uint64_t used_gnss_pv_cnt = 0;
    double sum_sq_n2 = 0;
    double sum_sq_e2 = 0;
    uint64_t predict_cnt = 0;
    uint64_t imu_step = 0;
    
    while (ros::ok())
    {
        imu1 = waitIMU_main();
        imu2 = waitIMU_main();

        if (pload->zupt_timelength > 0)
        {
            imuvec.push_back(imu1);
            imuvec.push_back(imu2);
        }

        gi_kf.insPropagation(imu1, imu2, 1.0 / pload->imudatarate);
        predict_cnt++;
        imu_step++;

        gistate = gi_kf.getNavState();

        if (imu_step % (pload->imudatarate / 2 / pload->nav_pub_rate) == 0)
        {
            Eigen::Matrix3d cbn = gi_kf.getPVA().att.cbn;
            Eigen::Matrix3d cmb;
            cmb << 0, -1, 0, 1, 0, 0, 0, 0, 1;
            Eigen::Matrix3d cmn = cbn * cmb;

            NavState publishstate = gistate;
            publishstate.euler = Rotation::matrix2euler(cmn);

            if (gi_kf.timestamp() > ca_end_time)
            {
                publishNavState(gistate);
            }
            writeNavResult(update_cnt, predict_cnt, gistate, gnss, gi_kf.getCovariance(), rms_norm_omega);
        }

        if (is_redirect_from_file && imu_step > pload->calc_timelen * pload->imudatarate / 2)
        {
            ROS_INFO_STREAM("calc stop");
            ros::shutdown();
        }

        if (!gnss_origin_buf.empty() && gnss_origin_buf.front().time < std::max(imu1.time, imu2.time))
        {
            gnss = waitGNSS_Origin();
            is_gnss_update = true;
        }
        else
        {
            is_gnss_update = false;
        }

        if (!baro_buf.empty() && baro_buf.front().time < std::max(imu1.time, imu2.time))
        {
            baro = waitBARO();
            is_baro_update = true;
        }
        else
        {
            is_baro_update = false;
        }

        if (!range_buf.empty() && range_buf.front().time < std::max(imu1.time, imu2.time))
        {
            range = waitRANGE();
            is_range_update = true;
        }
        else
        {
            is_range_update = false;
        }

        if (imu_step > pload->gnss_timelen * pload->imudatarate / 2)
        {
            is_updating = false;
        }

        private_nh_.getParam("is_gnss_using", is_gnss_using);
        private_nh_.getParam("is_range_using", is_range_using);

        ROS_INFO_STREAM_ONCE("is_gnss_using: " << is_gnss_using << ", is_updating: " << is_updating << ", is_gnss_update: " << is_gnss_update);

        if (is_gnss_using && is_updating && is_gnss_update)
        {   
            // printf("gnss time: %.1f s   ", gnss.time / 1e9);
            if (gnss.flag_utc)
            {
                gi_kf.setUtcNs(gnss.utc_time);
            }

            if (gnss.flag_pos)
            {
                if (1)
                {
                    gi_kf.posUpdate(gnss.blh, pload->gnss_pos_std);
                }
                else
                {
                    //printf("blh:%f %f %f \n",gnss.blh(0)*R2D,gnss.blh(1)*R2D,gnss.blh(2));
                    gi_kf.latiUpdate(gnss.blh(0), pload->gnss_pos_std[0]);
                    gi_kf.longiUpdate(gnss.blh(1), pload->gnss_pos_std[1]);
                    gi_kf.altiUpdate(gnss.blh(2), pload->gnss_pos_std[2]);
                }
            }
            if (gnss.flag_vel)
            {
                gi_kf.velUpdate(gnss.vel, gnss.vstd);
            }
            update_cnt++;
            used_gnss_pv_cnt++;
            predict_cnt = 0;
            sum_sq_n2 = 0;
            sum_sq_e2 = 0;
            skipped_gnss_pv_cnt = 0;
        }

        if (is_baro_update && is_baro_using)
        {
            // printf("baro time: %.1f s   ", baro.time / 1e9);
            double alti_baro = baro.pressure + pos_mean[2];

            if (is_baro_using_kf)
            {
                gi_kf.altiUpdate(alti_baro, pload->baro_std);
            }
            else
            {
                gi_kf.setAltitude(alti_baro);
            }
        }

        if (is_range_update && is_range_using)
        {
            for (auto &anchor : range.anchors)
            {
                // printf("range_data: %d, %f,%f,%f, %f\n", anchor.tag, anchor.range, anchor.position(0), anchor.position(1), anchor.position(2));
                if (anchor.range < 1)
                {
                    continue;
                }
                else
                {
                    gi_kf.rangeUpdate(anchor.range, anchor.position, pload->range_std);
                    //ROS_INFO_STREAM("range update");
                }
            }
        }

        auto pos_err = Eigen::Vector3d(radiusMeridian, radiusLatiCircle, 1).cwiseProduct(gistate.pos - gnss.blh);
        auto pos_err_horiz = sqrt(pos_err(0) * pos_err(0) + pos_err(1) * pos_err(1));
        auto pos_err_3d = sqrt(pos_err_horiz * pos_err_horiz + pos_err(2) * pos_err(2));
        
        // Find the closest GNSS reference position by timestamp
        if (!(is_gnss_using && is_updating) && is_gnss_update)
        {
            used_gnss_pv_cnt = 0;
            skipped_gnss_pv_cnt++;
            sum_sq_n2 += pos_err(0) * pos_err(0);
            sum_sq_e2 += pos_err(1) * pos_err(1);
            double cep = 0.59 * (sqrt(sum_sq_n2 / skipped_gnss_pv_cnt) + sqrt(sum_sq_e2 / skipped_gnss_pv_cnt));
            if (skipped_gnss_pv_cnt % pload->precisionEchoEpoch == 0)
            {
                ROS_INFO_STREAM("Navigation time: "
                                       << std::fixed << std::setprecision(secSigniDigit)
                                       << static_cast<double>(imu_step) / (pload->imudatarate / 2)
                                       << " Dead reckoning time: "
                                       << std::fixed << std::setprecision(secSigniDigit)
                                       << static_cast<double>(predict_cnt) / (pload->imudatarate / 2)
                                       << " sec, skipped GNSS data: " << skipped_gnss_pv_cnt << std::endl
                                       << "Estimated position: [Lat[deg]: " << std::setprecision(degreeSigniDigit)
                                       << gistate.pos(0) * R2D << ", Lon[deg]: " << gistate.pos(1) * R2D << ", Alt[m]: "
                                       << std::setprecision(meterSigniDigit) << gistate.pos(2) << "]" << std::endl
                                       << "Reference position: [Lat[deg]: " << std::setprecision(degreeSigniDigit)
                                       << gnss.blh(0) * R2D << ", Lon[deg]: " << gnss.blh(1) * R2D << ", Alt[m]: "
                                       << std::setprecision(meterSigniDigit) << gnss.blh(2) << "]" << std::endl
                                       << std::setprecision(meterSigniDigit) << "Position error[m]: [North: " << pos_err(1)
                                       << ", East: " << pos_err(0) << ", Alt: " << pos_err(2) << "]" << std::endl
                                       << "Horizontal position error[m]: " << pos_err_horiz << std::endl
                                       << "3D position error[m]: " << pos_err_3d << std::endl
                                       << "Horizontal CEP[m]: " << cep);
            }
        }

        if ((is_gnss_using && is_updating) && is_gnss_update && gi_kf.timestamp() > ca_end_time)
        {
            if (update_cnt % pload->precisionEchoEpoch == 0)
            {
                // Calculate error using current GNSS reference
                Eigen::Vector3d used_pos_err = Eigen::Vector3d(radiusMeridian, radiusLatiCircle, 1).cwiseProduct(gistate.pos - gnss.blh);
                double used_pos_err_horiz = sqrt(used_pos_err(0) * used_pos_err(0) + used_pos_err(1) * used_pos_err(1));
                double used_pos_err_3d = sqrt(used_pos_err_horiz * used_pos_err_horiz + used_pos_err(2) * used_pos_err(2));
                
                ROS_INFO_STREAM("Navigation time: "
                                       << std::fixed << std::setprecision(secSigniDigit)
                                       << static_cast<double>(imu_step) / (pload->imudatarate / 2)
                                       << " sec, GNSS data used: " << used_gnss_pv_cnt << std::endl
                                       << "Estimated position: [Lat[deg]: " << std::setprecision(degreeSigniDigit)
                                       << gistate.pos(0) * R2D << ", Lon[deg]: " << gistate.pos(1) * R2D << ", Alt[m]: "
                                       << std::setprecision(meterSigniDigit) << gistate.pos(2) << "]" << std::endl
                                       << "Reference position: [Lat[deg]: " << std::setprecision(degreeSigniDigit)
                                       << gnss.blh(0) * R2D << ", Lon[deg]: " << gnss.blh(1) * R2D << ", Alt[m]: "
                                       << std::setprecision(meterSigniDigit) << gnss.blh(2) << "]" << std::endl
                                       << std::setprecision(meterSigniDigit) << "Position error[m]: [North: " << used_pos_err(1)
                                       << ", East: " << used_pos_err(0) << ", Alt: " << used_pos_err(2) << "]" << std::endl
                                       << "Horizontal position error[m]: " << used_pos_err_horiz << std::endl
                                       << "3D position error[m]: " << used_pos_err_3d << std::endl);
            }
        }

        if (imuvec.size() == 200)
        {
            std::vector<double> norFIX2_omega_seq;
            std::transform(imuvec.begin(), imuvec.end(), std::back_inserter(norFIX2_omega_seq),
                           [](const IMU &imu)
                           { return (imu.dtheta * pload->imudatarate * R2D).cwiseAbs2().sum(); });
            rms_norm_omega =
                sqrt(std::accumulate(norFIX2_omega_seq.begin(), norFIX2_omega_seq.end(), 0.0) / norFIX2_omega_seq.size());
            imuvec.clear();
            if (is_updating && imu_step < static_cast<uint64_t>(pload->zupt_timelength * pload->imudatarate / 2) &&
                rms_norm_omega < pload->omega_threshold)
            {
                ROS_INFO_STREAM_ONCE("Zero speed measurement used. ");
                gi_kf.velUpdate(Eigen::Vector3d::Zero(), pload->velstd);
                update_cnt++;
                predict_cnt = 0;
            }
        }
        if (is_redirect_from_file && replay_rate != 0 && gi_kf.timestamp() > ca_end_time)
        {
            std::this_thread::sleep_for(
                std::chrono::microseconds(static_cast<uint64_t>(1e6 / pload->imudatarate * 2 / replay_rate)));
        }
    }
}

void KfNav::publishNavState(const NavState &nav_state)
{
    std_msgs::Float64MultiArray navstate_msg;
    navstate_msg.data.clear();
    navstate_msg.data.push_back(static_cast<double>(nav_state.timestamp / u1e9) +
                                static_cast<double>(nav_state.timestamp % u1e9) / 1e9);
    navstate_msg.data.push_back(static_cast<double>(nav_state.utc_ns / u1e9) +
                                static_cast<double>(nav_state.utc_ns % u1e9) / 1e9);
    navstate_msg.data.push_back(static_cast<double>(nav_state.raw_sn));
    navstate_msg.data.push_back(nav_state.euler[0] * R2D);
    navstate_msg.data.push_back(nav_state.euler[1] * R2D);
    navstate_msg.data.push_back(nav_state.euler[2] * R2D);
    navstate_msg.data.push_back(nav_state.vel[0]);
    navstate_msg.data.push_back(nav_state.vel[1]);
    navstate_msg.data.push_back(nav_state.vel[2]);
    navstate_msg.data.push_back(nav_state.pos[0] * R2D);
    navstate_msg.data.push_back(nav_state.pos[1] * R2D);
    navstate_msg.data.push_back(nav_state.pos[2]);
    gi_pub.publish(navstate_msg);
}

void KfNav::dispVector(std::string title, const Eigen::Vector3d &vec)
{
    std::cout << "    " << title << ": " << std::fixed << std::setprecision(10) << vec[0] << " " << vec[1] << " "
              << vec[2] << std::endl;
}

void KfNav::dispNavState(std::string title, const NavState &nav_state)
{
    std::cout << "--- " << title << std::endl;

    dispVector("Euler", nav_state.euler * R2D);
    dispVector("Velocity", nav_state.vel);
    dispVector("Position", nav_state.pos.cwiseProduct(Eigen::Vector3d(R2D, R2D, 1.0)));
    dispVector("AcclBias", nav_state.imuerror.accbias / 9.79e-9);
    dispVector("GyroBias", nav_state.imuerror.gyrbias * RS2DH);

    std::cout << "-\n\n";
}

void KfNav::writeNavResult(uint64_t update_cnt, uint64_t predict_cnt, const NavState &navstate, const GNSS &gnss,
                           const Eigen::MatrixXd &cov, double omega_rms)
{

    std::vector<double> result;

    result.clear();
    result.push_back(static_cast<double>(update_cnt));
    result.push_back(static_cast<double>(predict_cnt));
    result.push_back(static_cast<double>(navstate.timestamp / u1e9) +
                     static_cast<double>(navstate.timestamp % u1e9) / 1e9);
    result.push_back(static_cast<double>(navstate.utc_ns / u1e9) +
                     static_cast<double>(navstate.utc_ns % u1e9) / 1e9);
    result.push_back(static_cast<double>(navstate.raw_sn));
    result.push_back(navstate.euler[0] * R2D);
    result.push_back(navstate.euler[1] * R2D);
    result.push_back(navstate.euler[2] * R2D);
    result.push_back(navstate.vel[0]);
    result.push_back(navstate.vel[1]);
    result.push_back(navstate.vel[2]);
    result.push_back(navstate.pos[0] * R2D);
    result.push_back(navstate.pos[1] * R2D);
    result.push_back(navstate.pos[2]);
    result.push_back(navstate.imuerror.gyrbias[0] * RS2DH);
    result.push_back(navstate.imuerror.gyrbias[1] * RS2DH);
    result.push_back(navstate.imuerror.gyrbias[2] * RS2DH);
    result.push_back(navstate.imuerror.accbias[0] / 9.79e-6);
    result.push_back(navstate.imuerror.accbias[1] / 9.79e-6);
    result.push_back(navstate.imuerror.accbias[2] / 9.79e-6);

    result.push_back(gnss.blh[0] * R2D);
    result.push_back(gnss.blh[1] * R2D);
    result.push_back(gnss.blh[2]);
    result.push_back(gnss.vel[0]);
    result.push_back(gnss.vel[1]);
    result.push_back(gnss.vel[2]);

    auto cov_diag = cov.diagonal();
    result.push_back(sqrt(cov_diag[InsPvStekf15states::StateID::PHI_ID + 0]) * R2D);
    result.push_back(sqrt(cov_diag[InsPvStekf15states::StateID::PHI_ID + 1]) * R2D);
    result.push_back(sqrt(cov_diag[InsPvStekf15states::StateID::PHI_ID + 2]) * R2D);
    result.push_back(sqrt(cov_diag[InsPvStekf15states::StateID::V_ID + 0]));
    result.push_back(sqrt(cov_diag[InsPvStekf15states::StateID::V_ID + 1]));
    result.push_back(sqrt(cov_diag[InsPvStekf15states::StateID::V_ID + 2]));
    result.push_back(sqrt(cov_diag[InsPvStekf15states::StateID::P_ID + 0]) * R2D);
    result.push_back(sqrt(cov_diag[InsPvStekf15states::StateID::P_ID + 1]) * R2D);
    result.push_back(sqrt(cov_diag[InsPvStekf15states::StateID::P_ID + 2]));
    result.push_back(sqrt(cov_diag[InsPvStekf15states::StateID::BG_ID + 0]) * RS2DH);
    result.push_back(sqrt(cov_diag[InsPvStekf15states::StateID::BG_ID + 1]) * RS2DH);
    result.push_back(sqrt(cov_diag[InsPvStekf15states::StateID::BG_ID + 2]) * RS2DH);
    result.push_back(sqrt(cov_diag[InsPvStekf15states::StateID::BA_ID + 0]) / 9.79e-6);
    result.push_back(sqrt(cov_diag[InsPvStekf15states::StateID::BA_ID + 1]) / 9.79e-6);
    result.push_back(sqrt(cov_diag[InsPvStekf15states::StateID::BA_ID + 2]) / 9.79e-6);
    result.push_back(omega_rms);

    fs.set_columns(result.size());
    fs.dump(result);
}

void KfNav::sendNavState(const NavState &nav_state)
{
    pvat_t data;
    auto quat = Rotation::euler2quaternion(nav_state.euler);
    data.q0 = quat.w();
    data.q1 = quat.x();
    data.q2 = quat.y();
    data.q3 = quat.z();
    data.roll = nav_state.euler[0];
    data.pitch = nav_state.euler[1];
    data.yaw = nav_state.euler[2];
    data.vel_n = nav_state.vel[0];
    data.vel_e = nav_state.vel[1];
    data.vel_d = nav_state.vel[2];
    data.latitude = nav_state.pos[0] * R2D;
    data.longitude = nav_state.pos[1] * R2D;
    data.altitude = nav_state.pos[2];
    data.pos_x = 0;
    data.pos_y = 0;
    data.pos_z = 0;

    uint64_t uinx_timestamp = nav_state.utc_ns / u1e9;
    std::tm *now_tm = std::localtime((std::time_t *)&uinx_timestamp);

    data.year = now_tm->tm_year + 1900;
    data.month = now_tm->tm_mon + 1;
    data.day = now_tm->tm_mday;

    auto sec = (now_tm->tm_hour * 60 + now_tm->tm_min) * 60 + now_tm->tm_sec;
    data.second = sec + (nav_state.utc_ns % u1e9) / 1e9;
    context.send_struct(sizeof(data), (char *)(&data));
}
