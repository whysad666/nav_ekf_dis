#include <param_loader.h>
std::shared_ptr<ParamLoader> pload;

bool ParamLoader::load(const std::string &filename) {

    std::cout << " -- param_loader file: " << filename << std::endl;
    YAML::Node config = YAML::LoadFile(filename);

    // IMU freq
    imudatarate = config["imu_freq"].as<uint64_t>(200);
    std::cout << " -- imudatarate       :" << imudatarate << std::endl;

    // Nav state pub freq
    nav_pub_rate = config["nav_pub_rate"].as<uint64_t>(1);
    std::cout << " -- nav_pub_rate       :" << nav_pub_rate << std::endl;

        // Nav state print freq
        precisionEchoEpoch = config["precisionEchoEpoch"].as<uint64_t>(1);
        std::cout << " -- precisionEchoEpoch       :" << precisionEchoEpoch << std::endl;

    // ca timelength
    ca_timelen = config["ca_timelength"].as<uint64_t>(60);
    std::cout << " -- CA timelength     :" << ca_timelen << std::endl;
    // GNSS-INS integral time
    gnss_timelen = config["gnss_timelength"].as<double>(INFINITY);
    std::cout << " -- gnss-int timelength:" << gnss_timelen << std::endl;
    // Navigation Calc time
    calc_timelen = config["calc_timelength"].as<double>(INFINITY);
    std::cout << " -- nav calc timelength:" << calc_timelen << std::endl;

    // 零速观测
    // static threshold
    // ZUPT timelength
    zupt_timelength = config["zupt_timelength"].as<uint64_t>(600);
    std::cout << " -- zupt_timelen        :" << zupt_timelength << std::endl;
    // static velocity threshold
    omega_threshold = config["omega_threshold"].as<double>(0);
    std::cout << " -- omega_threshold     :" << omega_threshold << "[deg/s]" << std::endl;
    // static velocity noise
    velstd = ParamCommon::getVector3d(config["velstd"], Eigen::Vector3d(0.1, 0.1, 0.1));
    std::cout << " -- velstd              :" << velstd.transpose() << std::endl;
    //range ID 1-3
    range_id_1_3 = ParamCommon::getVector3d(config["range_id_1_3"], Eigen::Vector3d(1,2,3));
    std::cout << " -- range_id_1_3        :" << range_id_1_3.transpose() << std::endl;
    //range ID 4-6
    range_id_4_6 = ParamCommon::getVector3d(config["range_id_4_6"], Eigen::Vector3d(4,5,6));
    std::cout << " -- range_id_4_6        :" << range_id_4_6.transpose() << std::endl;
    //range ID 7-9
    range_id_7_9 = ParamCommon::getVector3d(config["range_id_7_9"], Eigen::Vector3d(7,8,9));
    std::cout << " -- range_id_7_9        :" << range_id_7_9.transpose() << std::endl;
    //range ID 10-12
    range_id_10_12 = ParamCommon::getVector3d(config["range_id_10_12"], Eigen::Vector3d(10,11,12));
    std::cout << " -- range_id_10_12        :" << range_id_10_12.transpose() << std::endl;
    //anchor_id_1_3
    anchor_id_1_3 = ParamCommon::getVector3d(config["anchor_id_1_3"], Eigen::Vector3d(1,2,3));
    std::cout << " -- anchor_id_1_3        :" << anchor_id_1_3.transpose() << std::endl;


    //baro noise
    baro_std = config["baro_std"].as<double>(0.5);
    std::cout << " -- baro_std              :" << baro_std << std::endl;
    //is_baro_using
    is_baro_using = config["is_baro_using"].as<bool>(false);
    std::cout << " -- is_baro_using              :" << is_baro_using << std::endl;
    //is_baro_using_kf
    is_baro_using_kf = config["is_baro_using_kf"].as<bool>(false);
    std::cout << " -- is_baro_using_kf              :" << is_baro_using_kf << std::endl;
    //range noise,Vector3d
    range_std = ParamCommon::getVector3d(config["range_std"], Eigen::Vector3d(0.5,0.5,0.5));
    std::cout << " -- range_std              :" << range_std.transpose() << std::endl;
    //gnss_pos noise
    gnss_pos_std = ParamCommon::getVector3d(config["gnss_pos_std"], Eigen::Vector3d(2,2,5));
    std::cout << " -- gnss_pos_std        :" << gnss_pos_std.transpose() << std::endl;

    //is_range_using
    is_range_using = config["is_range_using"].as<bool>(false);
    std::cout << " -- is_range_using              :" << is_range_using << std::endl;

    //is_mag_yaw_using
    is_mag_yaw_using = config["is_mag_yaw_using"].as<bool>(false);
    std::cout << " -- is_mag_yaw_using              :" << is_mag_yaw_using << std::endl;


    // 输入输出话题
    // topics
    imu_topic = config["topic"]["imu"].as<std::string>("/imu/data");
    std::cout << " -- imu_topic         :" << imu_topic << std::endl;
    gnss_origin_topic = config["topic"]["gnss_origin"].as<std::string>("/gps/fix");
    std::cout << " -- gnss_origin_topic        :" << gnss_origin_topic << std::endl;
    gnss_pv_topic = config["topic"]["gnss_pv"].as<std::string>("unuse");
    std::cout << " -- gnss_pv_topic        :" << gnss_pv_topic << std::endl;
    mag_topic = config["topic"]["mag"].as<std::string>("/mag_origin");
    std::cout << " -- mag_topic        :" << mag_topic << std::endl;
    //baro
    baro_topic = config["topic"]["baro"].as<std::string>("/baro/data");
    std::cout << " -- baro_topic        :" << baro_topic << std::endl;
    //range
    range_topic = config["topic"]["range"].as<std::string>("/range/data");
    std::cout << " -- range_topic        :" << range_topic << std::endl;

    // output
    gi_topic = config["topic"]["gi_nav"].as<std::string>("/kfnav_gi/output");
    std::cout << " -- gi_topic        :" << gi_topic << std::endl;

    return true;
}
