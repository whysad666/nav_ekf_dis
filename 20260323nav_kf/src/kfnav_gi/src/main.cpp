#include <iostream>
#include <ros/ros.h>
#include <kfnav.h>
#include <param_loader.h>

auto main(int argc, char **argv) -> int {
    pload = std::make_shared<ParamLoader>();
    ros::init(argc, argv, "kfnav_gi");

    KfNav node;

    ros::spin();

    return 0;
}
