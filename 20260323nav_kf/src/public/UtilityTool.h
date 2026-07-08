#ifndef UTILITY_TOOL_H
#define UTILITY_TOOL_H


#include <thread>
#include <memory>
#include <sstream>
#include <boost/date_time.hpp>
#include <boost/format.hpp>
#include <boost/asio.hpp>
#include <ros/ros.h>


#define LOG_TAG "KfNav"


#define CREATE_LOG_MSG(MSG) std::stringstream abc123;abc123<<"["<<std::this_thread::get_id()<<"]["<<__FUNCTION__<<"@"<<__LINE__<<"]"<<MSG
#define LOG_INFO(MSG) {CREATE_LOG_MSG(MSG);ROS_INFO_STREAM(abc123.str());}
#define LOG_ERROR(MSG) {CREATE_LOG_MSG(MSG);ROS_ERROR_STREAM(abc123.str());}
#define LOG_WARN(MSG) {CREATE_LOG_MSG(MSG);ROS_WARN_STREAM(abc123.str());}
#define LOG_DEBUG(MSG) {CREATE_LOG_MSG(MSG);ROS_DEBUG_STREAM(abc123.str());}

#define JOY_STATUS_ABS 0x00
#define JOY_STATUS_KEY_A 0x130
#define JOY_STATUS_KEY_B 0x131
#define JOY_STATUS_KEY_X 0x133
#define JOY_STATUS_KEY_Y 0x134
#define JOY_STATUS_KEY_LB 0x136
#define JOY_STATUS_KEY_RB 0x137

#define JOY_STATUS_CHARGE 0x5001

using FramePtr = std::shared_ptr<std::vector<unsigned char>>;
using SerialPtr = std::shared_ptr<boost::asio::serial_port>;

#endif // UTILITY_TOOL_H
