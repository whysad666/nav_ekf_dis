#pragma once

// #include "kf_gins_types.h"
#include <Eigen/Dense>
#include <common/angle.h>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace ParamCommon {

std::string getString(const YAML::Node &node, const std::string fallback);
uint64_t getUint64(const YAML::Node &node, const uint64_t fallback);
double getDouble(const YAML::Node &node, const double fallback);
Eigen::Vector3d getVector3d(const YAML::Node &node, const Eigen::Vector3d fallback);
Eigen::Matrix3d getMatrix3d(const YAML::Node &node, const Eigen::Matrix3d fallback);

std::vector<double> eigenVec2stdVec(const Eigen::Vector3d &vec);
}// namespace ParamCommon
