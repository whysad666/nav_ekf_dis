#include <param_common.h>

std::string ParamCommon::getString(const YAML::Node &node, const std::string fallback) {
    return node.as<std::string>(fallback);
}

uint64_t ParamCommon::getUint64(const YAML::Node &node, const uint64_t fallback) {
    return node.as<uint64_t>(fallback);
}

double ParamCommon::getDouble(const YAML::Node &node, const double fallback) {
    return node.as<double>(fallback);
}

Eigen::Vector3d ParamCommon::getVector3d(const YAML::Node &node, const Eigen::Vector3d fallback) {
    Eigen::Vector3d ret;
    std::vector<double> vec = node.as<std::vector<double>>(eigenVec2stdVec(fallback));
    assert(vec.size() == 3);
    ret << vec[0], vec[1], vec[2];
    return ret;
}

Eigen::Matrix3d ParamCommon::getMatrix3d(const YAML::Node &node, const Eigen::Matrix3d fallback) {
    Eigen::Matrix3d ret;
    ret << getVector3d(node["r1"], fallback.block<1,3>(0, 0)).transpose(),
        getVector3d(node["r2"], fallback.block<1,3>(1, 0)).transpose(),
        getVector3d(node["r3"], fallback.block<1,3>(2, 0)).transpose();
    return ret;
}

std::vector<double> ParamCommon::eigenVec2stdVec(const Eigen::Vector3d &vec) {
    std::vector<double> ret;
    ret.push_back(vec[0]);
    ret.push_back(vec[1]);
    ret.push_back(vec[2]);
    return ret;
}
