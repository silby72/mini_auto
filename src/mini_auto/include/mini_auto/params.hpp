/*
複数ノードで共通の車輪パラメータ読み込み

車輪は "<prefix>_x", "<prefix>_y", "<prefix>_dir_deg" の3配列(同じ長さ)で与える。
config/robot.yaml の全ノード共通の節に書いて、omni_drive / tracking_odom / sim_robot で共有する。
*/

#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "mini_auto/kinematics.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mini_auto {

inline std::vector<WheelMount> DeclareWheelMounts(rclcpp::Node &node, const std::string &prefix,
                                                  size_t expected) {
    const auto xs = node.declare_parameter<std::vector<double>>(prefix + "_x", std::vector<double>{});
    const auto ys = node.declare_parameter<std::vector<double>>(prefix + "_y", std::vector<double>{});
    const auto dirs =
        node.declare_parameter<std::vector<double>>(prefix + "_dir_deg", std::vector<double>{});
    if (xs.size() != expected || ys.size() != expected || dirs.size() != expected) {
        throw std::runtime_error(prefix + "_x/_y/_dir_deg must each have " +
                                 std::to_string(expected) + " elements");
    }
    std::vector<WheelMount> mounts(expected);
    for (size_t i = 0; i < expected; ++i) {
        mounts[i] = {xs[i], ys[i], dirs[i] * M_PI / 180.0};
    }
    return mounts;
}

// 長さを検査しつつ int 配列を宣言する
inline std::vector<int> DeclareIntArray(rclcpp::Node &node, const std::string &name, size_t expected) {
    const auto values = node.declare_parameter<std::vector<int64_t>>(name, std::vector<int64_t>{});
    if (values.size() != expected) {
        throw std::runtime_error(name + " must have " + std::to_string(expected) + " elements");
    }
    return std::vector<int>(values.begin(), values.end());
}

}  // namespace mini_auto
