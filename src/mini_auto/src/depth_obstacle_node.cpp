/*
深度画像 → 障害物スキャン (D456 用)

深度画像の画素を間引いて3次元点に戻し、base_link 座標で
min_height <= z <= max_height の点だけを残して、水平方向の角度ごとの最近距離を
LaserScan (frame = base_link) にして出す。床 (z ≒ 0) と機体より高い物は捨てる。
TG30 の高さに映らない低い物(床の物・段差の縁・椅子の脚の下側)や、机の天板を拾うためのもの。
SLAM には入れず、simple_nav の obstacle_topics にだけ渡す。

深度は 16UC1 (depth_scale 倍で [m], RealSense は 0.001) と 32FC1 ([m]) に対応。
カメラの姿勢は TF base_link -> 画像の frame_id から取る(取り付け角の誤差は床の誤検出に直結する)。
*/

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class DepthObstacle : public rclcpp::Node {
public:
    DepthObstacle() : Node("depth_obstacle") {
        DeclareParameters();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, rclcpp::SensorDataQoS());
        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { info_ = msg; });
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, rclcpp::SensorDataQoS(),
            std::bind(&DepthObstacle::DepthCallback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "depth obstacle: %s -> %s (height %.2f..%.2f m)",
                    depth_topic_.c_str(), output_topic_.c_str(), min_height_, max_height_);
    }

private:
    // ===== 初期化 =====
    void DeclareParameters() {
        depth_topic_ = declare_parameter<std::string>("depth_topic", "camera/camera/depth/image_rect_raw");
        info_topic_ = declare_parameter<std::string>("info_topic", "camera/camera/depth/camera_info");
        output_topic_ = declare_parameter<std::string>("output_topic", "depth_scan");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        depth_scale_ = declare_parameter<double>("depth_scale", 0.001);  // 16UC1 の 1 カウント [m]
        min_height_ = declare_parameter<double>("min_height", 0.05);     // 床より上 [m]
        max_height_ = declare_parameter<double>("max_height", 0.6);      // 機体の高さ [m]
        min_range_ = declare_parameter<double>("min_range", 0.3);        // [m]
        max_range_ = declare_parameter<double>("max_range", 3.0);        // [m] 遠いほど深度ノイズが大きい
        stride_ = declare_parameter<int>("stride", 4);                   // 画素の間引き
        bins_ = declare_parameter<int>("bins", 720);                     // 出力スキャンの分割数(全周)
        min_points_ = declare_parameter<int>("min_points_per_bin", 3);   // ノイズ除去
    }

    // ===== 変換 =====
    void DepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (!info_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for camera_info");
            return;
        }
        const bool is_u16 = msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                            msg->encoding == sensor_msgs::image_encodings::MONO16;
        const bool is_f32 = msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1;
        if (!is_u16 && !is_f32) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "unsupported encoding %s",
                                 msg->encoding.c_str());
            return;
        }

        tf2::Transform base_from_cam;
        try {
            const auto tf = tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id,
                                                        tf2::TimePointZero);
            tf2::fromMsg(tf.transform, base_from_cam);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "camera TF: %s", ex.what());
            return;
        }

        const double fx = info_->k[0];
        const double fy = info_->k[4];
        const double cx = info_->k[2];
        const double cy = info_->k[5];
        if (fx <= 0.0 || fy <= 0.0) {
            return;
        }

        const double angle_increment = 2.0 * M_PI / bins_;
        std::vector<float> nearest(bins_, std::numeric_limits<float>::infinity());
        std::vector<int> count(bins_, 0);

        for (uint32_t v = 0; v < msg->height; v += stride_) {
            const uint8_t *row = msg->data.data() + static_cast<size_t>(v) * msg->step;
            for (uint32_t u = 0; u < msg->width; u += stride_) {
                double d;
                if (is_u16) {
                    uint16_t raw;
                    std::memcpy(&raw, row + u * 2, 2);
                    d = raw * depth_scale_;
                } else {
                    float raw;
                    std::memcpy(&raw, row + u * 4, 4);
                    d = raw;
                }
                if (!std::isfinite(d) || d <= 0.0 || d > max_range_) {
                    continue;
                }
                const tf2::Vector3 p =
                    base_from_cam * tf2::Vector3((u - cx) / fx * d, (v - cy) / fy * d, d);
                if (p.z() < min_height_ || p.z() > max_height_) {
                    continue;
                }
                const double r = std::hypot(p.x(), p.y());
                if (r < min_range_) {
                    continue;
                }
                int bin = static_cast<int>(std::floor((std::atan2(p.y(), p.x()) + M_PI) / angle_increment));
                bin = std::min(std::max(bin, 0), bins_ - 1);
                nearest[bin] = std::min(nearest[bin], static_cast<float>(r));
                ++count[bin];
            }
        }

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = msg->header.stamp;
        scan.header.frame_id = base_frame_;
        scan.angle_min = -M_PI + 0.5 * angle_increment;  // ビンの中心
        scan.angle_increment = angle_increment;
        scan.angle_max = scan.angle_min + angle_increment * (bins_ - 1);
        scan.range_min = min_range_;
        scan.range_max = max_range_;
        scan.ranges.resize(bins_);
        for (int i = 0; i < bins_; ++i) {
            scan.ranges[i] = (count[i] >= min_points_) ? nearest[i]
                                                       : std::numeric_limits<float>::infinity();
        }
        scan_pub_->publish(scan);
    }

    // ===== パラメータ =====
    std::string depth_topic_, info_topic_, output_topic_, base_frame_;
    double depth_scale_, min_height_, max_height_, min_range_, max_range_;
    int stride_, bins_, min_points_;

    // ===== 状態 =====
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;

    // ===== ROS =====
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthObstacle>());
    rclcpp::shutdown();
    return 0;
}
