/*
簡易シミュレータ (実機なしで全体を通すためのもの)

ros2can の代わりに振る舞う:
  serial_tx_[ID] (omni_drive の duty) → 4輪の転がり速度 → 最小二乗で機体速度 → 真値を積分
  真値の動き → 3計測輪の移動距離(+すべりノイズ) → serial_rx_[ID]_zeroed のカウント
さらに壁(線分の集合)に対してレイキャストした /scan と、
ドリフトする yaw を持つ /imu (WT901C + wit_node の代わり) を出す。
D456 の代わりに深度画像 (16UC1, mm) と camera_info も出す。カメラの取り付けは
TF base_link -> depth_frame を読む(launch の static TF と食い違わないように)。
low_obstacles の箱は LiDAR より低い想定で、深度カメラにだけ映る。

sim/ground_truth には「開始姿勢を原点とした」真値を出す。slam_ros2 の map も
開始姿勢が原点なので、RViz で SLAM の推定と直接見比べられる。
*/

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mini_auto/kinematics.hpp"
#include "mini_auto/params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace {

// 床から height まで立っている壁
struct Segment {
    double x1, y1, x2, y2;
    double height;
};

// (px, py) + t (dx, dy) が線分に当たる t。当たらなければ +inf。
// (dx, dy) が単位ベクトルなら t は距離になる
double RayParamToSegment(double px, double py, double dx, double dy, const Segment &s) {
    const double ex = s.x2 - s.x1;
    const double ey = s.y2 - s.y1;
    const double denom = dx * ey - dy * ex;
    if (std::fabs(denom) < 1e-12) {
        return std::numeric_limits<double>::infinity();
    }
    const double wx = s.x1 - px;
    const double wy = s.y1 - py;
    const double t = (wx * ey - wy * ex) / denom;  // レイ上の距離
    const double u = (wx * dy - wy * dx) / denom;  // 線分上の位置 0..1
    if (t > 0.0 && u >= 0.0 && u <= 1.0) {
        return t;
    }
    return std::numeric_limits<double>::infinity();
}

double RayToSegment(double px, double py, double a, const Segment &s) {
    return RayParamToSegment(px, py, std::cos(a), std::sin(a), s);
}

}  // namespace

class SimRobot : public rclcpp::Node {
public:
    SimRobot() : Node("sim_robot"), rng_(std::random_device{}()) {
        DeclareParameters();

        for (int id : drive_device_ids_) {
            if (tx_subs_.count(id) == 0) {
                tx_subs_[id] = create_subscription<std_msgs::msg::Int16MultiArray>(
                    "serial_tx_" + std::to_string(id), 10,
                    [this, id](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
                        TxCallback(id, *msg);
                    });
            }
        }
        for (int id : tracking_device_ids_) {
            if (rx_pubs_.count(id) == 0) {
                rx_pubs_[id] = create_publisher<std_msgs::msg::Int32MultiArray>(
                    "serial_rx_" + std::to_string(id) + "_zeroed", 10);
            }
        }
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
        truth_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("sim/ground_truth", 10);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
        depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "camera/camera/depth/image_rect_raw", rclcpp::SensorDataQoS());
        info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera/camera/depth/camera_info", rclcpp::SensorDataQoS());
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sim_timer_ = create_wall_timer(std::chrono::milliseconds(sim_period_ms_),
                                       std::bind(&SimRobot::Step, this));
        scan_timer_ = create_wall_timer(std::chrono::milliseconds(scan_period_ms_),
                                        std::bind(&SimRobot::PublishScan, this));
        imu_timer_ = create_wall_timer(std::chrono::milliseconds(imu_period_ms_),
                                       std::bind(&SimRobot::PublishImu, this));
        depth_timer_ = create_wall_timer(std::chrono::milliseconds(depth_period_ms_),
                                         std::bind(&SimRobot::PublishDepth, this));
        RCLCPP_INFO(get_logger(), "sim robot started at (%.2f, %.2f) with %zu walls, %zu low boxes",
                    pose_.x, pose_.y, walls_.size(), low_segments_.size() / 4);
    }

private:
    // ===== 初期化 =====
    void DeclareParameters() {
        // 足回り(omni_drive と同じ値を robot.yaml の /** から読む)
        drive_wheels_ = mini_auto::DeclareWheelMounts(*this, "drive_wheel", 4);
        drive_device_ids_ = mini_auto::DeclareIntArray(*this, "drive_device_ids", 4);
        drive_tx_slots_ = mini_auto::DeclareIntArray(*this, "drive_tx_slots", 4);
        drive_signs_ = mini_auto::DeclareIntArray(*this, "drive_signs", 4);
        max_wheel_speed_ = declare_parameter<double>("max_wheel_speed", 1.5);
        duty_max_ = declare_parameter<double>("duty_max", 255.0);
        drive_duty_.assign(4, 0.0);

        // 計測輪(tracking_odom と同じ値)
        tracking_wheels_ = mini_auto::DeclareWheelMounts(*this, "tracking_wheel", 3);
        tracking_device_ids_ = mini_auto::DeclareIntArray(*this, "tracking_device_ids", 3);
        tracking_rx_slots_ = mini_auto::DeclareIntArray(*this, "tracking_rx_slots", 3);
        tracking_signs_ = mini_auto::DeclareIntArray(*this, "tracking_signs", 3);
        tracking_radius_ = declare_parameter<double>("tracking_wheel_radius", 0.024);
        tracking_cpr_ = declare_parameter<double>("tracking_counts_per_rev", 2048.0);
        tracking_dist_.assign(3, 0.0);

        // シミュレーション
        rx_slot_num_ = declare_parameter<int>("rx_slot_num", 24);
        sim_period_ms_ = declare_parameter<int>("sim_period_ms", 10);
        scan_period_ms_ = declare_parameter<int>("scan_period_ms", 100);
        // 実際に出る速度 / 指令速度 (オープンループの誤差を模擬)
        drive_speed_gain_ = declare_parameter<double>("drive_speed_gain", 0.9);
        // 計測輪の移動距離にかける誤差の標準偏差(比率)
        tracking_slip_ = declare_parameter<double>("tracking_slip_stddev", 0.02);
        pose_.x = declare_parameter<double>("start_x", 0.0);
        pose_.y = declare_parameter<double>("start_y", 0.0);
        pose_.yaw = declare_parameter<double>("start_yaw_deg", 0.0) * M_PI / 180.0;
        start_ = pose_;

        // LiDAR (機体中心に取り付け。launch の base_link->laser_frame も 0 にしておく)
        scan_frame_ = declare_parameter<std::string>("scan_frame", "laser_frame");
        scan_beams_ = declare_parameter<int>("scan_beams", 720);
        scan_range_min_ = declare_parameter<double>("scan_range_min", 0.1);
        scan_range_max_ = declare_parameter<double>("scan_range_max", 12.0);
        scan_noise_ = declare_parameter<double>("scan_noise_stddev", 0.01);  // [m]

        // IMU (WT901C 相当: 本体で融合済みの姿勢角を出す。6軸モードなので yaw はドリフトする)
        imu_period_ms_ = declare_parameter<int>("imu_period_ms", 10);
        imu_drift_ = declare_parameter<double>("imu_yaw_drift_deg_per_min", 0.5) * M_PI / 180.0 / 60.0;
        imu_noise_ = declare_parameter<double>("imu_yaw_noise_deg", 0.05) * M_PI / 180.0;

        // 深度カメラ (D456 相当, 解像度は計算量のため低め)
        depth_frame_ = declare_parameter<std::string>("depth_frame", "camera_depth_optical_frame");
        depth_width_ = declare_parameter<int>("depth_width", 160);
        depth_height_ = declare_parameter<int>("depth_height", 100);
        depth_hfov_ = declare_parameter<double>("depth_hfov_deg", 87.0) * M_PI / 180.0;
        depth_period_ms_ = declare_parameter<int>("depth_period_ms", 100);
        depth_min_ = declare_parameter<double>("depth_min", 0.4);  // これより近いと測れない [m]
        depth_max_ = declare_parameter<double>("depth_max", 6.0);  // [m]

        // 壁: [x1, y1, x2, y2, ...] (ワールド座標, 高さ wall_height)
        const double wall_height = declare_parameter<double>("wall_height", 2.0);
        const auto flat = declare_parameter<std::vector<double>>("walls", std::vector<double>{});
        if (flat.size() % 4 != 0) {
            throw std::runtime_error("walls must be a multiple of 4 numbers");
        }
        for (size_t i = 0; i < flat.size(); i += 4) {
            walls_.push_back({flat[i], flat[i + 1], flat[i + 2], flat[i + 3], wall_height});
        }
        // LiDAR より低い箱: [xmin, ymin, xmax, ymax, height, ...]
        const auto boxes = declare_parameter<std::vector<double>>("low_obstacles", std::vector<double>{});
        if (boxes.size() % 5 != 0) {
            throw std::runtime_error("low_obstacles must be a multiple of 5 numbers");
        }
        for (size_t i = 0; i < boxes.size(); i += 5) {
            const double x0 = boxes[i], y0 = boxes[i + 1], x1 = boxes[i + 2], y1 = boxes[i + 3];
            const double h = boxes[i + 4];
            low_segments_.push_back({x0, y0, x1, y0, h});
            low_segments_.push_back({x1, y0, x1, y1, h});
            low_segments_.push_back({x1, y1, x0, y1, h});
            low_segments_.push_back({x0, y1, x0, y0, h});
        }
    }

    // ===== 受信 =====
    void TxCallback(int device_id, const std_msgs::msg::Int16MultiArray &msg) {
        for (size_t i = 0; i < drive_wheels_.size(); ++i) {
            if (drive_device_ids_[i] == device_id &&
                static_cast<size_t>(drive_tx_slots_[i]) < msg.data.size()) {
                drive_duty_[i] = msg.data[drive_tx_slots_[i]];
            }
        }
    }

    // ===== 物理 =====
    void Step() {
        const double dt = sim_period_ms_ / 1000.0;

        // duty → 転がり速度 (omni_drive の変換の逆) → 機体速度
        std::vector<double> speeds(drive_wheels_.size());
        for (size_t i = 0; i < speeds.size(); ++i) {
            speeds[i] = drive_signs_[i] * drive_duty_[i] / duty_max_ * max_wheel_speed_ *
                        drive_speed_gain_;
        }
        mini_auto::Twist2D twist;
        mini_auto::OmniBodyTwist(drive_wheels_, speeds, &twist);
        const mini_auto::Twist2D delta{twist.vx * dt, twist.vy * dt, twist.wz * dt};
        pose_ = mini_auto::Integrate(pose_, delta);
        imu_yaw_ = mini_auto::WrapPi(imu_yaw_ + delta.wz + imu_drift_ * dt);
        body_wz_ = twist.wz;

        // 計測輪の移動距離 → カウント (tracking_odom の変換の逆)
        std::normal_distribution<double> slip(1.0, tracking_slip_);
        std::map<int, std_msgs::msg::Int32MultiArray> rx;
        for (size_t i = 0; i < tracking_wheels_.size(); ++i) {
            tracking_dist_[i] += mini_auto::WheelSurfaceSpeed(tracking_wheels_[i], delta) * slip(rng_);
            auto &msg = rx[tracking_device_ids_[i]];
            msg.data.resize(rx_slot_num_, 0);
            msg.data[tracking_rx_slots_[i]] = static_cast<int32_t>(std::lround(
                tracking_signs_[i] * tracking_dist_[i] / (2.0 * M_PI * tracking_radius_) *
                tracking_cpr_));
        }
        for (auto &[id, msg] : rx) {
            rx_pubs_[id]->publish(msg);
        }
        PublishTruth();
    }

    // ===== センサ =====
    void PublishScan() {
        std::normal_distribution<double> noise(0.0, scan_noise_);
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = now();
        scan.header.frame_id = scan_frame_;
        scan.angle_min = -M_PI;
        scan.angle_increment = 2.0 * M_PI / scan_beams_;
        scan.angle_max = scan.angle_min + scan.angle_increment * (scan_beams_ - 1);
        scan.scan_time = scan_period_ms_ / 1000.0;
        scan.range_min = scan_range_min_;
        scan.range_max = scan_range_max_;
        scan.ranges.resize(scan_beams_);
        for (int i = 0; i < scan_beams_; ++i) {
            const double a = pose_.yaw + scan.angle_min + i * scan.angle_increment;
            double r = std::numeric_limits<double>::infinity();
            for (const auto &w : walls_) {
                r = std::min(r, RayToSegment(pose_.x, pose_.y, a, w));
            }
            scan.ranges[i] = (r <= scan_range_max_)
                                 ? static_cast<float>(r + noise(rng_))
                                 : std::numeric_limits<float>::infinity();
        }
        scan_pub_->publish(scan);
    }

    // ピンホールカメラで壁・低い箱・床にレイを飛ばす
    void PublishDepth() {
        tf2::Transform base_from_cam;
        try {
            const auto tf = tf_buffer_->lookupTransform("base_link", depth_frame_, tf2::TimePointZero);
            tf2::fromMsg(tf.transform, base_from_cam);
        } catch (const tf2::TransformException &) {
            return;  // static TF がまだ来ていない
        }
        tf2::Transform world_from_base;
        world_from_base.setOrigin(tf2::Vector3(pose_.x, pose_.y, 0.0));
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, pose_.yaw);
        world_from_base.setRotation(q);
        const tf2::Transform world_from_cam = world_from_base * base_from_cam;
        const tf2::Vector3 c = world_from_cam.getOrigin();
        const tf2::Matrix3x3 rot = world_from_cam.getBasis();

        const double fx = 0.5 * depth_width_ / std::tan(0.5 * depth_hfov_);
        const double fy = fx;
        const double cx = 0.5 * depth_width_;
        const double cy = 0.5 * depth_height_;

        sensor_msgs::msg::Image img;
        img.header.stamp = now();
        img.header.frame_id = depth_frame_;
        img.width = depth_width_;
        img.height = depth_height_;
        img.encoding = "16UC1";
        img.step = depth_width_ * 2;
        img.data.assign(img.step * img.height, 0);
        std::normal_distribution<double> noise(0.0, 0.005);

        for (int v = 0; v < depth_height_; ++v) {
            for (int u = 0; u < depth_width_; ++u) {
                // 光学座標の z=1 のレイ → t がそのまま深度になる
                const tf2::Vector3 d = rot * tf2::Vector3((u + 0.5 - cx) / fx, (v + 0.5 - cy) / fy, 1.0);
                double t = std::numeric_limits<double>::infinity();
                if (d.z() < -1e-9) {
                    t = -c.z() / d.z();  // 床
                }
                for (const auto *list : {&walls_, &low_segments_}) {
                    for (const auto &seg : *list) {
                        const double ts = RayParamToSegment(c.x(), c.y(), d.x(), d.y(), seg);
                        const double z = c.z() + d.z() * ts;
                        if (ts < t && z >= 0.0 && z <= seg.height) {
                            t = ts;
                        }
                    }
                }
                if (t < depth_min_ || t > depth_max_) {
                    continue;
                }
                const uint16_t mm = static_cast<uint16_t>(std::lround((t + noise(rng_)) * 1000.0));
                std::memcpy(&img.data[v * img.step + u * 2], &mm, 2);
            }
        }

        sensor_msgs::msg::CameraInfo info;
        info.header = img.header;
        info.width = depth_width_;
        info.height = depth_height_;
        info.distortion_model = "plumb_bob";
        info.d.assign(5, 0.0);
        info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
        info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
        depth_pub_->publish(img);
        info_pub_->publish(info);
    }

    void PublishImu() {
        std::normal_distribution<double> noise(0.0, imu_noise_);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, mini_auto::WrapPi(imu_yaw_ + noise(rng_)));
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = now();
        msg.header.frame_id = "imu_link";
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();
        msg.angular_velocity.z = body_wz_;
        msg.linear_acceleration.z = 9.8;
        imu_pub_->publish(msg);
    }

    // 開始姿勢を原点とした真値
    void PublishTruth() {
        const double dx = pose_.x - start_.x;
        const double dy = pose_.y - start_.y;
        const double c = std::cos(start_.yaw);
        const double s = std::sin(start_.yaw);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, mini_auto::WrapPi(pose_.yaw - start_.yaw));

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.pose.position.x = c * dx + s * dy;
        msg.pose.position.y = -s * dx + c * dy;
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        truth_pub_->publish(msg);
    }

    // ===== パラメータ =====
    std::vector<mini_auto::WheelMount> drive_wheels_, tracking_wheels_;
    std::vector<int> drive_device_ids_, drive_tx_slots_, drive_signs_;
    std::vector<int> tracking_device_ids_, tracking_rx_slots_, tracking_signs_;
    double max_wheel_speed_, duty_max_, tracking_radius_, tracking_cpr_;
    int rx_slot_num_, sim_period_ms_, scan_period_ms_, scan_beams_;
    double drive_speed_gain_, tracking_slip_;
    int imu_period_ms_;
    double imu_drift_, imu_noise_;
    double scan_range_min_, scan_range_max_, scan_noise_;
    std::string scan_frame_;
    std::vector<Segment> walls_;
    std::vector<Segment> low_segments_;
    std::string depth_frame_;
    int depth_width_, depth_height_, depth_period_ms_;
    double depth_hfov_, depth_min_, depth_max_;

    // ===== 状態 =====
    std::mt19937 rng_;
    mini_auto::Pose2D pose_, start_;
    std::vector<double> drive_duty_;
    std::vector<double> tracking_dist_;
    double imu_yaw_ = 0.0;  // 電源投入時の向きが 0 (WT901C の 6軸モードと同じ)
    double body_wz_ = 0.0;

    // ===== ROS =====
    std::map<int, rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr> tx_subs_;
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr> rx_pubs_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr truth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr sim_timer_, scan_timer_, imu_timer_, depth_timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimRobot>());
    rclcpp::shutdown();
    return 0;
}
