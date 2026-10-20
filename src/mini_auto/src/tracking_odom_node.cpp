/*
3輪計測輪(トラッキングホイール)オドメトリノード

ros2can の serial_rx_[ID]_zeroed (原点セットからの相対カウント, int16 ラップ展開済み) から
3輪の移動距離を求め、3x3 ヤコビアンの逆行列で機体座標の (dx, dy, dθ) に変換して積分する。
出力: nav_msgs/Odometry (odom) と TF odom -> base_link。
slam_ros2 の slam_node はこの TF を ICP の初期値として使う (use_odom_tf: true)。

3輪は別々のデバイスに繋がっていてもよい。最新カウントをタイマ周期でまとめて使う。

use_imu: true のとき、回転量 dθ は IMU (WT901C, wit_node の /imu) の yaw の差分を使い、
並進 (dx, dy) は各輪の読みから回転ぶんを引いて最小二乗で解く。計測輪の yaw は取り付け誤差や
すべりに弱いので、ジャイロ由来の yaw の方が安定する。IMU が imu_timeout 途切れたら
計測輪だけの計算に自動で戻り、復帰したら yaw の基準を取り直す。
IMU は z 軸が機体の上下と平行(水平置き)である前提。逆さ付けなら imu_yaw_sign: -1。
*/

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mini_auto/kinematics.hpp"
#include "mini_auto/params.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace {
constexpr size_t kWheelNum = 3;
constexpr double kTwistFilterGain = 0.3;  // 速度LPFの新しい値の重み
}  // namespace

class TrackingOdom : public rclcpp::Node {
public:
    TrackingOdom() : Node("tracking_odom") {
        DeclareParameters();

        std::array<mini_auto::WheelMount, kWheelNum> mounts;
        std::copy(wheels_.begin(), wheels_.end(), mounts.begin());
        if (!odometry_.Configure(mounts)) {
            throw std::runtime_error(
                "tracking wheel layout is degenerate (vx, vy, wz cannot be recovered)");
        }

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        for (int id : device_ids_) {
            if (enc_subs_.count(id) == 0) {
                enc_subs_[id] = create_subscription<std_msgs::msg::Int32MultiArray>(
                    "serial_rx_" + std::to_string(id) + "_zeroed", 10,
                    [this, id](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
                        EncCallback(id, *msg);
                    });
            }
        }

        if (use_imu_) {
            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic_, rclcpp::SensorDataQoS(),
                std::bind(&TrackingOdom::ImuCallback, this, std::placeholders::_1));
        }

        timer_ = create_wall_timer(std::chrono::milliseconds(period_ms_),
                                   std::bind(&TrackingOdom::Update, this));
        RCLCPP_INFO(get_logger(), "tracking odom started (publish_tf=%s, yaw from %s)",
                    publish_tf_ ? "true" : "false", use_imu_ ? imu_topic_.c_str() : "wheels");
    }

private:
    // ===== 初期化 =====
    void DeclareParameters() {
        wheels_ = mini_auto::DeclareWheelMounts(*this, "tracking_wheel", kWheelNum);
        device_ids_ = mini_auto::DeclareIntArray(*this, "tracking_device_ids", kWheelNum);
        rx_slots_ = mini_auto::DeclareIntArray(*this, "tracking_rx_slots", kWheelNum);
        signs_ = mini_auto::DeclareIntArray(*this, "tracking_signs", kWheelNum);
        wheel_radius_ = declare_parameter<double>("tracking_wheel_radius", 0.024);  // [m]
        counts_per_rev_ = declare_parameter<double>("tracking_counts_per_rev", 2048.0);
        // 1周期でこれ以上動いたらカウントの飛び(原点の再セット等)とみなして捨てる [m]
        max_step_ = declare_parameter<double>("max_wheel_step", 0.1);

        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        publish_tf_ = declare_parameter<bool>("publish_tf", true);
        period_ms_ = declare_parameter<int>("period_ms", 10);

        use_imu_ = declare_parameter<bool>("use_imu", false);
        imu_topic_ = declare_parameter<std::string>("imu_topic", "imu");
        imu_yaw_sign_ = declare_parameter<int>("imu_yaw_sign", 1);
        imu_timeout_ = declare_parameter<double>("imu_timeout", 0.5);  // [s]
    }

    // ===== 受信 =====
    void EncCallback(int device_id, const std_msgs::msg::Int32MultiArray &msg) {
        for (size_t i = 0; i < kWheelNum; ++i) {
            if (device_ids_[i] != device_id) {
                continue;
            }
            if (rx_slots_[i] < 0 || static_cast<size_t>(rx_slots_[i]) >= msg.data.size()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "serial_rx_%d_zeroed too short (%zu)", device_id,
                                     msg.data.size());
                continue;
            }
            counts_[i] = msg.data[rx_slots_[i]];
            received_[i] = true;
        }
    }

    void ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        imu_yaw_ = imu_yaw_sign_ * tf2::getYaw(msg->orientation);
        last_imu_time_ = now();
        has_imu_ = true;
    }

    // IMU が使えるなら前回からの yaw 変化量を返す。使えなければ false
    bool ImuYawDelta(const rclcpp::Time &t, double *dtheta) {
        const bool fresh = has_imu_ && (t - last_imu_time_).seconds() <= imu_timeout_;
        if (!fresh) {
            if (imu_active_) {
                RCLCPP_WARN(get_logger(), "IMU lost; falling back to wheel yaw");
            }
            imu_active_ = false;
            return false;
        }
        if (!imu_active_) {
            // 初回 or 復帰: 基準を取り直すだけで、この周期は計測輪の yaw を使う
            RCLCPP_INFO(get_logger(), "IMU yaw in use");
            imu_active_ = true;
            last_imu_yaw_ = imu_yaw_;
            return false;
        }
        *dtheta = mini_auto::WrapPi(imu_yaw_ - last_imu_yaw_);
        last_imu_yaw_ = imu_yaw_;
        return true;
    }

    // ===== 積分 =====
    void Update() {
        const rclcpp::Time t = now();
        if (!std::all_of(received_.begin(), received_.end(), [](bool b) { return b; })) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "waiting for all tracking wheel encoders");
            return;
        }
        if (!initialized_) {
            last_counts_ = counts_;
            last_time_ = t;
            initialized_ = true;
            return;
        }

        mini_auto::Row3 dist{};
        for (size_t i = 0; i < kWheelNum; ++i) {
            const double revs = static_cast<double>(counts_[i] - last_counts_[i]) / counts_per_rev_;
            dist[i] = signs_[i] * revs * 2.0 * M_PI * wheel_radius_;
        }
        last_counts_ = counts_;
        const double dt = (t - last_time_).seconds();
        last_time_ = t;

        for (double d : dist) {
            if (std::fabs(d) > max_step_) {
                RCLCPP_WARN(get_logger(), "encoder jump (%.3f m) ignored", d);
                return;
            }
        }

        double dtheta = 0.0;
        const mini_auto::Twist2D delta = (use_imu_ && ImuYawDelta(t, &dtheta))
                                             ? odometry_.BodyDeltaWithYaw(dist, dtheta)
                                             : odometry_.BodyDelta(dist);
        pose_ = mini_auto::Integrate(pose_, delta);
        if (dt > 0.0) {
            twist_.vx += kTwistFilterGain * (delta.vx / dt - twist_.vx);
            twist_.vy += kTwistFilterGain * (delta.vy / dt - twist_.vy);
            twist_.wz += kTwistFilterGain * (delta.wz / dt - twist_.wz);
        }
        Publish(t);
    }

    // ===== 送信 =====
    void Publish(const rclcpp::Time &stamp) {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, pose_.yaw);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = pose_.x;
        odom.pose.pose.position.y = pose_.y;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x = twist_.vx;
        odom.twist.twist.linear.y = twist_.vy;
        odom.twist.twist.angular.z = twist_.wz;
        odom_pub_->publish(odom);

        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header = odom.header;
            tf.child_frame_id = base_frame_;
            tf.transform.translation.x = pose_.x;
            tf.transform.translation.y = pose_.y;
            tf.transform.rotation = odom.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf);
        }
    }

    // ===== パラメータ =====
    std::vector<mini_auto::WheelMount> wheels_;
    std::vector<int> device_ids_, rx_slots_, signs_;
    double wheel_radius_, counts_per_rev_, max_step_;
    std::string odom_frame_, base_frame_;
    bool publish_tf_;
    int period_ms_;
    bool use_imu_;
    std::string imu_topic_;
    int imu_yaw_sign_;
    double imu_timeout_;

    // ===== 状態 =====
    mini_auto::TrackingOdometry odometry_;
    std::array<int32_t, kWheelNum> counts_{};
    std::array<int32_t, kWheelNum> last_counts_{};
    std::array<bool, kWheelNum> received_{};
    bool initialized_ = false;
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
    mini_auto::Pose2D pose_;
    mini_auto::Twist2D twist_;
    bool has_imu_ = false;     // 一度でも受信したか
    bool imu_active_ = false;  // yaw の基準が取れていて、今 IMU を使っているか
    double imu_yaw_ = 0.0;     // 最新
    double last_imu_yaw_ = 0.0;  // 前回の積分で使った値
    rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};

    // ===== ROS =====
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::map<int, rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr> enc_subs_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackingOdom>());
    rclcpp::shutdown();
    return 0;
}
