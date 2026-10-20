/*
4輪オムニ 足回りノード

cmd_vel (機体座標の vx, vy, wz) を加速度制限したうえで各輪の転がり速度に変換し、
duty にして ros2can の serial_tx_[ID] に送る。ドライブはオープンループ
(速度 max_wheel_speed ⇔ duty_max の比例)。位置のずれは上位(SLAM + ナビ)で吸収する。

1輪でも max_wheel_speed を超える場合は、全輪を同じ比率で落として進行方向を保つ。
*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "mini_auto/kinematics.hpp"
#include "mini_auto/params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

namespace {
constexpr size_t kWheelNum = 4;
}  // namespace

class OmniDrive : public rclcpp::Node {
public:
    OmniDrive() : Node("omni_drive") {
        DeclareParameters();

        for (int id : device_ids_) {
            if (tx_pubs_.count(id) == 0) {
                tx_pubs_[id] = create_publisher<std_msgs::msg::Int16MultiArray>(
                    "serial_tx_" + std::to_string(id), 10);
                tx_data_[id].assign(tx_slot_num_, 0);
            }
        }
        state_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("omni_drive/wheel_speed", 10);

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&OmniDrive::CmdCallback, this, std::placeholders::_1));
        timer_ = create_wall_timer(std::chrono::milliseconds(period_ms_),
                                   std::bind(&OmniDrive::ControlLoop, this));

        last_cmd_time_ = now();
        last_loop_time_ = now();
        RCLCPP_INFO(get_logger(), "omni drive started (%zu wheels, max %.2f m/s/wheel)",
                    wheels_.size(), max_wheel_speed_);
    }

    void SendZero() {
        for (auto &[id, data] : tx_data_) {
            std::fill(data.begin(), data.end(), 0);
        }
        PublishTx();
    }

private:
    // ===== 初期化 =====
    void DeclareParameters() {
        // 取り付け位置と転がり方向(robot.yaml の /** で共有)
        wheels_ = mini_auto::DeclareWheelMounts(*this, "drive_wheel", kWheelNum);
        // 各輪の送信先 serial_tx_[ID] とスロット
        device_ids_ = mini_auto::DeclareIntArray(*this, "drive_device_ids", kWheelNum);
        tx_slots_ = mini_auto::DeclareIntArray(*this, "drive_tx_slots", kWheelNum);
        signs_ = mini_auto::DeclareIntArray(*this, "drive_signs", kWheelNum);
        tx_slot_num_ = declare_parameter<int>("tx_slot_num", 24);
        for (int slot : tx_slots_) {
            if (slot < 0 || slot >= tx_slot_num_) {
                throw std::runtime_error("drive_tx_slots out of range");
            }
        }

        max_wheel_speed_ = declare_parameter<double>("max_wheel_speed", 1.5);  // duty_max に対応 [m/s]
        duty_max_ = declare_parameter<double>("duty_max", 255.0);
        max_acc_ = declare_parameter<double>("max_acc", 1.0);          // [m/s^2]
        max_ang_acc_ = declare_parameter<double>("max_ang_acc", 3.0);  // [rad/s^2]
        cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);  // [s]
        period_ms_ = declare_parameter<int>("period_ms", 20);
    }

    // ===== 受信 =====
    void CmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_.vx = msg->linear.x;
        target_.vy = msg->linear.y;
        target_.wz = msg->angular.z;
        last_cmd_time_ = now();
    }

    // ===== 制御 =====
    void ControlLoop() {
        const rclcpp::Time t = now();
        const double dt = std::clamp((t - last_loop_time_).seconds(), 0.0, 0.1);
        last_loop_time_ = t;

        // 指令が途切れたら停止(加速度制限つきで減速する)
        if ((t - last_cmd_time_).seconds() > cmd_timeout_) {
            target_ = mini_auto::Twist2D{};
        }
        ApplyAccelLimit(dt);

        std::vector<double> speeds = mini_auto::OmniWheelSpeeds(wheels_, current_);
        double peak = 0.0;
        for (double s : speeds) {
            peak = std::max(peak, std::fabs(s));
        }
        if (peak > max_wheel_speed_) {
            const double k = max_wheel_speed_ / peak;
            for (double &s : speeds) {
                s *= k;
            }
        }

        for (size_t i = 0; i < kWheelNum; ++i) {
            const double duty = signs_[i] * speeds[i] / max_wheel_speed_ * duty_max_;
            tx_data_[device_ids_[i]][tx_slots_[i]] = static_cast<int16_t>(std::lround(duty));
        }
        PublishTx();
        PublishState(speeds);
    }

    // 並進は速度ベクトルの変化量の大きさで、回転は単独で制限する
    void ApplyAccelLimit(double dt) {
        const double dvx = target_.vx - current_.vx;
        const double dvy = target_.vy - current_.vy;
        const double dv = std::hypot(dvx, dvy);
        const double dv_max = max_acc_ * dt;
        const double k = (dv > dv_max && dv > 0.0) ? dv_max / dv : 1.0;
        current_.vx += dvx * k;
        current_.vy += dvy * k;
        const double dw_max = max_ang_acc_ * dt;
        current_.wz += std::clamp(target_.wz - current_.wz, -dw_max, dw_max);
    }

    // ===== 送信 =====
    void PublishTx() {
        for (auto &[id, pub] : tx_pubs_) {
            std_msgs::msg::Int16MultiArray msg;
            msg.data = tx_data_[id];
            pub->publish(msg);
        }
    }

    // 各輪の目標転がり速度 [m/s]
    void PublishState(const std::vector<double> &speeds) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data.assign(speeds.begin(), speeds.end());
        state_pub_->publish(msg);
    }

    // ===== パラメータ =====
    std::vector<mini_auto::WheelMount> wheels_;
    std::vector<int> device_ids_, tx_slots_, signs_;
    int tx_slot_num_;
    double max_wheel_speed_, duty_max_, max_acc_, max_ang_acc_, cmd_timeout_;
    int period_ms_;

    // ===== 状態 =====
    mini_auto::Twist2D target_;   // 最新の cmd_vel
    mini_auto::Twist2D current_;  // 加速度制限後
    rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_loop_time_{0, 0, RCL_ROS_TIME};
    std::map<int, std::vector<int16_t>> tx_data_;

    // ===== ROS =====
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr> tx_pubs_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OmniDrive>();
    rclcpp::spin(node);
    node->SendZero();
    rclcpp::shutdown();
    return 0;
}
