/*
簡易ナビゲーションノード (ホロノミック用)

  /goal_pose (RViz の 2D Goal Pose)  → 目標
  /map  (slam_ros2 のヒットグリッド: 占有=100, 未知=-1)
  + obstacle_topics の LaserScan (TG30 の /scan, D456 由来の /depth_scan など) → 障害物
  TF map -> base_link (slam_ros2 が map->odom、tracking_odom が odom->base_link)
  → A* で経路計画 (plan_period ごとに再計画) → pure pursuit で /cmd_vel

オムニなので、先読み点へ向かう速度ベクトルをそのまま機体座標の (vx, vy) にし、
向きは独立に P 制御する。face_travel_direction: true なら移動中は進行方向を向き
(前方しか見えない D456 で進む先を見るため)、ゴール手前 face_goal_distance 以内でゴールの yaw に合わせる。
向きのずれが大きい間は並進を cos(ずれ) で落とし、カメラが前を向いてから進む。
ヒットグリッドには「空いている」情報がないので、未知セルは通行可能として扱う。

障害物スキャンは map 座標に直して obstacle_memory 秒だけ覚えておき、計画に使う。
(D456 は前方しか見えないので、横移動や旋回で視野から外れた障害物も忘れないようにする)
安全: 各トピックの最新スキャンのうち、進行方向の機体幅の帯にある点までの距離で減速・停止する。
*/

#include <algorithm>
#include <chrono>
#include <deque>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mini_auto/grid_planner.hpp"
#include "mini_auto/kinematics.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class SimpleNav : public rclcpp::Node {
public:
    SimpleNav() : Node("simple_nav") {
        DeclareParameters();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        plan_pub_ = create_publisher<nav_msgs::msg::Path>("nav/plan", 10);

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(1).transient_local(),
            [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { map_ = msg; });
        sources_.resize(obstacle_topics_.size());
        for (size_t i = 0; i < obstacle_topics_.size(); ++i) {
            sources_[i].sub = create_subscription<sensor_msgs::msg::LaserScan>(
                obstacle_topics_[i], rclcpp::SensorDataQoS(),
                [this, i](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { ScanCallback(i, msg); });
        }
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&SimpleNav::GoalCallback, this, std::placeholders::_1));
        cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
            "nav/cancel", 10, [this](std_msgs::msg::Empty::ConstSharedPtr) {
                RCLCPP_INFO(get_logger(), "goal canceled");
                ClearGoal();
            });

        timer_ = create_wall_timer(std::chrono::milliseconds(period_ms_),
                                   std::bind(&SimpleNav::ControlLoop, this));
        RCLCPP_INFO(get_logger(), "simple nav started; send a goal on /goal_pose");
    }

private:
    struct Point {
        double x, y;
    };

    // 障害物スキャン1トピックぶん。点はすべて map 座標
    struct ObstacleSource {
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
        std::deque<std::pair<rclcpp::Time, std::vector<Point>>> history;  // 古い順
    };

    // ===== 初期化 =====
    void DeclareParameters() {
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        period_ms_ = declare_parameter<int>("period_ms", 50);

        // 計画
        robot_radius_ = declare_parameter<double>("robot_radius", 0.25);         // [m]
        inflation_radius_ = declare_parameter<double>("inflation_radius", 0.5);  // [m]
        // 計画で通行不可にする範囲を robot_radius からさらに広げる量。先読み追従は角を内側に
        // ショートカットするので、その分の余裕 [m]
        footprint_padding_ = declare_parameter<double>("footprint_padding", 0.08);
        cost_weight_ = declare_parameter<double>("cost_weight", 3.0);
        occupied_threshold_ = declare_parameter<int>("occupied_threshold", 50);
        use_scan_obstacles_ = declare_parameter<bool>("use_scan_obstacles", true);
        obstacle_topics_ = declare_parameter<std::vector<std::string>>(
            "obstacle_topics", std::vector<std::string>{"scan"});
        obstacle_memory_ = declare_parameter<double>("obstacle_memory", 3.0);    // [s]
        obstacle_timeout_ = declare_parameter<double>("obstacle_timeout", 0.5);  // [s] 安全判定に使う鮮度
        // 機体中心からこの半径以内の点は自機(支柱・ケーブル等)とみなして捨てる。0 で無効 [m]
        self_filter_radius_ = declare_parameter<double>("self_filter_radius", 0.0);
        plan_period_ = declare_parameter<double>("plan_period", 1.0);    // [s]
        plan_margin_ = declare_parameter<double>("plan_margin", 1.0);    // 地図外に広げる量 [m]
        max_plan_failures_ = declare_parameter<int>("max_plan_failures", 5);

        // 追従
        max_vel_ = declare_parameter<double>("max_vel", 0.4);        // [m/s]
        min_vel_ = declare_parameter<double>("min_vel", 0.05);       // 摩擦に負けない最低速度 [m/s]
        max_decel_ = declare_parameter<double>("max_decel", 0.5);    // ゴール手前の減速 [m/s^2]
        max_wz_ = declare_parameter<double>("max_wz", 1.0);          // [rad/s]
        yaw_kp_ = declare_parameter<double>("yaw_kp", 1.5);
        face_travel_direction_ = declare_parameter<bool>("face_travel_direction", true);
        face_goal_distance_ = declare_parameter<double>("face_goal_distance", 0.5);  // [m]
        min_heading_scale_ = declare_parameter<double>("min_heading_scale", 0.2);
        lookahead_ = declare_parameter<double>("lookahead", 0.4);    // [m]
        xy_tolerance_ = declare_parameter<double>("xy_tolerance", 0.05);    // [m]
        yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.05);  // [rad]

        // 安全(機体の縁からの距離)
        stop_gap_ = declare_parameter<double>("stop_gap", 0.10);  // [m]
        slow_gap_ = declare_parameter<double>("slow_gap", 0.40);  // [m]
    }

    // ===== 受信 =====
    void GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        geometry_msgs::msg::PoseStamped goal = *msg;
        if (goal.header.frame_id != map_frame_) {
            try {
                goal = tf_buffer_->transform(goal, map_frame_, tf2::durationFromSec(0.2));
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(get_logger(), "goal rejected: %s", ex.what());
                return;
            }
        }
        goal_ = {goal.pose.position.x, goal.pose.position.y};
        goal_yaw_ = tf2::getYaw(goal.pose.orientation);
        has_goal_ = true;
        path_.clear();
        plan_failures_ = 0;
        last_plan_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        RCLCPP_INFO(get_logger(), "new goal (%.2f, %.2f, %.0f deg)", goal_.x, goal_.y,
                    goal_yaw_ * 180.0 / M_PI);
    }

    // スキャン点を map 座標に直して履歴に積む
    void ScanCallback(size_t source, const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        geometry_msgs::msg::TransformStamped sensor_tf, base_tf;
        try {
            sensor_tf = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id, tf2::TimePointZero);
            base_tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s TF: %s",
                                 obstacle_topics_[source].c_str(), ex.what());
            return;
        }
        const double ox = sensor_tf.transform.translation.x;
        const double oy = sensor_tf.transform.translation.y;
        const double oyaw = tf2::getYaw(sensor_tf.transform.rotation);
        const double bx = base_tf.transform.translation.x;
        const double by = base_tf.transform.translation.y;

        std::vector<Point> points;
        points.reserve(msg->ranges.size());
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            const double r = msg->ranges[i];
            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
                continue;
            }
            const double a = oyaw + msg->angle_min + i * msg->angle_increment;
            const Point p{ox + r * std::cos(a), oy + r * std::sin(a)};
            if (std::hypot(p.x - bx, p.y - by) < self_filter_radius_) {
                continue;
            }
            points.push_back(p);
        }

        const rclcpp::Time t = now();
        auto &history = sources_[source].history;
        history.emplace_back(t, std::move(points));
        while (!history.empty() && (t - history.front().first).seconds() > obstacle_memory_) {
            history.pop_front();
        }
    }

    // ===== 制御 =====
    void ControlLoop() {
        if (!has_goal_) {
            return;
        }
        mini_auto::Pose2D pose;
        if (!LookupPose(&pose)) {
            PublishCmd(0.0, 0.0, 0.0);
            return;
        }

        if (path_.empty() || (now() - last_plan_time_).seconds() >= plan_period_) {
            Plan(pose);
        }
        if (path_.empty()) {
            PublishCmd(0.0, 0.0, 0.0);
            return;
        }

        const double dist = std::hypot(goal_.x - pose.x, goal_.y - pose.y);
        const double yaw_err = mini_auto::WrapPi(goal_yaw_ - pose.yaw);
        if (dist < xy_tolerance_ && std::fabs(yaw_err) < yaw_tolerance_) {
            RCLCPP_INFO(get_logger(), "goal reached (err %.3f m, %.1f deg)", dist,
                        yaw_err * 180.0 / M_PI);
            ClearGoal();
            return;
        }

        // 並進: 先読み点へ向かうベクトル (map座標) → 機体座標
        double vx = 0.0, vy = 0.0;
        double target_yaw = goal_yaw_;
        if (dist >= xy_tolerance_) {
            const Point target = LookaheadPoint(pose);
            const double dx = target.x - pose.x;
            const double dy = target.y - pose.y;
            const double len = std::hypot(dx, dy);
            if (len > 1e-6) {
                double speed = std::min(max_vel_, std::sqrt(2.0 * max_decel_ * dist));
                speed = std::max(speed, min_vel_);
                if (face_travel_direction_ && dist > face_goal_distance_) {
                    target_yaw = std::atan2(dy, dx);
                    const double heading_err = mini_auto::WrapPi(target_yaw - pose.yaw);
                    speed *= std::max(min_heading_scale_, std::cos(heading_err));
                }
                const double mx = dx / len * speed;
                const double my = dy / len * speed;
                const double c = std::cos(pose.yaw);
                const double s = std::sin(pose.yaw);
                vx = c * mx + s * my;
                vy = -s * mx + c * my;
            }
        }
        const double scale = SafetyScale(pose, vx, vy);
        vx *= scale;
        vy *= scale;

        const double wz = std::clamp(yaw_kp_ * mini_auto::WrapPi(target_yaw - pose.yaw), -max_wz_, max_wz_);
        PublishCmd(vx, vy, wz);
    }

    bool LookupPose(mini_auto::Pose2D *pose) {
        try {
            const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
            pose->x = tf.transform.translation.x;
            pose->y = tf.transform.translation.y;
            pose->yaw = tf2::getYaw(tf.transform.rotation);
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "pose TF: %s", ex.what());
            return false;
        }
    }

    // ===== 計画 =====
    void Plan(const mini_auto::Pose2D &pose) {
        last_plan_time_ = now();
        mini_auto::CostGrid grid(MakeGridInfo(pose));
        if (map_) {
            const auto &mi = map_->info;
            for (unsigned int row = 0; row < mi.height; ++row) {
                for (unsigned int col = 0; col < mi.width; ++col) {
                    if (map_->data[row * mi.width + col] >= occupied_threshold_) {
                        grid.MarkObstacle(mi.origin.position.x + (col + 0.5) * mi.resolution,
                                          mi.origin.position.y + (row + 0.5) * mi.resolution);
                    }
                }
            }
        }
        if (use_scan_obstacles_) {
            for (const auto &source : sources_) {
                for (const auto &[stamp, points] : source.history) {
                    for (const auto &p : points) {
                        grid.MarkObstacle(p.x, p.y);
                    }
                }
            }
        }
        grid.Inflate(robot_radius_ + footprint_padding_, inflation_radius_);

        mini_auto::Path2D raw;
        std::string error;
        if (!mini_auto::PlanPath(grid, pose.x, pose.y, goal_.x, goal_.y, cost_weight_,
                                 robot_radius_ + footprint_padding_, &raw, &error)) {
            path_.clear();
            ++plan_failures_;
            RCLCPP_WARN(get_logger(), "planning failed (%d/%d): %s", plan_failures_,
                        max_plan_failures_, error.c_str());
            if (plan_failures_ >= max_plan_failures_) {
                RCLCPP_ERROR(get_logger(), "giving up on this goal");
                ClearGoal();
            }
            return;
        }
        plan_failures_ = 0;
        path_.clear();
        for (const auto &[x, y] : raw) {
            path_.push_back({x, y});
        }
        progress_ = 0;
        PublishPlan();
    }

    // 地図・現在地・ゴールをすべて含む範囲にする(未知は通行可なので地図外も計画できる)
    mini_auto::GridInfo MakeGridInfo(const mini_auto::Pose2D &pose) const {
        const double res = map_ ? map_->info.resolution : 0.05;
        double min_x = std::min(pose.x, goal_.x);
        double max_x = std::max(pose.x, goal_.x);
        double min_y = std::min(pose.y, goal_.y);
        double max_y = std::max(pose.y, goal_.y);
        if (map_) {
            const auto &mi = map_->info;
            min_x = std::min(min_x, mi.origin.position.x);
            min_y = std::min(min_y, mi.origin.position.y);
            max_x = std::max(max_x, mi.origin.position.x + mi.width * mi.resolution);
            max_y = std::max(max_y, mi.origin.position.y + mi.height * mi.resolution);
        }
        mini_auto::GridInfo info;
        info.resolution = res;
        info.origin_x = min_x - plan_margin_;
        info.origin_y = min_y - plan_margin_;
        info.width = static_cast<int>(std::ceil((max_x - min_x + 2.0 * plan_margin_) / res));
        info.height = static_cast<int>(std::ceil((max_y - min_y + 2.0 * plan_margin_) / res));
        return info;
    }

    // ===== 追従 =====
    // 経路上の最寄り点(前進方向のみ更新)から lookahead 進んだ点
    Point LookaheadPoint(const mini_auto::Pose2D &pose) {
        double best = std::numeric_limits<double>::infinity();
        const size_t search_end = std::min(path_.size(), progress_ + 50);
        for (size_t i = progress_; i < search_end; ++i) {
            const double d = std::hypot(path_[i].x - pose.x, path_[i].y - pose.y);
            if (d < best) {
                best = d;
                progress_ = i;
            }
        }
        double acc = 0.0;
        for (size_t i = progress_; i + 1 < path_.size(); ++i) {
            acc += std::hypot(path_[i + 1].x - path_[i].x, path_[i + 1].y - path_[i].y);
            if (acc >= lookahead_) {
                return path_[i + 1];
            }
        }
        return path_.back();
    }

    // 進行方向の機体幅の帯にあるスキャン点までの距離で 0..1 の速度倍率を返す
    double SafetyScale(const mini_auto::Pose2D &pose, double vx, double vy) {
        const double speed = std::hypot(vx, vy);
        if (speed < 1e-6) {
            return 1.0;
        }
        // 進行方向 (機体座標) を map 座標に回す
        const double c = std::cos(pose.yaw);
        const double s = std::sin(pose.yaw);
        const double ux = (c * vx - s * vy) / speed;
        const double uy = (s * vx + c * vy) / speed;
        const rclcpp::Time t = now();
        double nearest = std::numeric_limits<double>::infinity();
        for (const auto &source : sources_) {
            if (source.history.empty() ||
                (t - source.history.back().first).seconds() > obstacle_timeout_) {
                continue;
            }
            for (const auto &p : source.history.back().second) {
                const double rx = p.x - pose.x;
                const double ry = p.y - pose.y;
                const double along = rx * ux + ry * uy;
                const double lateral = std::fabs(-rx * uy + ry * ux);
                if (along > 0.0 && lateral < robot_radius_) {
                    nearest = std::min(nearest, along - robot_radius_);
                }
            }
        }
        const double scale = std::clamp((nearest - stop_gap_) / (slow_gap_ - stop_gap_), 0.0, 1.0);
        if (scale == 0.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "obstacle %.2f m ahead; stopping and replanning", nearest);
            last_plan_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        }
        return scale;
    }

    // ===== 送信 =====
    void ClearGoal() {
        has_goal_ = false;
        path_.clear();
        PublishCmd(0.0, 0.0, 0.0);
        PublishPlan();
    }

    void PublishCmd(double vx, double vy, double wz) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.angular.z = wz;
        cmd_pub_->publish(cmd);
    }

    void PublishPlan() {
        nav_msgs::msg::Path msg;
        msg.header.stamp = now();
        msg.header.frame_id = map_frame_;
        for (const auto &p : path_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        plan_pub_->publish(msg);
    }

    // ===== パラメータ =====
    std::string map_frame_, base_frame_;
    int period_ms_;
    double robot_radius_, inflation_radius_, footprint_padding_, cost_weight_;
    int occupied_threshold_;
    bool use_scan_obstacles_;
    std::vector<std::string> obstacle_topics_;
    double obstacle_memory_, obstacle_timeout_, self_filter_radius_;
    double plan_period_, plan_margin_;
    int max_plan_failures_;
    double max_vel_, min_vel_, max_decel_, max_wz_, yaw_kp_, lookahead_;
    bool face_travel_direction_;
    double face_goal_distance_, min_heading_scale_;
    double xy_tolerance_, yaw_tolerance_, stop_gap_, slow_gap_;

    // ===== 状態 =====
    bool has_goal_ = false;
    Point goal_{0.0, 0.0};
    double goal_yaw_ = 0.0;
    std::vector<Point> path_;
    size_t progress_ = 0;
    int plan_failures_ = 0;
    rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
    std::vector<ObstacleSource> sources_;  // obstacle_topics と同じ順

    // ===== ROS =====
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleNav>());
    rclcpp::shutdown();
    return 0;
}
