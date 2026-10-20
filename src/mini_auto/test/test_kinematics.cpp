#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "mini_auto/kinematics.hpp"

using mini_auto::Twist2D;
using mini_auto::WheelMount;

namespace {

constexpr double kDeg = M_PI / 180.0;

// robot.yaml のデフォルトと同じ: 半径 r, pos_deg の位置に接線方向で取り付け
WheelMount Tangent(double r, double pos_deg) {
    return {r * std::cos(pos_deg * kDeg), r * std::sin(pos_deg * kDeg), (pos_deg + 90.0) * kDeg};
}

std::array<WheelMount, 3> Triangle() {
    return {Tangent(0.12, 90.0), Tangent(0.12, 210.0), Tangent(0.12, 330.0)};
}

std::vector<WheelMount> OmniX() {
    return {Tangent(0.2, 45.0), Tangent(0.2, 135.0), Tangent(0.2, 225.0), Tangent(0.2, 315.0)};
}

}  // namespace

TEST(Kinematics, PureRotationSpinsEveryTangentWheelEqually) {
    const auto speeds = mini_auto::OmniWheelSpeeds(OmniX(), {0.0, 0.0, 1.0});
    for (double s : speeds) {
        EXPECT_NEAR(s, 0.2, 1e-9);
    }
}

TEST(Kinematics, OmniForwardInverseRoundTrip) {
    const Twist2D in{0.3, -0.2, 0.7};
    Twist2D out;
    ASSERT_TRUE(mini_auto::OmniBodyTwist(OmniX(), mini_auto::OmniWheelSpeeds(OmniX(), in), &out));
    EXPECT_NEAR(out.vx, in.vx, 1e-9);
    EXPECT_NEAR(out.vy, in.vy, 1e-9);
    EXPECT_NEAR(out.wz, in.wz, 1e-9);
}

TEST(Kinematics, TrackingOdometryRecoversBodyDelta) {
    mini_auto::TrackingOdometry odo;
    ASSERT_TRUE(odo.Configure(Triangle()));
    const Twist2D delta{0.01, -0.004, 0.02};
    mini_auto::Row3 d;
    for (int i = 0; i < 3; ++i) {
        d[i] = mini_auto::WheelSurfaceSpeed(Triangle()[i], delta);
    }
    const Twist2D out = odo.BodyDelta(d);
    EXPECT_NEAR(out.vx, delta.vx, 1e-12);
    EXPECT_NEAR(out.vy, delta.vy, 1e-12);
    EXPECT_NEAR(out.wz, delta.wz, 1e-12);
}

TEST(Kinematics, TrackingOdometryWithImuYaw) {
    mini_auto::TrackingOdometry odo;
    ASSERT_TRUE(odo.Configure(Triangle()));
    const Twist2D delta{0.01, -0.004, 0.02};
    mini_auto::Row3 d;
    for (int i = 0; i < 3; ++i) {
        d[i] = mini_auto::WheelSurfaceSpeed(Triangle()[i], delta);
    }
    // IMU が正しい回転量を言えば並進もそのまま復元できる
    Twist2D out = odo.BodyDeltaWithYaw(d, delta.wz);
    EXPECT_NEAR(out.vx, delta.vx, 1e-12);
    EXPECT_NEAR(out.vy, delta.vy, 1e-12);
    EXPECT_DOUBLE_EQ(out.wz, delta.wz);

    // 回転は IMU の値がそのまま採用される
    out = odo.BodyDeltaWithYaw(d, 0.0);
    EXPECT_DOUBLE_EQ(out.wz, 0.0);
}

TEST(Kinematics, DegenerateTrackingLayoutIsRejected) {
    // 3輪とも同じ向き → 回転と横移動が分離できない
    mini_auto::TrackingOdometry odo;
    EXPECT_FALSE(odo.Configure({WheelMount{0.0, 0.1, 0.0}, WheelMount{0.0, -0.1, 0.0},
                                WheelMount{0.1, 0.0, 0.0}}));
}

TEST(Kinematics, IntegrateDrivesInWorldFrame) {
    // 90°向いた機体が前に 1 m 進む → ワールドでは +y
    const auto p = mini_auto::Integrate({0.0, 0.0, M_PI / 2.0}, {1.0, 0.0, 0.0});
    EXPECT_NEAR(p.x, 0.0, 1e-9);
    EXPECT_NEAR(p.y, 1.0, 1e-9);
}

TEST(Kinematics, CircleIntegrationClosesLoop) {
    // 前進しながら回転を 1 周分刻んで積分すると元の位置に戻る
    mini_auto::Pose2D p;
    const int n = 1000;
    for (int i = 0; i < n; ++i) {
        p = mini_auto::Integrate(p, {2.0 * M_PI / n, 0.0, 2.0 * M_PI / n});
    }
    EXPECT_NEAR(p.x, 0.0, 1e-6);
    EXPECT_NEAR(p.y, 0.0, 1e-6);
    EXPECT_NEAR(p.yaw, 0.0, 1e-6);
}
