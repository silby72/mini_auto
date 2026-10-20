/*
平面ロボットの運動学ユーティリティ (ROS非依存)

駆動輪(4輪オムニ)も計測輪(3輪トラッキングホイール)も、
「ロボット中心からの位置 (x, y)」と「正転したとき接地点が進む向き dir」だけで表す。
車輪の転がり方向の速度は
    v_i = cos(dir) * (vx - wz * y) + sin(dir) * (vy + wz * x)
になる。これを行列にしたものがヤコビアン J (行 = [cos, sin, x sin - y cos])。
  - 4輪オムニの逆運動学:   v = J * twist
  - 3輪計測輪の順運動学:   delta = J^-1 * d   (3x3 なので逆行列が一意に決まる)
*/

#pragma once

#include <array>
#include <cmath>
#include <vector>

namespace mini_auto {

struct Twist2D {
    double vx = 0.0;  // 機体座標 [m/s] (積分用の微小移動として [m] でも使う)
    double vy = 0.0;
    double wz = 0.0;  // [rad/s] (同上 [rad])
};

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

// 車輪1個の取り付け
struct WheelMount {
    double x = 0.0;    // ロボット中心からの位置 [m]
    double y = 0.0;
    double dir = 0.0;  // 正転時に接地点が進む向き [rad] (機体座標)
};

using Row3 = std::array<double, 3>;
using Mat3 = std::array<Row3, 3>;

// 角度を [-pi, pi] に正規化する
inline double WrapPi(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

// ヤコビアンの1行
inline Row3 JacobianRow(const WheelMount &w) {
    const double c = std::cos(w.dir);
    const double s = std::sin(w.dir);
    return {c, s, w.x * s - w.y * c};
}

// ロボットが twist で動くときの、車輪の転がり方向の速度
inline double WheelSurfaceSpeed(const WheelMount &w, const Twist2D &t) {
    const Row3 j = JacobianRow(w);
    return j[0] * t.vx + j[1] * t.vy + j[2] * t.wz;
}

// 3x3 の逆行列。特異(|det| が小さい)なら false。
inline bool Invert3(const Mat3 &m, Mat3 *inv) {
    const double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                       m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                       m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if (std::fabs(det) < 1e-9) {
        return false;
    }
    const double k = 1.0 / det;
    (*inv)[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * k;
    (*inv)[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * k;
    (*inv)[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * k;
    (*inv)[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * k;
    (*inv)[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * k;
    (*inv)[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * k;
    (*inv)[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * k;
    (*inv)[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * k;
    (*inv)[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * k;
    return true;
}

inline Twist2D Multiply(const Mat3 &m, const Row3 &v) {
    Twist2D t;
    t.vx = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2];
    t.vy = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2];
    t.wz = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2];
    return t;
}

// 3輪計測輪オドメトリ: 各輪の移動距離 → 機体座標での微小移動
class TrackingOdometry {
public:
    // 配置が退化している(3輪の情報で vx, vy, wz が決まらない)なら false
    bool Configure(const std::array<WheelMount, 3> &wheels) {
        j_ = {JacobianRow(wheels[0]), JacobianRow(wheels[1]), JacobianRow(wheels[2])};
        return Invert3(j_, &inv_);
    }

    Twist2D BodyDelta(const Row3 &wheel_distances) const { return Multiply(inv_, wheel_distances); }

    // 回転量 dtheta を外部(IMU)から与えたときの微小移動。
    // 各輪の読みから回転ぶん j2 * dtheta を引き、残りから (dx, dy) を最小二乗で解く。
    Twist2D BodyDeltaWithYaw(const Row3 &wheel_distances, double dtheta) const {
        double a00 = 0.0, a01 = 0.0, a11 = 0.0, b0 = 0.0, b1 = 0.0;
        for (int i = 0; i < 3; ++i) {
            const double r = wheel_distances[i] - j_[i][2] * dtheta;
            a00 += j_[i][0] * j_[i][0];
            a01 += j_[i][0] * j_[i][1];
            a11 += j_[i][1] * j_[i][1];
            b0 += j_[i][0] * r;
            b1 += j_[i][1] * r;
        }
        // Configure が通っていれば (dx, dy) の列は独立なので det > 0
        const double det = a00 * a11 - a01 * a01;
        Twist2D t;
        t.vx = (a11 * b0 - a01 * b1) / det;
        t.vy = (a00 * b1 - a01 * b0) / det;
        t.wz = dtheta;
        return t;
    }

private:
    Mat3 j_{};
    Mat3 inv_{};
};

// 4輪(N輪)オムニの逆運動学: 機体速度 → 各輪の転がり速度
inline std::vector<double> OmniWheelSpeeds(const std::vector<WheelMount> &wheels,
                                           const Twist2D &twist) {
    std::vector<double> speeds;
    speeds.reserve(wheels.size());
    for (const auto &w : wheels) {
        speeds.push_back(WheelSurfaceSpeed(w, twist));
    }
    return speeds;
}

// オムニの順運動学(最小二乗): 各輪の転がり速度 → 機体速度。twist = (J^T J)^-1 J^T v
inline bool OmniBodyTwist(const std::vector<WheelMount> &wheels,
                          const std::vector<double> &speeds, Twist2D *twist) {
    if (wheels.size() != speeds.size() || wheels.size() < 3) {
        return false;
    }
    Mat3 jtj{};
    Row3 jtv{};
    for (size_t i = 0; i < wheels.size(); ++i) {
        const Row3 j = JacobianRow(wheels[i]);
        for (int r = 0; r < 3; ++r) {
            jtv[r] += j[r] * speeds[i];
            for (int c = 0; c < 3; ++c) {
                jtj[r][c] += j[r] * j[c];
            }
        }
    }
    Mat3 inv;
    if (!Invert3(jtj, &inv)) {
        return false;
    }
    *twist = Multiply(inv, jtv);
    return true;
}

// 機体座標の微小移動 delta を、ワールド座標の姿勢に積分する(中点法)
inline Pose2D Integrate(const Pose2D &pose, const Twist2D &delta) {
    const double mid = pose.yaw + 0.5 * delta.wz;
    Pose2D out;
    out.x = pose.x + std::cos(mid) * delta.vx - std::sin(mid) * delta.vy;
    out.y = pose.y + std::sin(mid) * delta.vx + std::cos(mid) * delta.vy;
    out.yaw = WrapPi(pose.yaw + delta.wz);
    return out;
}

}  // namespace mini_auto
