/*
グリッド上の経路計画 (ROS非依存)

CostGrid: 障害物セルを登録し、ロボット半径で膨張させたコストを作る
  - 障害物から robot_radius 以内      → kLethal (通行不可)
  - robot_radius 〜 inflation_radius   → 1..kLethal-1 (通れるが壁から離れたい)
  - それ以外                          → 0
PlanPath: 8近傍 A*。ステップコスト = 距離 * (1 + cost_weight * cost / kLethal)
*/

#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace mini_auto {

struct GridInfo {
    double origin_x = 0.0;  // セル(0,0)の左下角 [m]
    double origin_y = 0.0;
    double resolution = 0.05;  // [m/cell]
    int width = 0;
    int height = 0;
};

class CostGrid {
public:
    static constexpr uint8_t kLethal = 254;

    explicit CostGrid(const GridInfo &info);

    const GridInfo &info() const { return info_; }
    bool InBounds(int cx, int cy) const;
    bool WorldToCell(double x, double y, int *cx, int *cy) const;
    void CellToWorld(int cx, int cy, double *x, double *y) const;

    void MarkObstacle(double x, double y);
    void MarkObstacleCell(int cx, int cy);
    // 障害物の登録が終わってから呼ぶ
    void Inflate(double robot_radius, double inflation_radius);

    uint8_t Cost(int cx, int cy) const { return cost_[Index(cx, cy)]; }

private:
    size_t Index(int cx, int cy) const { return static_cast<size_t>(cy) * info_.width + cx; }

    GridInfo info_;
    std::vector<uint8_t> occupied_;
    std::vector<uint8_t> cost_;
};

using Path2D = std::vector<std::pair<double, double>>;

// start/goal がLethal内なら search_radius [m] 以内の最寄りの通行可能セルに置き換える。
// 失敗したら false と理由を返す。
bool PlanPath(const CostGrid &grid, double sx, double sy, double gx, double gy,
              double cost_weight, double search_radius, Path2D *path, std::string *error);

}  // namespace mini_auto
