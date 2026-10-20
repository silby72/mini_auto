#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "mini_auto/grid_planner.hpp"

using mini_auto::CostGrid;
using mini_auto::GridInfo;
using mini_auto::Path2D;

namespace {

// 4 m x 4 m, 0.05 m/cell
GridInfo Info() {
    GridInfo info;
    info.origin_x = 0.0;
    info.origin_y = 0.0;
    info.resolution = 0.05;
    info.width = 80;
    info.height = 80;
    return info;
}

// x = 2.0 の壁。y < gap_bottom を塞ぐ
void AddWall(CostGrid *grid, double gap_bottom) {
    for (double y = 0.0; y < gap_bottom; y += 0.025) {
        grid->MarkObstacle(2.0, y);
    }
}

}  // namespace

TEST(GridPlanner, StraightLineInFreeSpace) {
    CostGrid grid(Info());
    grid.Inflate(0.2, 0.4);
    Path2D path;
    std::string err;
    ASSERT_TRUE(mini_auto::PlanPath(grid, 0.5, 0.5, 3.5, 0.5, 3.0, 0.2, &path, &err)) << err;
    EXPECT_NEAR(path.back().first, 3.5, 1e-9);
    for (const auto &p : path) {
        EXPECT_NEAR(p.second, 0.5, 0.05);
    }
}

TEST(GridPlanner, GoesThroughGapWithClearance) {
    CostGrid grid(Info());
    AddWall(&grid, 3.0);  // 上 1 m だけ空いている
    grid.Inflate(0.2, 0.4);
    Path2D path;
    std::string err;
    ASSERT_TRUE(mini_auto::PlanPath(grid, 0.5, 0.5, 3.5, 0.5, 3.0, 0.2, &path, &err)) << err;
    bool crossed_in_gap = false;
    for (const auto &[x, y] : path) {
        // 壁の端 (2.0, 3.0) から robot_radius 以上離れている
        if (y < 3.0) {
            EXPECT_GT(std::fabs(x - 2.0), 0.2);
        } else {
            EXPECT_GT(std::hypot(x - 2.0, y - 3.0), 0.2);
        }
        crossed_in_gap |= (std::fabs(x - 2.0) < 0.05 && y > 3.0);
    }
    EXPECT_TRUE(crossed_in_gap);
}

TEST(GridPlanner, FailsWhenFullyBlocked) {
    CostGrid grid(Info());
    AddWall(&grid, 4.0);
    grid.Inflate(0.2, 0.4);
    Path2D path;
    std::string err;
    EXPECT_FALSE(mini_auto::PlanPath(grid, 0.5, 0.5, 3.5, 0.5, 3.0, 0.2, &path, &err));
}

TEST(GridPlanner, StartInsideInflationIsMovedOut) {
    CostGrid grid(Info());
    AddWall(&grid, 3.0);
    grid.Inflate(0.2, 0.4);
    Path2D path;
    std::string err;
    // 壁から 0.1 m (robot_radius 内) から出発しても計画できる
    ASSERT_TRUE(mini_auto::PlanPath(grid, 1.9, 1.0, 0.5, 1.0, 3.0, 0.2, &path, &err)) << err;
}
