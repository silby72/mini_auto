#include "mini_auto/grid_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace mini_auto {

CostGrid::CostGrid(const GridInfo &info)
    : info_(info),
      occupied_(static_cast<size_t>(std::max(info.width, 0)) * std::max(info.height, 0), 0),
      cost_(occupied_.size(), 0) {}

bool CostGrid::InBounds(int cx, int cy) const {
    return cx >= 0 && cy >= 0 && cx < info_.width && cy < info_.height;
}

bool CostGrid::WorldToCell(double x, double y, int *cx, int *cy) const {
    *cx = static_cast<int>(std::floor((x - info_.origin_x) / info_.resolution));
    *cy = static_cast<int>(std::floor((y - info_.origin_y) / info_.resolution));
    return InBounds(*cx, *cy);
}

void CostGrid::CellToWorld(int cx, int cy, double *x, double *y) const {
    *x = info_.origin_x + (cx + 0.5) * info_.resolution;
    *y = info_.origin_y + (cy + 0.5) * info_.resolution;
}

void CostGrid::MarkObstacle(double x, double y) {
    int cx, cy;
    if (WorldToCell(x, y, &cx, &cy)) {
        MarkObstacleCell(cx, cy);
    }
}

void CostGrid::MarkObstacleCell(int cx, int cy) {
    if (InBounds(cx, cy)) {
        occupied_[Index(cx, cy)] = 1;
    }
}

void CostGrid::Inflate(double robot_radius, double inflation_radius) {
    inflation_radius = std::max(inflation_radius, robot_radius);
    const int r_cells = static_cast<int>(std::ceil(inflation_radius / info_.resolution));

    // 膨張カーネル(オフセット → コスト)を先に作っておく
    struct Offset {
        int dx, dy;
        uint8_t cost;
    };
    std::vector<Offset> kernel;
    for (int dy = -r_cells; dy <= r_cells; ++dy) {
        for (int dx = -r_cells; dx <= r_cells; ++dx) {
            const double d = std::hypot(dx, dy) * info_.resolution;
            if (d > inflation_radius) {
                continue;
            }
            uint8_t c = kLethal;
            if (d > robot_radius) {
                const double ratio = (d - robot_radius) / (inflation_radius - robot_radius);
                c = static_cast<uint8_t>(std::max(1.0, (kLethal - 1) * (1.0 - ratio)));
            }
            kernel.push_back({dx, dy, c});
        }
    }

    std::fill(cost_.begin(), cost_.end(), 0);
    for (int cy = 0; cy < info_.height; ++cy) {
        for (int cx = 0; cx < info_.width; ++cx) {
            if (!occupied_[Index(cx, cy)]) {
                continue;
            }
            for (const auto &k : kernel) {
                const int x = cx + k.dx;
                const int y = cy + k.dy;
                if (InBounds(x, y)) {
                    uint8_t &dst = cost_[Index(x, y)];
                    dst = std::max(dst, k.cost);
                }
            }
        }
    }
}

namespace {

// (cx, cy) から search_radius 以内で最も近い非Lethalセルを探す
bool NearestFree(const CostGrid &grid, int cx, int cy, double search_radius, int *ox, int *oy) {
    if (grid.InBounds(cx, cy) && grid.Cost(cx, cy) < CostGrid::kLethal) {
        *ox = cx;
        *oy = cy;
        return true;
    }
    const int r = static_cast<int>(std::ceil(search_radius / grid.info().resolution));
    double best = std::numeric_limits<double>::infinity();
    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            const int x = cx + dx;
            const int y = cy + dy;
            const double d = std::hypot(dx, dy);
            if (d > r || d >= best || !grid.InBounds(x, y) || grid.Cost(x, y) >= CostGrid::kLethal) {
                continue;
            }
            best = d;
            *ox = x;
            *oy = y;
        }
    }
    return std::isfinite(best);
}

}  // namespace

bool PlanPath(const CostGrid &grid, double sx, double sy, double gx, double gy,
              double cost_weight, double search_radius, Path2D *path, std::string *error) {
    path->clear();
    const GridInfo &info = grid.info();

    int scx, scy, gcx, gcy;
    grid.WorldToCell(sx, sy, &scx, &scy);
    grid.WorldToCell(gx, gy, &gcx, &gcy);
    if (!NearestFree(grid, scx, scy, search_radius, &scx, &scy)) {
        *error = "start is inside an obstacle";
        return false;
    }
    if (!NearestFree(grid, gcx, gcy, search_radius, &gcx, &gcy)) {
        *error = "goal is inside an obstacle (or outside the grid)";
        return false;
    }

    const size_t n = static_cast<size_t>(info.width) * info.height;
    auto index = [&](int x, int y) { return static_cast<size_t>(y) * info.width + x; };
    // 8近傍のオクタイル距離 (セル単位)。コスト倍率は >= 1 なので許容的
    auto heuristic = [&](int x, int y) {
        const int dx = std::abs(x - gcx);
        const int dy = std::abs(y - gcy);
        return (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
    };

    std::vector<double> g(n, std::numeric_limits<double>::infinity());
    std::vector<int64_t> parent(n, -1);
    std::vector<uint8_t> closed(n, 0);
    using Entry = std::pair<double, size_t>;  // (f, index)
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

    const size_t start = index(scx, scy);
    const size_t goal = index(gcx, gcy);
    g[start] = 0.0;
    open.push({heuristic(scx, scy), start});

    constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!open.empty()) {
        const size_t cur = open.top().second;
        open.pop();
        if (closed[cur]) {
            continue;
        }
        closed[cur] = 1;
        if (cur == goal) {
            break;
        }
        const int cx = static_cast<int>(cur % info.width);
        const int cy = static_cast<int>(cur / info.width);
        for (int k = 0; k < 8; ++k) {
            const int nx = cx + kDx[k];
            const int ny = cy + kDy[k];
            if (!grid.InBounds(nx, ny)) {
                continue;
            }
            const uint8_t c = grid.Cost(nx, ny);
            if (c >= CostGrid::kLethal) {
                continue;
            }
            // 斜め移動で角をすり抜けないようにする
            if (k >= 4 && (grid.Cost(cx + kDx[k], cy) >= CostGrid::kLethal ||
                           grid.Cost(cx, cy + kDy[k]) >= CostGrid::kLethal)) {
                continue;
            }
            const size_t ni = index(nx, ny);
            const double step = (k < 4 ? 1.0 : std::sqrt(2.0)) *
                                (1.0 + cost_weight * c / static_cast<double>(CostGrid::kLethal));
            if (g[cur] + step < g[ni]) {
                g[ni] = g[cur] + step;
                parent[ni] = static_cast<int64_t>(cur);
                open.push({g[ni] + heuristic(nx, ny), ni});
            }
        }
    }

    if (!closed[goal]) {
        *error = "no path found";
        return false;
    }

    for (int64_t i = static_cast<int64_t>(goal); i >= 0; i = parent[i]) {
        double x, y;
        grid.CellToWorld(static_cast<int>(i % info.width), static_cast<int>(i / info.width), &x, &y);
        path->emplace_back(x, y);
    }
    std::reverse(path->begin(), path->end());
    // 終点はセル中心ではなく指定ゴールそのものにする(ゴールが通行可能な場合)
    int ccx, ccy;
    if (grid.WorldToCell(gx, gy, &ccx, &ccy) && ccx == gcx && ccy == gcy) {
        path->back() = {gx, gy};
    }
    return true;
}

}  // namespace mini_auto
