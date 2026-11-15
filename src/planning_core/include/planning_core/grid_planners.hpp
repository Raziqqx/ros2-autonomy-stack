// Global path planners on a binary 2D grid: A* (8-connected, baseline) and
// Theta* (any-angle: paths are not restricted to the grid's 45-degree
// headings, so they are shorter and need no post-smoothing). No ROS types.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace planning_core {

struct GridCell {
    int x = 0;
    int y = 0;
    bool operator==(const GridCell& o) const { return x == o.x && y == o.y; }
};

struct PlanningGrid {
    int width = 0;
    int height = 0;
    std::vector<uint8_t> blocked;  // row-major, non-zero = obstacle

    bool inBounds(GridCell c) const { return c.x >= 0 && c.y >= 0 && c.x < width && c.y < height; }
    // Out-of-bounds cells count as blocked.
    bool isBlocked(GridCell c) const {
        return !inBounds(c) || blocked[static_cast<std::size_t>(c.y) * width + c.x] != 0;
    }
};

struct PlanResult {
    bool found = false;
    // Waypoints from start to goal inclusive. A* returns every cell; Theta*
    // returns only the turning points.
    std::vector<GridCell> path;
    double cost = 0.0;  // Euclidean length in cells
    std::size_t expansions = 0;
};

struct AStarParams {
    // 1 = optimal A*; 0 = Dijkstra; > 1 = weighted A* (faster, suboptimal).
    double heuristic_weight = 1.0;
};

// 8-connected A* with octile heuristic. Diagonal moves may not cut a corner:
// both orthogonal neighbours must be free.
PlanResult planAStar(const PlanningGrid& grid, GridCell start, GridCell goal,
                     const AStarParams& params = {});

// Basic Theta* (Nash et al. 2007) with a Euclidean heuristic.
PlanResult planThetaStar(const PlanningGrid& grid, GridCell start, GridCell goal);

// True if the segment between the centres of a and b crosses only free cells
// (supercover traversal; when it passes exactly through a cell corner both
// side cells must be free, matching A*'s no-corner-cutting rule).
bool lineOfSight(const PlanningGrid& grid, GridCell a, GridCell b);

// Marks every cell within `radius` cells (Euclidean) of an obstacle as
// blocked, so planning a point robot equals planning a disc robot.
PlanningGrid inflate(const PlanningGrid& grid, double radius);

double pathLength(const std::vector<GridCell>& path);

}  // namespace planning_core
