// Occupancy-grid mapping primitives: grid geometry, Bresenham ray traversal,
// a log-odds occupancy grid, LiDAR raycasting against a binary map and
// ROS-style PGM/YAML map I/O. No ROS dependency.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace grid_core {

struct Cell {
    int x = 0;
    int y = 0;
    bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
};

// Row-major grid of `width` x `height` square cells; cell (0, 0) has its lower
// left corner at (origin_x, origin_y) in the world (map) frame.
struct GridGeometry {
    int width = 0;
    int height = 0;
    double resolution = 0.05;  // metres per cell
    double origin_x = 0.0;
    double origin_y = 0.0;

    bool inBounds(const Cell& c) const {
        return c.x >= 0 && c.y >= 0 && c.x < width && c.y < height;
    }
    std::size_t index(const Cell& c) const {
        return static_cast<std::size_t>(c.y) * static_cast<std::size_t>(width) +
               static_cast<std::size_t>(c.x);
    }
    Cell worldToCell(double wx, double wy) const;
    // Centre of the cell in world coordinates.
    void cellToWorld(const Cell& c, double& wx, double& wy) const;
};

// All cells on the digital line from a to b inclusive (integer Bresenham,
// all octants). Consecutive cells are 8-connected.
std::vector<Cell> bresenham(Cell a, Cell b);

struct LogOddsParams {
    double l_occ = 0.85;   // log-odds increment for a cell hit by a beam
    double l_free = -0.4;  // increment for a cell a beam passed through
    double l_min = -4.0;   // clamping keeps the map able to change its mind
    double l_max = 4.0;
    double occupied_prob = 0.65;  // thresholds for the ternary export
    double free_prob = 0.35;
};

class LogOddsGrid {
public:
    LogOddsGrid(const GridGeometry& geometry, const LogOddsParams& params = {});

    // Integrates one beam from the sensor origin to the endpoint (world
    // frame). Cells strictly between are updated as free; the endpoint cell is
    // updated as occupied only when `hit` is true (not a max-range return).
    void insertRay(double ox, double oy, double ex, double ey, bool hit);

    // Integrates a planar scan taken at sensor pose (x, y, theta). Ranges that
    // are non-finite or below range_min are skipped; ranges >= range_max are
    // cleared up to range_max without marking an obstacle.
    void insertScan(double x, double y, double theta, const std::vector<float>& ranges,
                    double angle_min, double angle_increment, double range_min, double range_max);

    double logOdds(const Cell& c) const { return cells_[geom_.index(c)]; }
    double probability(const Cell& c) const;
    // ROS nav_msgs/OccupancyGrid convention: -1 unknown, else 0..100.
    std::vector<int8_t> toOccupancy() const;
    std::size_t countOccupied() const;
    std::size_t countFree() const;
    const GridGeometry& geometry() const { return geom_; }

private:
    void addLogOdds(const Cell& c, double delta);
    GridGeometry geom_;
    LogOddsParams params_;
    std::vector<double> cells_;
};

// Binary (occupied / free) map, as loaded from a PGM + YAML pair.
struct StaticMap {
    GridGeometry geometry;
    std::vector<uint8_t> occupied;  // 1 = occupied (or unknown, treated as blocked)
    bool isOccupied(const Cell& c) const {
        return !geometry.inBounds(c) || occupied[geometry.index(c)];
    }
};

// Distance from (x, y) along `angle` to the first occupied cell, stepping the
// grid with an Amanatides-Woo DDA; returns max_range if nothing is hit.
double raycast(const StaticMap& map, double x, double y, double angle, double max_range);

// True if a disc of `radius` centred at (x, y) overlaps any occupied cell
// (or leaves the map).
bool discCollides(const StaticMap& map, double x, double y, double radius);

struct Point2d {
    double x = 0.0;
    double y = 0.0;
};

// Points on the obstacle surface: the midpoint of every edge shared by an
// occupied cell and a free 4-neighbour. These lie where a LiDAR beam actually
// hits, unlike occupied cell centres (half a cell inside the wall), so they
// are the right ICP target.
std::vector<Point2d> surfacePoints(const StaticMap& map);

// Loads a ROS map_server style YAML (image, resolution, origin, negate,
// free_thresh) and its binary (P5) or ASCII (P2) PGM image.
// Returns std::nullopt and fills `error` on failure.
std::optional<StaticMap> loadMap(const std::string& yaml_path, std::string* error = nullptr);

}  // namespace grid_core
