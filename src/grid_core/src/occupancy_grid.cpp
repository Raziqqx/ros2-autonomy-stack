#include <algorithm>
#include <cmath>
#include <limits>

#include "grid_core/grid.hpp"

namespace grid_core {

Cell GridGeometry::worldToCell(double wx, double wy) const {
    return {static_cast<int>(std::floor((wx - origin_x) / resolution)),
            static_cast<int>(std::floor((wy - origin_y) / resolution))};
}

void GridGeometry::cellToWorld(const Cell& c, double& wx, double& wy) const {
    wx = origin_x + (c.x + 0.5) * resolution;
    wy = origin_y + (c.y + 0.5) * resolution;
}

LogOddsGrid::LogOddsGrid(const GridGeometry& geometry, const LogOddsParams& params)
    : geom_(geometry),
      params_(params),
      cells_(static_cast<std::size_t>(geometry.width) * static_cast<std::size_t>(geometry.height),
             0.0) {}

void LogOddsGrid::addLogOdds(const Cell& c, double delta) {
    double& l = cells_[geom_.index(c)];
    l = std::clamp(l + delta, params_.l_min, params_.l_max);
}

void LogOddsGrid::insertRay(double ox, double oy, double ex, double ey, bool hit) {
    const Cell a = geom_.worldToCell(ox, oy), b = geom_.worldToCell(ex, ey);
    const std::vector<Cell> line = bresenham(a, b);
    // Every traversed cell except the endpoint is evidence of free space.
    for (std::size_t i = 0; i + 1 < line.size(); ++i)
        if (geom_.inBounds(line[i])) addLogOdds(line[i], params_.l_free);
    if (geom_.inBounds(b)) addLogOdds(b, hit ? params_.l_occ : params_.l_free);
}

void LogOddsGrid::insertScan(double x, double y, double theta, const std::vector<float>& ranges,
                             double angle_min, double angle_increment, double range_min,
                             double range_max) {
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        double r = ranges[i];
        if (std::isnan(r) || r < range_min) continue;
        const bool hit = std::isfinite(r) && r < range_max;
        if (!hit) r = range_max;
        const double a = theta + angle_min + angle_increment * static_cast<double>(i);
        insertRay(x, y, x + r * std::cos(a), y + r * std::sin(a), hit);
    }
}

double LogOddsGrid::probability(const Cell& c) const {
    return 1.0 - 1.0 / (1.0 + std::exp(logOdds(c)));
}

std::vector<int8_t> LogOddsGrid::toOccupancy() const {
    std::vector<int8_t> out(cells_.size());
    for (std::size_t i = 0; i < cells_.size(); ++i)
        out[i] = cells_[i] == 0.0 ? int8_t{-1}  // never observed
                                  : static_cast<int8_t>(std::lround(
                                        100.0 * (1.0 - 1.0 / (1.0 + std::exp(cells_[i])))));
    return out;
}

std::size_t LogOddsGrid::countOccupied() const {
    const double l = std::log(params_.occupied_prob / (1.0 - params_.occupied_prob));
    return static_cast<std::size_t>(
        std::count_if(cells_.begin(), cells_.end(), [l](double v) { return v > l; }));
}

std::size_t LogOddsGrid::countFree() const {
    const double l = std::log(params_.free_prob / (1.0 - params_.free_prob));
    return static_cast<std::size_t>(
        std::count_if(cells_.begin(), cells_.end(), [l](double v) { return v < l; }));
}

double raycast(const StaticMap& map, double x, double y, double angle, double max_range) {
    const GridGeometry& g = map.geometry;
    const double dx = std::cos(angle), dy = std::sin(angle);
    Cell c = g.worldToCell(x, y);
    if (map.isOccupied(c)) return 0.0;
    // Amanatides & Woo: advance to whichever cell boundary (x or y) is nearer.
    const int step_x = dx > 0 ? 1 : -1, step_y = dy > 0 ? 1 : -1;
    const double inf = std::numeric_limits<double>::infinity();
    const double next_x = g.origin_x + (c.x + (step_x > 0 ? 1 : 0)) * g.resolution;
    const double next_y = g.origin_y + (c.y + (step_y > 0 ? 1 : 0)) * g.resolution;
    double t_max_x = dx != 0.0 ? (next_x - x) / dx : inf;
    double t_max_y = dy != 0.0 ? (next_y - y) / dy : inf;
    const double t_delta_x = dx != 0.0 ? g.resolution / std::abs(dx) : inf;
    const double t_delta_y = dy != 0.0 ? g.resolution / std::abs(dy) : inf;
    while (true) {
        double t;
        if (t_max_x < t_max_y) {
            t = t_max_x;
            t_max_x += t_delta_x;
            c.x += step_x;
        } else {
            t = t_max_y;
            t_max_y += t_delta_y;
            c.y += step_y;
        }
        if (t >= max_range) return max_range;
        if (map.isOccupied(c)) return t;
    }
}

bool discCollides(const StaticMap& map, double x, double y, double radius) {
    const GridGeometry& g = map.geometry;
    const Cell lo = g.worldToCell(x - radius, y - radius),
               hi = g.worldToCell(x + radius, y + radius);
    for (int cy = lo.y; cy <= hi.y; ++cy)
        for (int cx = lo.x; cx <= hi.x; ++cx) {
            if (!map.isOccupied({cx, cy})) continue;
            // Distance from the disc centre to the closest point of the cell.
            const double x0 = g.origin_x + cx * g.resolution, y0 = g.origin_y + cy * g.resolution;
            const double dx = std::max({x0 - x, 0.0, x - (x0 + g.resolution)});
            const double dy = std::max({y0 - y, 0.0, y - (y0 + g.resolution)});
            if (dx * dx + dy * dy < radius * radius) return true;
        }
    return false;
}

std::vector<Point2d> surfacePoints(const StaticMap& map) {
    const GridGeometry& g = map.geometry;
    std::vector<Point2d> pts;
    constexpr int kDx[4] = {1, -1, 0, 0}, kDy[4] = {0, 0, 1, -1};
    for (int cy = 0; cy < g.height; ++cy)
        for (int cx = 0; cx < g.width; ++cx) {
            if (!map.isOccupied({cx, cy})) continue;
            double wx, wy;
            g.cellToWorld({cx, cy}, wx, wy);
            for (int k = 0; k < 4; ++k) {
                const Cell n{cx + kDx[k], cy + kDy[k]};
                if (g.inBounds(n) && !map.isOccupied(n))
                    pts.push_back(
                        {wx + 0.5 * kDx[k] * g.resolution, wy + 0.5 * kDy[k] * g.resolution});
            }
        }
    return pts;
}

}  // namespace grid_core
