#include <algorithm>
#include <cstdlib>

#include "grid_core/grid.hpp"

namespace grid_core {

std::vector<Cell> bresenham(Cell a, Cell b) {
    // Error-accumulating integer form that handles every octant at once.
    std::vector<Cell> cells;
    const int dx = std::abs(b.x - a.x), dy = -std::abs(b.y - a.y);
    const int sx = a.x < b.x ? 1 : -1, sy = a.y < b.y ? 1 : -1;
    cells.reserve(static_cast<std::size_t>(std::max(dx, -dy)) + 1);
    int err = dx + dy;
    Cell c = a;
    while (true) {
        cells.push_back(c);
        if (c == b) break;
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            c.x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            c.y += sy;
        }
    }
    return cells;
}

}  // namespace grid_core
