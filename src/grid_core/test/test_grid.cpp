#include <gtest/gtest.h>
#include <algorithm>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>

#include "grid_core/grid.hpp"

using namespace grid_core;

TEST(Geometry, WorldCellRoundTrip) {
    const GridGeometry g{100, 50, 0.1, -2.0, -1.0};
    EXPECT_EQ(g.worldToCell(-2.0, -1.0), (Cell{0, 0}));
    EXPECT_EQ(g.worldToCell(-1.95, -0.85), (Cell{0, 1}));
    EXPECT_EQ(g.worldToCell(-2.01, 0.0), (Cell{-1, 10}));
    double wx, wy;
    g.cellToWorld({3, 4}, wx, wy);
    EXPECT_NEAR(wx, -1.65, 1e-12);
    EXPECT_NEAR(wy, -0.55, 1e-12);
    EXPECT_FALSE(g.inBounds({100, 0}));
}

TEST(Bresenham, KnownLines) {
    EXPECT_EQ(bresenham({0, 0}, {3, 0}), (std::vector<Cell>{{0, 0}, {1, 0}, {2, 0}, {3, 0}}));
    EXPECT_EQ(bresenham({0, 0}, {-2, -2}), (std::vector<Cell>{{0, 0}, {-1, -1}, {-2, -2}}));
    EXPECT_EQ(bresenham({0, 0}, {5, 2}),
              (std::vector<Cell>{{0, 0}, {1, 0}, {2, 1}, {3, 1}, {4, 2}, {5, 2}}));
    EXPECT_EQ(bresenham({1, 1}, {1, 1}), (std::vector<Cell>{{1, 1}}));
}

// Property over random segments in every octant: endpoints included, cells are
// 8-connected, the count is max(|dx|, |dy|) + 1, and no cell is farther than
// half a cell (in the minor axis) from the ideal line.
TEST(Bresenham, RandomSegmentsAreConnectedAndTight) {
    std::mt19937 rng(1);
    std::uniform_int_distribution<int> u(-40, 40);
    for (int k = 0; k < 2000; ++k) {
        const Cell a{u(rng), u(rng)}, b{u(rng), u(rng)};
        const auto line = bresenham(a, b);
        ASSERT_EQ(line.front(), a);
        ASSERT_EQ(line.back(), b);
        ASSERT_EQ(line.size(),
                  static_cast<std::size_t>(std::max(std::abs(b.x - a.x), std::abs(b.y - a.y))) + 1);
        const double len = std::hypot(b.x - a.x, b.y - a.y);
        for (std::size_t i = 0; i < line.size(); ++i) {
            if (i > 0) {
                ASSERT_LE(std::abs(line[i].x - line[i - 1].x), 1);
                ASSERT_LE(std::abs(line[i].y - line[i - 1].y), 1);
            }
            if (len > 0) {
                // Perpendicular distance to the ideal line, scaled to the minor axis.
                const double cross =
                    std::abs((b.x - a.x) * (line[i].y - a.y) - (b.y - a.y) * (line[i].x - a.x)) /
                    len;
                const double major = std::max(std::abs(b.x - a.x), std::abs(b.y - a.y));
                ASSERT_LE(cross * major / len, 0.5 + 1e-9);
            }
        }
    }
}

TEST(LogOddsGrid, RayMarksFreeThenOccupied) {
    LogOddsGrid grid({20, 20, 1.0, 0.0, 0.0});
    grid.insertRay(0.5, 0.5, 5.5, 0.5, true);
    for (int x = 0; x < 5; ++x) EXPECT_LT(grid.probability({x, 0}), 0.5) << x;
    EXPECT_GT(grid.probability({5, 0}), 0.5);
    EXPECT_EQ(grid.logOdds({6, 0}), 0.0);  // beyond the hit: untouched
    EXPECT_EQ(grid.countOccupied(), 1u);
    EXPECT_EQ(grid.countFree(), 0u);  // one free update (p=0.40) is not yet "free"
}

TEST(LogOddsGrid, MaxRangeBeamDoesNotMarkObstacle) {
    LogOddsGrid grid({20, 20, 1.0, 0.0, 0.0});
    grid.insertRay(0.5, 0.5, 5.5, 0.5, false);
    EXPECT_LT(grid.probability({5, 0}), 0.5);
    EXPECT_EQ(grid.countOccupied(), 0u);
}

TEST(LogOddsGrid, ClampsAndExportsRosConvention) {
    LogOddsParams p;
    LogOddsGrid grid({4, 1, 1.0, 0.0, 0.0}, p);
    for (int i = 0; i < 100; ++i) grid.insertRay(0.5, 0.5, 2.5, 0.5, true);
    EXPECT_DOUBLE_EQ(grid.logOdds({2, 0}), p.l_max);
    EXPECT_DOUBLE_EQ(grid.logOdds({0, 0}), p.l_min);
    const auto occ = grid.toOccupancy();
    EXPECT_EQ(occ[2], 98);  // 1 - 1/(1+e^4) = 0.982
    EXPECT_EQ(occ[0], 2);
    EXPECT_EQ(occ[3], -1);  // never observed
}

TEST(LogOddsGrid, ScanOfSquareRoomFindsWalls) {
    // Synthetic 360-beam scan from the centre of a 4 m x 4 m room.
    StaticMap room;
    room.geometry = {100, 100, 0.05, -2.5, -2.5};
    room.occupied.assign(100 * 100, 0);
    for (int i = 0; i < 100; ++i)
        for (int j = 0; j < 100; ++j)
            if (i < 10 || j < 10 || i >= 90 || j >= 90)
                room.occupied[room.geometry.index({i, j})] = 1;
    std::vector<float> ranges(360);
    for (int i = 0; i < 360; ++i)
        ranges[i] = static_cast<float>(raycast(room, 0.0, 0.0, i * M_PI / 180.0, 10.0));
    LogOddsGrid grid(room.geometry);
    grid.insertScan(0.0, 0.0, 0.0, ranges, 0.0, M_PI / 180.0, 0.1, 10.0);
    // Every cell marked occupied lies on the wall surface (the endpoint of a
    // beam is exactly on a cell boundary, so either neighbouring cell is valid).
    std::size_t on_wall = 0;
    for (int i = 0; i < 100; ++i)
        for (int j = 0; j < 100; ++j)
            if (grid.probability({i, j}) > 0.65) {
                const auto near = [](int v, int edge) { return v == edge || v == edge + 1; };
                EXPECT_TRUE(near(i, 9) || near(j, 9) || near(i, 89) || near(j, 89))
                    << i << "," << j;
                ++on_wall;
            }
    EXPECT_GT(on_wall, 200u);
    EXPECT_LT(grid.probability({50, 50}), 0.5);
}

TEST(Raycast, KnownDistances) {
    StaticMap m;
    m.geometry = {100, 100, 0.1, 0.0, 0.0};
    m.occupied.assign(100 * 100, 0);
    for (int y = 0; y < 100; ++y) m.occupied[m.geometry.index({80, y})] = 1;  // wall at x = 8.0
    EXPECT_NEAR(raycast(m, 3.05, 5.0, 0.0, 20.0), 4.95, 1e-9);
    EXPECT_NEAR(raycast(m, 3.0, 5.0, std::atan2(1.0, 1.0), 20.0), 5.0 * std::sqrt(2.0), 1e-9);
    EXPECT_DOUBLE_EQ(raycast(m, 3.0, 5.0, 0.0, 2.0), 2.0);     // max range
    EXPECT_NEAR(raycast(m, 3.0, 5.0, M_PI, 20.0), 3.0, 1e-9);  // map edge counts as occupied
}

TEST(StaticMapHelpers, DiscCollisionAndSurfacePoints) {
    StaticMap m;
    m.geometry = {10, 10, 1.0, 0.0, 0.0};
    m.occupied.assign(100, 0);
    m.occupied[m.geometry.index({5, 5})] = 1;  // the unit square [5,6] x [5,6]
    EXPECT_TRUE(discCollides(m, 4.6, 5.5, 0.5));
    EXPECT_FALSE(discCollides(m, 4.4, 5.5, 0.5));
    EXPECT_FALSE(discCollides(m, 4.6, 4.6, 0.5));  // corner distance 0.566
    EXPECT_TRUE(discCollides(m, 0.3, 5.0, 0.5));   // leaves the map
    const auto pts = surfacePoints(m);
    ASSERT_EQ(pts.size(), 4u);  // one per face of the isolated cell
    for (const auto& p : pts) {
        const bool on_vertical_face = (p.x == 5.0 || p.x == 6.0) && p.y == 5.5;
        const bool on_horizontal_face = (p.y == 5.0 || p.y == 6.0) && p.x == 5.5;
        EXPECT_TRUE(on_vertical_face || on_horizontal_face) << p.x << "," << p.y;
    }
}

class MapIoTest : public ::testing::Test {
protected:
    void SetUp() override {
        dir_ =
            std::filesystem::temp_directory_path() /
            ("grid_core_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
             "_" + ::testing::UnitTest::GetInstance()->current_test_info()->name());
        std::filesystem::create_directories(dir_);
    }
    void TearDown() override { std::filesystem::remove_all(dir_); }
    std::string write(const std::string& name, const std::string& content) {
        std::ofstream(dir_ / name, std::ios::binary) << content;
        return (dir_ / name).string();
    }
    std::filesystem::path dir_;
};

TEST_F(MapIoTest, LoadsBinaryPgmWithFlippedRows) {
    // 3x2 image: top row black (occupied), bottom row white (free); middle
    // pixel of the bottom row is grey (unknown -> blocked).
    std::string pgm = "P5\n# comment\n3 2\n255\n";
    pgm += std::string("\x00\x00\x00", 3);
    pgm += std::string("\xfe\xcd\xfe", 3);
    write("m.pgm", pgm);
    const auto yaml = write("m.yaml",
                            "image: m.pgm\nresolution: 0.5\norigin: [-1.0, 2.0, 0.0]\n"
                            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n");
    std::string err;
    const auto map = loadMap(yaml, &err);
    ASSERT_TRUE(map) << err;
    EXPECT_EQ(map->geometry.width, 3);
    EXPECT_EQ(map->geometry.height, 2);
    EXPECT_DOUBLE_EQ(map->geometry.origin_x, -1.0);
    EXPECT_DOUBLE_EQ(map->geometry.origin_y, 2.0);
    EXPECT_FALSE(map->isOccupied({0, 0}));  // bottom-left: white
    EXPECT_TRUE(map->isOccupied({1, 0}));   // grey: unknown
    EXPECT_TRUE(map->isOccupied({0, 1}));   // top row: black
}

TEST_F(MapIoTest, LoadsAsciiPgm) {
    write("a.pgm", "P2\n2 1\n255\n255 0\n");
    const auto map = loadMap(write("a.yaml", "image: a.pgm\nresolution: 1\norigin: [0, 0, 0]\n"));
    ASSERT_TRUE(map);
    EXPECT_FALSE(map->isOccupied({0, 0}));
    EXPECT_TRUE(map->isOccupied({1, 0}));
}

TEST_F(MapIoTest, ReportsErrors) {
    std::string err;
    EXPECT_FALSE(loadMap((dir_ / "missing.yaml").string(), &err));
    EXPECT_NE(err.find("cannot open"), std::string::npos);
    EXPECT_FALSE(loadMap(write("b.yaml", "resolution: 1\n"), &err));
    write("t.pgm", "P5\n4 4\n255\n\x01");
    EXPECT_FALSE(loadMap(write("t.yaml", "image: t.pgm\nresolution: 1\n"), &err));
    EXPECT_EQ(err, "truncated PGM");
}
