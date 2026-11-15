// 2D ICP (iterative closest point) scan matching: point-to-point (baseline)
// and point-to-line (stronger: converges in far fewer iterations on walls and
// escapes the "sliding along a wall" fixed points of point-to-point).
//
// Aligns a source point set (e.g. a LiDAR scan in the robot frame) to a target
// point set (e.g. occupied cells of a prior map in the map frame), returning
// the rigid transform T such that T * source ~= target.
#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace icp_core {

struct Point2 {
    double x = 0.0;
    double y = 0.0;
};

struct Transform2D {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    Point2 apply(const Point2& p) const;
    // this * other (apply `other` first).
    Transform2D compose(const Transform2D& other) const;
};

// Closed-form least-squares rigid transform mapping src[i] onto dst[i]
// (2D Kabsch/Umeyama without scale). Requires src.size() == dst.size() >= 2.
Transform2D bestFitTransform(const std::vector<Point2>& src, const std::vector<Point2>& dst);

// Nearest-neighbour index over a fixed point set: a uniform hash grid whose
// cell size equals the maximum query radius, so a query visits 3x3 buckets.
class PointIndex {
public:
    PointIndex(std::vector<Point2> points, double cell_size);
    // Index of the nearest point within `max_dist`, or -1 if none.
    long nearest(const Point2& q, double max_dist) const;
    const std::vector<Point2>& points() const { return points_; }

    // Estimates a unit normal per point by PCA over neighbours within
    // `radius`; points with fewer than 3 neighbours get no normal.
    void computeNormals(double radius);
    bool hasNormal(std::size_t i) const { return i < has_normal_.size() && has_normal_[i]; }
    const Point2& normal(std::size_t i) const { return normals_[i]; }

private:
    static long long key(long long ix, long long iy) { return (ix << 32) ^ (iy & 0xffffffffLL); }
    std::vector<Point2> points_;
    double cell_;
    std::unordered_map<long long, std::vector<std::size_t>> buckets_;
    std::vector<Point2> normals_;
    std::vector<bool> has_normal_;
};

enum class IcpMethod { PointToPoint, PointToLine };

struct IcpParams {
    int max_iterations = 50;
    // Pairs farther apart than this are treated as outliers.
    double max_correspondence_dist = 0.5;
    // Stop when an iteration's increment is smaller than these.
    double translation_eps = 1e-5;
    double rotation_eps = 1e-5;
    std::size_t min_correspondences = 20;
    // PointToLine needs PointIndex::computeNormals() on the target first.
    IcpMethod method = IcpMethod::PointToPoint;
};

struct IcpResult {
    Transform2D transform;
    bool converged = false;
    int iterations = 0;
    // RMS distance of the inlier correspondences at the final transform.
    double rmse = 0.0;
    std::size_t num_correspondences = 0;
    // Fraction of source points that found a correspondence.
    double inlier_ratio = 0.0;
};

IcpResult align(const std::vector<Point2>& source, const PointIndex& target,
                const Transform2D& initial_guess, const IcpParams& params = {});

}  // namespace icp_core
