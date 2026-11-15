#include "icp_core/icp.hpp"

#include <cmath>
#include <limits>

namespace icp_core {

Point2 Transform2D::apply(const Point2& p) const {
    const double c = std::cos(theta), s = std::sin(theta);
    return {c * p.x - s * p.y + x, s * p.x + c * p.y + y};
}

Transform2D Transform2D::compose(const Transform2D& other) const {
    const Point2 t = apply({other.x, other.y});
    return {t.x, t.y, std::remainder(theta + other.theta, 2.0 * M_PI)};
}

Transform2D bestFitTransform(const std::vector<Point2>& src, const std::vector<Point2>& dst) {
    const double n = static_cast<double>(src.size());
    Point2 cs, cd;
    for (std::size_t i = 0; i < src.size(); ++i) {
        cs.x += src[i].x;
        cs.y += src[i].y;
        cd.x += dst[i].x;
        cd.y += dst[i].y;
    }
    cs = {cs.x / n, cs.y / n};
    cd = {cd.x / n, cd.y / n};
    // In 2D the optimal rotation has a closed form: theta = atan2(sum of
    // cross products, sum of dot products) of the centred pairs.
    double sxx = 0.0, sxy = 0.0;
    for (std::size_t i = 0; i < src.size(); ++i) {
        const double ax = src[i].x - cs.x, ay = src[i].y - cs.y;
        const double bx = dst[i].x - cd.x, by = dst[i].y - cd.y;
        sxx += ax * bx + ay * by;
        sxy += ax * by - ay * bx;
    }
    const double theta = std::atan2(sxy, sxx);
    const double c = std::cos(theta), s = std::sin(theta);
    return {cd.x - (c * cs.x - s * cs.y), cd.y - (s * cs.x + c * cs.y), theta};
}

PointIndex::PointIndex(std::vector<Point2> points, double cell_size)
    : points_(std::move(points)), cell_(cell_size) {
    for (std::size_t i = 0; i < points_.size(); ++i) {
        const auto ix = static_cast<long long>(std::floor(points_[i].x / cell_));
        const auto iy = static_cast<long long>(std::floor(points_[i].y / cell_));
        buckets_[key(ix, iy)].push_back(i);
    }
}

long PointIndex::nearest(const Point2& q, double max_dist) const {
    const auto ix = static_cast<long long>(std::floor(q.x / cell_));
    const auto iy = static_cast<long long>(std::floor(q.y / cell_));
    // Visit enough rings of buckets to cover max_dist (one ring when
    // max_dist <= cell size, which is how the index is normally built).
    const long long rings = static_cast<long long>(std::ceil(max_dist / cell_));
    long best = -1;
    double best_d2 = max_dist * max_dist;
    for (long long dx = -rings; dx <= rings; ++dx)
        for (long long dy = -rings; dy <= rings; ++dy) {
            const auto it = buckets_.find(key(ix + dx, iy + dy));
            if (it == buckets_.end()) continue;
            for (const std::size_t i : it->second) {
                const double ex = points_[i].x - q.x, ey = points_[i].y - q.y;
                const double d2 = ex * ex + ey * ey;
                if (d2 <= best_d2) {
                    best_d2 = d2;
                    best = static_cast<long>(i);
                }
            }
        }
    return best;
}

void PointIndex::computeNormals(double radius) {
    normals_.assign(points_.size(), Point2{});
    has_normal_.assign(points_.size(), false);
    const long long rings = static_cast<long long>(std::ceil(radius / cell_));
    for (std::size_t i = 0; i < points_.size(); ++i) {
        const Point2& c = points_[i];
        const auto ix = static_cast<long long>(std::floor(c.x / cell_));
        const auto iy = static_cast<long long>(std::floor(c.y / cell_));
        // Covariance of the neighbourhood; the normal is the eigenvector of
        // the smallest eigenvalue (perpendicular to the local wall direction).
        double mx = 0, my = 0, sxx = 0, sxy = 0, syy = 0;
        int n = 0;
        for (long long dx = -rings; dx <= rings; ++dx)
            for (long long dy = -rings; dy <= rings; ++dy) {
                const auto it = buckets_.find(key(ix + dx, iy + dy));
                if (it == buckets_.end()) continue;
                for (const std::size_t j : it->second) {
                    const double ex = points_[j].x - c.x, ey = points_[j].y - c.y;
                    if (ex * ex + ey * ey > radius * radius) continue;
                    mx += points_[j].x;
                    my += points_[j].y;
                    sxx += points_[j].x * points_[j].x;
                    sxy += points_[j].x * points_[j].y;
                    syy += points_[j].y * points_[j].y;
                    ++n;
                }
            }
        if (n < 3) continue;
        mx /= n;
        my /= n;
        const double cxx = sxx / n - mx * mx, cxy = sxy / n - mx * my, cyy = syy / n - my * my;
        // Principal (largest-variance) direction angle of a 2x2 symmetric matrix.
        const double phi = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
        normals_[i] = {-std::sin(phi), std::cos(phi)};
        has_normal_[i] = true;
    }
}

namespace {

// Solves the 3x3 system a * x = b by Cramer's rule; false if near-singular
// (a degenerate geometry such as a single straight wall).
bool solve3(const double a[3][3], const double b[3], double x[3]) {
    auto det3 = [](const double m[3][3]) {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    };
    const double d = det3(a);
    if (std::abs(d) < 1e-12) return false;
    for (int c = 0; c < 3; ++c) {
        double m[3][3];
        for (int r = 0; r < 3; ++r)
            for (int k = 0; k < 3; ++k) m[r][k] = (k == c) ? b[r] : a[r][k];
        x[c] = det3(m) / d;
    }
    return true;
}

}  // namespace

IcpResult align(const std::vector<Point2>& source, const PointIndex& target,
                const Transform2D& initial_guess, const IcpParams& params) {
    IcpResult result;
    result.transform = initial_guess;
    std::vector<Point2> src_matched, dst_matched;
    src_matched.reserve(source.size());
    dst_matched.reserve(source.size());

    // Pairs every source point (under transform t) with its nearest target
    // point; returns the sum of squared inlier distances.
    std::vector<std::size_t> dst_index;
    auto associate = [&](const Transform2D& t) {
        src_matched.clear();
        dst_matched.clear();
        dst_index.clear();
        double sse = 0.0;
        for (const Point2& p : source) {
            const Point2 q = t.apply(p);
            const long j = target.nearest(q, params.max_correspondence_dist);
            if (j < 0) continue;
            const Point2& m = target.points()[static_cast<std::size_t>(j)];
            src_matched.push_back(p);
            dst_matched.push_back(m);
            dst_index.push_back(static_cast<std::size_t>(j));
            sse += (m.x - q.x) * (m.x - q.x) + (m.y - q.y) * (m.y - q.y);
        }
        return sse;
    };

    // One Gauss-Newton step minimising sum (n_i . (T p_i - m_i))^2 with the
    // update linearised as a small rotation about the map origin plus a
    // translation, applied on the left of the current transform.
    auto pointToLineStep = [&](const Transform2D& t, Transform2D* out) {
        double a[3][3] = {}, b[3] = {};
        std::size_t used = 0;
        for (std::size_t k = 0; k < src_matched.size(); ++k) {
            if (!target.hasNormal(dst_index[k])) continue;
            const Point2 q = t.apply(src_matched[k]);
            const Point2& n = target.normal(dst_index[k]);
            const double r = n.x * (q.x - dst_matched[k].x) + n.y * (q.y - dst_matched[k].y);
            const double j[3] = {n.x, n.y, n.y * q.x - n.x * q.y};
            for (int u = 0; u < 3; ++u) {
                b[u] -= j[u] * r;
                for (int v = 0; v < 3; ++v) a[u][v] += j[u] * j[v];
            }
            ++used;
        }
        double d[3];
        if (used < params.min_correspondences || !solve3(a, b, d)) return false;
        *out = Transform2D{d[0], d[1], d[2]}.compose(t);
        return true;
    };

    for (int it = 0; it < params.max_iterations; ++it) {
        result.iterations = it + 1;
        associate(result.transform);
        if (src_matched.size() < params.min_correspondences) return result;  // not converged
        Transform2D next;
        if (params.method == IcpMethod::PointToPoint) {
            // Solving directly for the full transform from the original source
            // points avoids accumulating composition round-off.
            next = bestFitTransform(src_matched, dst_matched);
        } else if (!pointToLineStep(result.transform, &next)) {
            return result;  // degenerate geometry: not converged
        }
        const double dt = std::hypot(next.x - result.transform.x, next.y - result.transform.y);
        const double dr = std::abs(std::remainder(next.theta - result.transform.theta, 2.0 * M_PI));
        result.transform = next;
        if (dt < params.translation_eps && dr < params.rotation_eps) {
            result.converged = true;
            break;
        }
    }

    const double sse = associate(result.transform);
    result.num_correspondences = src_matched.size();
    if (result.num_correspondences < params.min_correspondences) result.converged = false;
    result.rmse = result.num_correspondences
                      ? std::sqrt(sse / static_cast<double>(result.num_correspondences))
                      : std::numeric_limits<double>::infinity();
    result.inlier_ratio = source.empty() ? 0.0
                                         : static_cast<double>(result.num_correspondences) /
                                               static_cast<double>(source.size());
    return result;
}

}  // namespace icp_core
