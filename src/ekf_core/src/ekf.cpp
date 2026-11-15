#include "ekf_core/ekf.hpp"

#include <cmath>

namespace ekf_core {

double normalizeAngle(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a <= 0.0) a += 2.0 * M_PI;
    return a - M_PI;
}

Mat3 identity3() { return diag3(1.0, 1.0, 1.0); }

Mat3 diag3(double a, double b, double c) {
    Mat3 m{};
    m[0][0] = a;
    m[1][1] = b;
    m[2][2] = c;
    return m;
}

Mat3 multiply(const Mat3& a, const Mat3& b) {
    Mat3 m{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) m[i][j] += a[i][k] * b[k][j];
    return m;
}

Mat3 transpose(const Mat3& a) {
    Mat3 m{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m[i][j] = a[j][i];
    return m;
}

Mat3 add(const Mat3& a, const Mat3& b) {
    Mat3 m{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m[i][j] = a[i][j] + b[i][j];
    return m;
}

bool invert(const Mat3& a, Mat3& out) {
    // Inverse via the adjugate (cofactor) matrix.
    const double c00 = a[1][1] * a[2][2] - a[1][2] * a[2][1];
    const double c01 = a[1][2] * a[2][0] - a[1][0] * a[2][2];
    const double c02 = a[1][0] * a[2][1] - a[1][1] * a[2][0];
    const double det = a[0][0] * c00 + a[0][1] * c01 + a[0][2] * c02;
    if (std::abs(det) < 1e-18) return false;
    const double inv = 1.0 / det;
    out[0][0] = c00 * inv;
    out[1][0] = c01 * inv;
    out[2][0] = c02 * inv;
    out[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv;
    out[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv;
    out[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv;
    out[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv;
    out[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv;
    out[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv;
    return true;
}

DiffDriveEkf::DiffDriveEkf(const Pose2D& initial, const Mat3& initial_cov, const EkfParams& params)
    : pose_(initial), cov_(initial_cov), params_(params) {
    pose_.theta = normalizeAngle(pose_.theta);
}

void DiffDriveEkf::predict(double v, double w, double dt) {
    if (dt <= 0.0) return;
    const double th = pose_.theta;
    const double dth = w * dt;
    Mat3 f = identity3();
    // V = d(state)/d(v, w), stored as the first two columns of a 3x3 matrix.
    Mat3 vj{};
    if (std::abs(w) < 1e-6) {
        // Straight-line limit of the arc model.
        const double c = std::cos(th), s = std::sin(th);
        pose_.x += v * dt * c;
        pose_.y += v * dt * s;
        f[0][2] = -v * dt * s;
        f[1][2] = v * dt * c;
        vj[0][0] = dt * c;
        vj[1][0] = dt * s;
        vj[0][1] = -0.5 * v * dt * dt * s;
        vj[1][1] = 0.5 * v * dt * dt * c;
        vj[2][1] = dt;
    } else {
        // Exact arc of radius v/w.
        const double r = v / w;
        const double s0 = std::sin(th), c0 = std::cos(th);
        const double s1 = std::sin(th + dth), c1 = std::cos(th + dth);
        pose_.x += r * (s1 - s0);
        pose_.y += r * (c0 - c1);
        f[0][2] = r * (c1 - c0);
        f[1][2] = r * (s1 - s0);
        vj[0][0] = (s1 - s0) / w;
        vj[1][0] = (c0 - c1) / w;
        vj[0][1] = -v * (s1 - s0) / (w * w) + r * c1 * dt;
        vj[1][1] = -v * (c0 - c1) / (w * w) + r * s1 * dt;
        vj[2][1] = dt;
    }
    pose_.theta = normalizeAngle(th + dth);

    // V already carries a factor dt, so M = sigma^2 / dt yields variance
    // sigma^2 * dt (see EkfParams).
    const Mat3 m =
        diag3(params_.sigma_v * params_.sigma_v / dt, params_.sigma_w * params_.sigma_w / dt, 0.0);
    const Mat3 q = multiply(multiply(vj, m), transpose(vj));
    cov_ = add(multiply(multiply(f, cov_), transpose(f)), q);
    symmetrise();
}

bool DiffDriveEkf::updateYaw(double yaw, double sigma) {
    const double innov = normalizeAngle(yaw - pose_.theta);
    const double s = cov_[2][2] + sigma * sigma;
    last_d2_ = innov * innov / s;
    if (last_d2_ > params_.yaw_gate) return false;

    // K = P H^T / S with H = [0 0 1].
    Mat3 k{};
    for (int i = 0; i < 3; ++i) k[i][2] = cov_[i][2] / s;
    pose_.x += k[0][2] * innov;
    pose_.y += k[1][2] * innov;
    pose_.theta = normalizeAngle(pose_.theta + k[2][2] * innov);

    josephUpdate(k, diag3(0.0, 0.0, 1.0), diag3(0.0, 0.0, sigma * sigma));
    return true;
}

bool DiffDriveEkf::updatePose(const Pose2D& z, const Mat3& r) {
    const Vec3 innov{z.x - pose_.x, z.y - pose_.y, normalizeAngle(z.theta - pose_.theta)};
    Mat3 s_inv;
    if (!invert(add(cov_, r), s_inv)) return false;
    last_d2_ = 0.0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) last_d2_ += innov[i] * s_inv[i][j] * innov[j];
    if (last_d2_ > params_.pose_gate) return false;

    const Mat3 k = multiply(cov_, s_inv);  // H = I
    Vec3 dx{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) dx[i] += k[i][j] * innov[j];
    pose_.x += dx[0];
    pose_.y += dx[1];
    pose_.theta = normalizeAngle(pose_.theta + dx[2]);

    josephUpdate(k, identity3(), r);
    return true;
}

void DiffDriveEkf::josephUpdate(const Mat3& k, const Mat3& h, const Mat3& r) {
    // P = (I - K H) P (I - K H)^T + K R K^T
    Mat3 ikh = identity3();
    const Mat3 kh = multiply(k, h);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) ikh[i][j] -= kh[i][j];
    cov_ =
        add(multiply(multiply(ikh, cov_), transpose(ikh)), multiply(multiply(k, r), transpose(k)));
    symmetrise();
}

void DiffDriveEkf::symmetrise() {
    for (int i = 0; i < 3; ++i)
        for (int j = i + 1; j < 3; ++j) {
            const double m = 0.5 * (cov_[i][j] + cov_[j][i]);
            cov_[i][j] = cov_[j][i] = m;
        }
}

}  // namespace ekf_core
