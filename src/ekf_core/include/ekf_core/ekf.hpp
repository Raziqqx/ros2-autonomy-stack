// Extended Kalman filter for a planar differential-drive robot.
//
// State x = [px, py, theta] in the map frame. The prediction uses wheel
// odometry velocities (v, w) as the control input; corrections come from an
// absolute yaw measurement (IMU) and from a full planar pose measurement
// (LiDAR scan matching). No ROS types, so it is unit-testable anywhere.
#pragma once

#include <array>

namespace ekf_core {

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

// Wraps an angle to (-pi, pi].
double normalizeAngle(double a);

// Small dense 3x3 helpers (the filter is 3-dimensional, so a linear-algebra
// dependency would cost more than it saves).
Mat3 identity3();
Mat3 diag3(double a, double b, double c);
Mat3 multiply(const Mat3& a, const Mat3& b);
Mat3 transpose(const Mat3& a);
Mat3 add(const Mat3& a, const Mat3& b);
// Returns false when |det| is too small to invert reliably.
bool invert(const Mat3& a, Mat3& out);

struct EkfParams {
    // Odometry noise densities (m/sqrt(s) and rad/sqrt(s)): integrated over a
    // step of length dt, the travelled distance and heading change get variances
    // sigma_v^2 * dt and sigma_w^2 * dt, so the covariance growth does not
    // depend on how often odometry messages arrive.
    double sigma_v = 0.05;
    double sigma_w = 0.05;
    // Squared Mahalanobis gates (chi-square, 99.9 %): 1 DOF and 3 DOF.
    double yaw_gate = 10.83;
    double pose_gate = 16.27;
};

class DiffDriveEkf {
public:
    DiffDriveEkf(const Pose2D& initial, const Mat3& initial_cov, const EkfParams& params = {});

    // Propagates the state with constant (v, w) over dt seconds using the exact
    // arc (straight line when w is ~0) and linearised covariance propagation.
    void predict(double v, double w, double dt);

    // Absolute yaw measurement with standard deviation sigma (rad). Returns
    // false (and leaves the state untouched) if the innovation fails the gate.
    bool updateYaw(double yaw, double sigma);

    // Full pose measurement z with covariance r (e.g. from ICP). Gated likewise.
    bool updatePose(const Pose2D& z, const Mat3& r);

    const Pose2D& pose() const { return pose_; }
    const Mat3& covariance() const { return cov_; }
    // Squared Mahalanobis distance of the last update's innovation.
    double lastMahalanobis2() const { return last_d2_; }

private:
    // Joseph-form covariance update, P = (I-KH) P (I-KH)^T + K R K^T, which
    // keeps P symmetric positive semi-definite despite rounding.
    void josephUpdate(const Mat3& k, const Mat3& h, const Mat3& r);
    void symmetrise();

    Pose2D pose_;
    Mat3 cov_;
    EkfParams params_;
    double last_d2_ = 0.0;
};

}  // namespace ekf_core
