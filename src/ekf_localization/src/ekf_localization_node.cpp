// ekf_localization: fuses wheel odometry (prediction), IMU yaw (update) and an
// ICP scan-match against the prior map (pose update) in ekf_core's EKF.
//
// Subscribes:  /odom (nav_msgs/Odometry; only the twist is used), /imu, /scan
// Publishes:   /ekf/pose (geometry_msgs/PoseWithCovarianceStamped, map frame)
//              TF map->odom, so map->odom->base_link gives the EKF pose.
#include <ekf_core/ekf.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <grid_core/grid.hpp>
#include <icp_core/icp.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace {

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(0.5 * yaw);
    q.w = std::cos(0.5 * yaw);
    return q;
}

}  // namespace

class EkfLocalizationNode : public rclcpp::Node {
public:
    EkfLocalizationNode() : Node("ekf_localization") {
        const ekf_core::Pose2D initial{declare_parameter("initial_x", 0.0),
                                       declare_parameter("initial_y", 0.0),
                                       declare_parameter("initial_theta", 0.0)};
        const double sxy = declare_parameter("initial_sigma_xy", 0.1);
        const double sth = declare_parameter("initial_sigma_theta", 0.05);
        ekf_core::EkfParams ekf_params;
        ekf_params.sigma_v = declare_parameter("sigma_v", ekf_params.sigma_v);
        ekf_params.sigma_w = declare_parameter("sigma_w", ekf_params.sigma_w);
        ekf_.emplace(initial, ekf_core::diag3(sxy * sxy, sxy * sxy, sth * sth), ekf_params);

        use_imu_ = declare_parameter("use_imu", true);
        imu_yaw_sigma_ = declare_parameter("imu_yaw_sigma", 0.02);
        use_icp_ = declare_parameter("use_icp", true);
        scan_stride_ = std::max<int>(1, declare_parameter("scan_stride", 2));
        icp_params_.method =
            declare_parameter<std::string>("icp_method", "point_to_line") == "point_to_point"
                ? icp_core::IcpMethod::PointToPoint
                : icp_core::IcpMethod::PointToLine;
        icp_params_.max_correspondence_dist = declare_parameter("icp_max_correspondence_dist", 0.3);
        icp_params_.max_iterations = declare_parameter("icp_max_iterations", 30);
        icp_params_.min_correspondences =
            static_cast<std::size_t>(declare_parameter("icp_min_correspondences", 40));
        icp_max_rmse_ = declare_parameter("icp_max_rmse", 0.05);
        icp_min_inlier_ratio_ = declare_parameter("icp_min_inlier_ratio", 0.5);
        const double icp_sxy = declare_parameter("icp_sigma_xy", 0.03);
        const double icp_sth = declare_parameter("icp_sigma_theta", 0.01);
        icp_cov_ = ekf_core::diag3(icp_sxy * icp_sxy, icp_sxy * icp_sxy, icp_sth * icp_sth);

        if (use_icp_) {
            const std::string map_yaml = declare_parameter<std::string>("map_yaml", "");
            std::string err;
            const auto map = grid_core::loadMap(map_yaml, &err);
            if (!map) throw std::runtime_error("ekf_localization: " + err);
            std::vector<icp_core::Point2> pts;
            for (const auto& p : grid_core::surfacePoints(*map)) pts.push_back({p.x, p.y});
            map_index_.emplace(std::move(pts), icp_params_.max_correspondence_dist);
            map_index_->computeNormals(3.0 * map->geometry.resolution);
            RCLCPP_INFO(get_logger(), "ICP target: %zu map surface points",
                        map_index_->points().size());
        }

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf/pose", 20);
        tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 50, [this](nav_msgs::msg::Odometry::SharedPtr m) { onOdom(*m); });
        if (use_imu_)
            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                "imu", 50, [this](sensor_msgs::msg::Imu::SharedPtr m) {
                    ekf_->updateYaw(yawFromQuaternion(m->orientation), imu_yaw_sigma_);
                });
        if (use_icp_)
            scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::LaserScan::SharedPtr m) { onScan(*m); });
    }

    ~EkfLocalizationNode() override {
        RCLCPP_INFO(get_logger(),
                    "ICP corrections: %zu accepted, %zu rejected by quality, %zu by gate",
                    icp_accepted_, icp_rejected_quality_, icp_rejected_gate_);
    }

private:
    void onOdom(const nav_msgs::msg::Odometry& m) {
        const rclcpp::Time stamp(m.header.stamp);
        if (last_odom_stamp_) {
            const double dt = (stamp - *last_odom_stamp_).seconds();
            ekf_->predict(m.twist.twist.linear.x, m.twist.twist.angular.z, dt);
        }
        last_odom_stamp_ = stamp;
        odom_pose_ = {m.pose.pose.position.x, m.pose.pose.position.y,
                      yawFromQuaternion(m.pose.pose.orientation)};
        publish(stamp);
    }

    void onScan(const sensor_msgs::msg::LaserScan& scan) {
        // Beams to points in the robot frame; no-return (inf) beams are skipped.
        std::vector<icp_core::Point2> pts;
        for (std::size_t i = 0; i < scan.ranges.size(); i += scan_stride_) {
            const float r = scan.ranges[i];
            if (!std::isfinite(r) || r < scan.range_min || r >= scan.range_max) continue;
            const double a = scan.angle_min + scan.angle_increment * static_cast<double>(i);
            pts.push_back({r * std::cos(a), r * std::sin(a)});
        }
        const ekf_core::Pose2D& p = ekf_->pose();
        const icp_core::IcpResult res =
            icp_core::align(pts, *map_index_, {p.x, p.y, p.theta}, icp_params_);
        if (!res.converged || res.rmse > icp_max_rmse_ ||
            res.inlier_ratio < icp_min_inlier_ratio_) {
            ++icp_rejected_quality_;
            return;
        }
        if (!ekf_->updatePose({res.transform.x, res.transform.y, res.transform.theta}, icp_cov_)) {
            ++icp_rejected_gate_;
            return;
        }
        ++icp_accepted_;
        RCLCPP_DEBUG(get_logger(), "ICP ok: rmse %.3f, inliers %.2f, %d it", res.rmse,
                     res.inlier_ratio, res.iterations);
    }

    void publish(const rclcpp::Time& stamp) {
        const ekf_core::Pose2D& p = ekf_->pose();
        const ekf_core::Mat3& c = ekf_->covariance();
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = p.x;
        msg.pose.pose.position.y = p.y;
        msg.pose.pose.orientation = yawToQuaternion(p.theta);
        // 6x6 row-major (x, y, z, roll, pitch, yaw): fill the planar block.
        const int idx[3] = {0, 1, 5};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) msg.pose.covariance[idx[i] * 6 + idx[j]] = c[i][j];
        pose_pub_->publish(msg);

        // map->odom = map->base * (odom->base)^-1
        const double dth = p.theta - odom_pose_.theta;
        const double cs = std::cos(dth), sn = std::sin(dth);
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "map";
        tf.child_frame_id = "odom";
        tf.transform.translation.x = p.x - (cs * odom_pose_.x - sn * odom_pose_.y);
        tf.transform.translation.y = p.y - (sn * odom_pose_.x + cs * odom_pose_.y);
        tf.transform.rotation = yawToQuaternion(dth);
        tf_->sendTransform(tf);
    }

    std::optional<ekf_core::DiffDriveEkf> ekf_;
    std::optional<icp_core::PointIndex> map_index_;
    icp_core::IcpParams icp_params_;
    ekf_core::Mat3 icp_cov_{};
    double icp_max_rmse_ = 0.05, icp_min_inlier_ratio_ = 0.5, imu_yaw_sigma_ = 0.02;
    bool use_imu_ = true, use_icp_ = true;
    std::size_t scan_stride_ = 2;
    std::size_t icp_accepted_ = 0, icp_rejected_quality_ = 0, icp_rejected_gate_ = 0;
    std::optional<rclcpp::Time> last_odom_stamp_;
    ekf_core::Pose2D odom_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EkfLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
