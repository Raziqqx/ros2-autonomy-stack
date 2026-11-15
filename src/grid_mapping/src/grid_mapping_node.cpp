// grid_mapping: builds a log-odds occupancy grid from LiDAR scans placed at
// the estimated robot pose (mapping with known poses).
//
// Subscribes:  /scan, /ekf/pose (geometry_msgs/PoseWithCovarianceStamped)
// Publishes:   /map (nav_msgs/OccupancyGrid, transient-local, periodic)
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <grid_core/grid.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class GridMappingNode : public rclcpp::Node {
public:
    GridMappingNode() : Node("grid_mapping") {
        grid_core::GridGeometry geom;
        geom.resolution = declare_parameter("resolution", 0.05);
        geom.origin_x = declare_parameter("origin_x", -1.0);
        geom.origin_y = declare_parameter("origin_y", -1.0);
        geom.width =
            static_cast<int>(std::lround(declare_parameter("width_m", 14.0) / geom.resolution));
        geom.height =
            static_cast<int>(std::lround(declare_parameter("height_m", 11.0) / geom.resolution));
        grid_core::LogOddsParams lp;
        lp.l_occ = declare_parameter("l_occ", lp.l_occ);
        lp.l_free = declare_parameter("l_free", lp.l_free);
        grid_.emplace(geom, lp);
        // A scan is only inserted if the latest pose is this fresh; otherwise
        // it would be smeared by the robot's motion since the pose.
        max_pose_age_ = declare_parameter("max_pose_age", 0.1);

        auto map_qos = rclcpp::QoS(1).transient_local().reliable();
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", map_qos);
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "ekf/pose", 20,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) { pose_ = *m; });
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr m) { onScan(*m); });
        timer_ = create_wall_timer(
            std::chrono::duration<double>(declare_parameter("publish_period", 1.0)),
            [this] { publishMap(); });
    }

private:
    void onScan(const sensor_msgs::msg::LaserScan& scan) {
        if (!pose_) return;
        const double age =
            (rclcpp::Time(scan.header.stamp) - rclcpp::Time(pose_->header.stamp)).seconds();
        if (std::abs(age) > max_pose_age_) {
            ++skipped_;
            return;
        }
        const auto& p = pose_->pose.pose;
        const double yaw = std::atan2(
            2.0 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
            1.0 - 2.0 * (p.orientation.y * p.orientation.y + p.orientation.z * p.orientation.z));
        grid_->insertScan(p.position.x, p.position.y, yaw, scan.ranges, scan.angle_min,
                          scan.angle_increment, scan.range_min, scan.range_max);
        ++inserted_;
    }

    void publishMap() {
        const grid_core::GridGeometry& g = grid_->geometry();
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.info.resolution = static_cast<float>(g.resolution);
        msg.info.width = static_cast<uint32_t>(g.width);
        msg.info.height = static_cast<uint32_t>(g.height);
        msg.info.origin.position.x = g.origin_x;
        msg.info.origin.position.y = g.origin_y;
        msg.info.origin.orientation.w = 1.0;
        msg.data = grid_->toOccupancy();
        map_pub_->publish(msg);
        RCLCPP_DEBUG(get_logger(), "map: %zu scans inserted (%zu skipped), %zu occupied cells",
                     inserted_, skipped_, grid_->countOccupied());
    }

    std::optional<grid_core::LogOddsGrid> grid_;
    std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> pose_;
    double max_pose_age_ = 0.1;
    std::size_t inserted_ = 0, skipped_ = 0;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridMappingNode>());
    rclcpp::shutdown();
    return 0;
}
