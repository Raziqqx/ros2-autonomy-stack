// sim2d: a deterministic (seeded) 2D simulator for a differential-drive robot.
//
// Subscribes:  /cmd_vel (geometry_msgs/Twist)
// Publishes:   /scan (sensor_msgs/LaserScan), /odom (nav_msgs/Odometry, noisy
//              wheel odometry), /imu (sensor_msgs/Imu, noisy yaw + gyro),
//              /ground_truth (nav_msgs/Odometry in the map frame), TF odom->base_link.
//
// The true robot follows the commanded velocities exactly (clamped to limits)
// and stops if the next step would collide with the map. All sensor errors are
// drawn from one std::mt19937 seeded by the `seed` parameter.
//
// Evaluation knobs: `noise_scale` multiplies every sensor error (white and
// systematic); `clutter_count` adds random boxes that are NOT in the map the
// localiser and planner load (an environment that changed since mapping);
// `drop_after_m` drops a wall across the robot's path once it has driven
// that far (blocked-path / recovery scenario). The true world is published on
// /sim/world (latched) for evaluation plots.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <grid_core/grid.hpp>
#include <limits>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace {

struct Pose {
    double x = 0.0, y = 0.0, theta = 0.0;
};

Pose integrate(const Pose& p, double v, double w, double dt) {
    Pose out = p;
    if (std::abs(w) < 1e-9) {
        out.x += v * dt * std::cos(p.theta);
        out.y += v * dt * std::sin(p.theta);
    } else {
        out.x += v / w * (std::sin(p.theta + w * dt) - std::sin(p.theta));
        out.y += v / w * (std::cos(p.theta) - std::cos(p.theta + w * dt));
    }
    out.theta = std::remainder(p.theta + w * dt, 2.0 * M_PI);
    return out;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(0.5 * yaw);
    q.w = std::cos(0.5 * yaw);
    return q;
}

}  // namespace

class Sim2dNode : public rclcpp::Node {
public:
    Sim2dNode() : Node("sim2d") {
        const std::string map_yaml = declare_parameter<std::string>("map_yaml", "");
        rng_.seed(static_cast<unsigned>(declare_parameter<int>("seed", 1)));
        truth_ = {declare_parameter("initial_x", 1.5), declare_parameter("initial_y", 1.5),
                  declare_parameter("initial_theta", 0.0)};
        odom_pose_ = {0.0, 0.0, 0.0};  // odometry starts at its own origin
        robot_radius_ = declare_parameter("robot_radius", 0.2);
        v_max_ = declare_parameter("v_max", 1.0);
        w_max_ = declare_parameter("w_max", 2.0);
        cmd_timeout_ = declare_parameter("cmd_timeout", 0.5);
        const double rate = declare_parameter("sim_rate", 50.0);
        scan_every_ = std::max<int>(
            1, static_cast<int>(std::lround(rate / declare_parameter("scan_rate", 10.0))));
        beams_ = declare_parameter("scan_beams", 360);
        range_min_ = declare_parameter("range_min", 0.1);
        range_max_ = declare_parameter("range_max", 8.0);
        scan_noise_ = declare_parameter("scan_noise", 0.01);
        // Wheel odometry: scale error on v and bias on w are systematic (they
        // cause the drift a filter must correct); the rest is white noise per
        // sample.
        odom_v_scale_ = declare_parameter("odom_v_scale", 1.02);
        odom_w_bias_ = declare_parameter("odom_w_bias", 0.02);
        odom_v_noise_ = declare_parameter("odom_v_noise", 0.05);
        odom_w_noise_ = declare_parameter("odom_w_noise", 0.05);
        imu_yaw_noise_ = declare_parameter("imu_yaw_noise", 0.02);
        imu_gyro_noise_ = declare_parameter("imu_gyro_noise", 0.01);
        const double k = declare_parameter("noise_scale", 1.0);
        scan_noise_ *= k;
        odom_v_scale_ = 1.0 + k * (odom_v_scale_ - 1.0);
        odom_w_bias_ *= k;
        odom_v_noise_ *= k;
        odom_w_noise_ *= k;
        imu_yaw_noise_ *= k;
        imu_gyro_noise_ *= k;
        drop_after_m_ = declare_parameter("drop_after_m", -1.0);
        drop_ahead_ = declare_parameter("drop_ahead", 0.45);
        drop_width_ = declare_parameter("drop_width", 1.6);
        dt_ = 1.0 / rate;

        std::string err;
        auto map = grid_core::loadMap(map_yaml, &err);
        if (!map) throw std::runtime_error("sim2d: " + err);
        map_ = std::move(*map);
        if (grid_core::discCollides(map_, truth_.x, truth_.y, robot_radius_))
            throw std::runtime_error("sim2d: initial pose is in collision");
        addClutter(declare_parameter("clutter_count", 0),
                   declare_parameter("clutter_keepout", std::vector<double>{}));

        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 20);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 20);
        truth_pub_ = create_publisher<nav_msgs::msg::Odometry>("ground_truth", 20);
        tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        world_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "sim/world", rclcpp::QoS(1).transient_local().reliable());
        publishWorld();
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                cmd_v_ = std::clamp(msg->linear.x, -v_max_, v_max_);
                cmd_w_ = std::clamp(msg->angular.z, -w_max_, w_max_);
                last_cmd_ = now();
            });
        last_cmd_ = now();
        timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this] { step(); });
        RCLCPP_INFO(get_logger(), "sim2d: map %dx%d @ %.3f m, seed %ld", map_.geometry.width,
                    map_.geometry.height, map_.geometry.resolution, get_parameter("seed").as_int());
    }

private:
    double gauss(double sigma) {
        return sigma > 0.0 ? std::normal_distribution<double>(0.0, sigma)(rng_) : 0.0;
    }

    void step() {
        const rclcpp::Time stamp = now();
        double v = cmd_v_, w = cmd_w_;
        if ((stamp - last_cmd_).seconds() > cmd_timeout_) v = w = 0.0;  // dead-man stop

        const Pose next = integrate(truth_, v, w, dt_);
        if (grid_core::discCollides(map_, next.x, next.y, robot_radius_)) {
            if (!in_collision_)
                RCLCPP_WARN(get_logger(), "collision at (%.2f, %.2f): stopping", next.x, next.y);
            in_collision_ = true;
            v = w = 0.0;
        } else {
            in_collision_ = false;
            travelled_ += std::hypot(next.x - truth_.x, next.y - truth_.y);
            truth_ = next;
        }
        if (drop_after_m_ >= 0.0 && !dropped_ && travelled_ >= drop_after_m_) dropWall();

        // Noisy wheel odometry, integrated in the odom frame. Encoders read
        // exactly zero while the wheels are still.
        const bool moving = v != 0.0 || w != 0.0;
        const double vm = moving ? v * odom_v_scale_ + gauss(odom_v_noise_) : 0.0;
        const double wm = moving ? w + odom_w_bias_ + gauss(odom_w_noise_) : 0.0;
        odom_pose_ = integrate(odom_pose_, vm, wm, dt_);

        publishOdometry(odom_pub_, stamp, "odom", odom_pose_, vm, wm);
        publishOdometry(truth_pub_, stamp, "map", truth_, v, w);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = odom_pose_.x;
        tf.transform.translation.y = odom_pose_.y;
        tf.transform.rotation = yawToQuaternion(odom_pose_.theta);
        tf_->sendTransform(tf);

        sensor_msgs::msg::Imu imu;
        imu.header.stamp = stamp;
        imu.header.frame_id = "base_link";
        imu.orientation = yawToQuaternion(truth_.theta + gauss(imu_yaw_noise_));
        imu.orientation_covariance[8] = imu_yaw_noise_ * imu_yaw_noise_;
        imu.angular_velocity.z = w + gauss(imu_gyro_noise_);
        imu.angular_velocity_covariance[8] = imu_gyro_noise_ * imu_gyro_noise_;
        imu.linear_acceleration_covariance[0] = -1.0;  // not provided
        imu_pub_->publish(imu);

        if (++steps_ % scan_every_ == 0) publishScan(stamp);
    }

    // Random axis-aligned boxes (0.3-0.6 m) in free space, away from the
    // robot and from the keep-out points (x, y pairs: e.g. the goals).
    void addClutter(long count, const std::vector<double>& keepout) {
        const auto& g = map_.geometry;
        std::uniform_real_distribution<double> ux(0.0, g.width * g.resolution),
            uy(0.0, g.height * g.resolution), usize(0.3, 0.6);
        long placed = 0;
        for (int attempt = 0; placed < count && attempt < 1000; ++attempt) {
            const double cx = g.origin_x + ux(rng_), cy = g.origin_y + uy(rng_);
            const double sx = usize(rng_), sy = usize(rng_);
            bool ok = std::hypot(cx - truth_.x, cy - truth_.y) > 1.0;
            for (std::size_t i = 0; ok && i + 1 < keepout.size(); i += 2)
                ok = std::hypot(cx - keepout[i], cy - keepout[i + 1]) > 1.0;
            if (!ok) continue;
            fillRect(cx, cy, 0.0, sx, sy);
            ++placed;
        }
        RCLCPP_INFO(get_logger(), "sim2d: %ld clutter boxes", placed);
    }

    // Marks cells whose centres lie in a rectangle (centre, yaw, size).
    void fillRect(double cx, double cy, double yaw, double sx, double sy) {
        const auto& g = map_.geometry;
        const double r = 0.5 * std::hypot(sx, sy);
        const auto lo = g.worldToCell(cx - r, cy - r), hi = g.worldToCell(cx + r, cy + r);
        for (int y = lo.y; y <= hi.y; ++y) {
            for (int x = lo.x; x <= hi.x; ++x) {
                const grid_core::Cell c{x, y};
                if (!g.inBounds(c)) continue;
                double wx, wy;
                g.cellToWorld(c, wx, wy);
                const double dx = wx - cx, dy = wy - cy;
                const double u = std::cos(yaw) * dx + std::sin(yaw) * dy;
                const double v = -std::sin(yaw) * dx + std::cos(yaw) * dy;
                if (std::abs(u) <= 0.5 * sx && std::abs(v) <= 0.5 * sy)
                    map_.occupied[g.index(c)] = 1;
            }
        }
    }

    // A thin wall across the robot's heading, drop_ahead metres in front.
    void dropWall() {
        dropped_ = true;
        const double cx = truth_.x + drop_ahead_ * std::cos(truth_.theta);
        const double cy = truth_.y + drop_ahead_ * std::sin(truth_.theta);
        fillRect(cx, cy, truth_.theta, 0.2, drop_width_);
        RCLCPP_WARN(get_logger(), "sim2d: dropped a wall at (%.2f, %.2f) after %.2f m", cx, cy,
                    travelled_);
        publishWorld();
    }

    void publishWorld() {
        const auto& g = map_.geometry;
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.info.resolution = static_cast<float>(g.resolution);
        msg.info.width = static_cast<unsigned>(g.width);
        msg.info.height = static_cast<unsigned>(g.height);
        msg.info.origin.position.x = g.origin_x;
        msg.info.origin.position.y = g.origin_y;
        msg.info.origin.orientation.w = 1.0;
        msg.data.resize(map_.occupied.size());
        for (std::size_t i = 0; i < map_.occupied.size(); ++i)
            msg.data[i] = map_.occupied[i] ? 100 : 0;
        world_pub_->publish(msg);
    }

    void publishOdometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& pub,
                         const rclcpp::Time& stamp, const std::string& frame, const Pose& p,
                         double v, double w) {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame;
        msg.child_frame_id = "base_link";
        msg.pose.pose.position.x = p.x;
        msg.pose.pose.position.y = p.y;
        msg.pose.pose.orientation = yawToQuaternion(p.theta);
        msg.twist.twist.linear.x = v;
        msg.twist.twist.angular.z = w;
        pub->publish(msg);
    }

    void publishScan(const rclcpp::Time& stamp) {
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = stamp;
        scan.header.frame_id = "base_link";
        scan.angle_min = -M_PI;
        scan.angle_increment = static_cast<float>(2.0 * M_PI / beams_);
        scan.angle_max = static_cast<float>(-M_PI + (beams_ - 1) * scan.angle_increment);
        scan.scan_time = static_cast<float>(scan_every_ * dt_);
        scan.range_min = static_cast<float>(range_min_);
        scan.range_max = static_cast<float>(range_max_);
        scan.ranges.resize(beams_);
        for (int i = 0; i < beams_; ++i) {
            const double a = truth_.theta + scan.angle_min + i * scan.angle_increment;
            const double r = grid_core::raycast(map_, truth_.x, truth_.y, a, range_max_);
            // REP 117: no return within range is +inf.
            scan.ranges[i] = r >= range_max_
                                 ? std::numeric_limits<float>::infinity()
                                 : static_cast<float>(std::max(range_min_, r + gauss(scan_noise_)));
        }
        scan_pub_->publish(scan);
    }

    grid_core::StaticMap map_;
    std::mt19937 rng_;
    Pose truth_, odom_pose_;
    double robot_radius_, v_max_, w_max_, cmd_timeout_, dt_;
    double range_min_, range_max_, scan_noise_;
    double odom_v_scale_, odom_w_bias_, odom_v_noise_, odom_w_noise_, imu_yaw_noise_,
        imu_gyro_noise_;
    double drop_after_m_, drop_ahead_, drop_width_, travelled_ = 0.0;
    bool dropped_ = false;
    int scan_every_, beams_;
    long steps_ = 0;
    double cmd_v_ = 0.0, cmd_w_ = 0.0;
    bool in_collision_ = false;
    rclcpp::Time last_cmd_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_, truth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr world_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sim2dNode>());
    rclcpp::shutdown();
    return 0;
}
