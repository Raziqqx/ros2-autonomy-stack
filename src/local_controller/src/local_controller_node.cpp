// local_controller: follows the global path with pure pursuit (baseline) or
// DWA (uses the latest scan as obstacles) and publishes /cmd_vel at a fixed
// rate.
//
// Subscribes:  /plan (nav_msgs/Path; empty = stop and go idle), /ekf/pose,
//              /odom (measured wheel speed, for stuck detection), /scan
// Publishes:   /cmd_vel, /controller/state (std_msgs/String:
//              idle | following | arrived | blocked)
//
// "blocked" means the robot cannot make progress: it has been commanded to
// drive but the wheels have not moved for stuck_timeout s (pressed against
// an obstacle), or DWA found no collision-free velocity for that long. The
// controller then stops; the behaviour tree decides how to recover.
#include <chrono>
#include <cmath>
#include <control_core/controllers.hpp>
#include <control_core/path_tracking.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

namespace cc = control_core;

namespace {
double yawOf(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

class LocalControllerNode : public rclcpp::Node {
public:
    LocalControllerNode() : Node("local_controller"), stuck_(1.0) {
        const std::string type = declare_parameter<std::string>("controller", "pure_pursuit");
        if (type != "pure_pursuit" && type != "dwa")
            throw std::runtime_error("local_controller: controller must be pure_pursuit or dwa");
        use_dwa_ = type == "dwa";
        const double rate = declare_parameter("rate", 10.0);
        goal_tolerance_ = declare_parameter("goal_tolerance", 0.1);
        stuck_ = cc::StuckDetector(declare_parameter("stuck_timeout", 1.0));

        cc::PurePursuitParams pp;
        pp.lookahead = declare_parameter("pp_lookahead", pp.lookahead);
        pp.v_max = declare_parameter("v_max", pp.v_max);
        pp.w_max = declare_parameter("w_max", pp.w_max);
        pp.goal_tolerance = goal_tolerance_;
        pure_pursuit_ = cc::PurePursuit(pp);

        dwa_.v_max = pp.v_max;
        dwa_.w_max = pp.w_max;
        dwa_.control_period = 1.0 / rate;
        dwa_.robot_radius = declare_parameter("robot_radius", 0.2);
        dwa_.horizon = declare_parameter("dwa_horizon", dwa_.horizon);
        dwa_.w_clearance = declare_parameter("dwa_w_clearance", dwa_.w_clearance);
        dwa_lookahead_ = declare_parameter("dwa_lookahead", 1.0);
        // Only scan points the robot could reach within the horizon matter.
        obstacle_range_ = dwa_.v_max * dwa_.horizon + dwa_.robot_radius + dwa_.clearance_cap;

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>("controller/state", 10);
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "plan", 10, [this](nav_msgs::msg::Path::SharedPtr m) { onPath(*m); });
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "ekf/pose", 20, [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
                pose_ = cc::Pose2{m->pose.pose.position.x, m->pose.pose.position.y,
                                  yawOf(m->pose.pose.orientation)};
            });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 20, [this](nav_msgs::msg::Odometry::SharedPtr m) {
                measured_ = {m->twist.twist.linear.x, m->twist.twist.angular.z};
            });
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr m) { scan_ = m; });
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { step(); });
        RCLCPP_INFO(get_logger(), "local_controller: %s", type.c_str());
    }

private:
    void onPath(const nav_msgs::msg::Path& msg) {
        if (msg.poses.empty()) {
            if (state_ != "idle") stop();
            path_.clear();
            state_ = "idle";
            return;
        }
        std::vector<cc::Point> path;
        for (const auto& p : msg.poses) path.push_back({p.pose.position.x, p.pose.position.y});
        // A replan of the same goal keeps the arrival / stuck state; a new
        // goal resets it.
        const bool new_goal = path_.empty() || std::hypot(path.back().x - path_.back().x,
                                                          path.back().y - path_.back().y) > 0.05;
        if (new_goal) {
            stuck_.reset();
            state_ = "following";
        }
        path_ = path;
        pure_pursuit_.setPath(path_);
    }

    void step() {
        if (state_ == "following" && pose_ && !path_.empty()) follow();
        std_msgs::msg::String s;
        s.data = state_;
        state_pub_->publish(s);
    }

    void follow() {
        const cc::Pose2& pose = *pose_;
        const double t = now().seconds();
        cc::VelocityCommand cmd;
        bool infeasible = false;
        if (std::hypot(path_.back().x - pose.x, path_.back().y - pose.y) < goal_tolerance_) {
            state_ = "arrived";
            stop();
            return;
        }
        if (use_dwa_) {
            const cc::Point goal = cc::localGoal(path_, pose, dwa_lookahead_);
            const cc::DwaOutput out = cc::computeDwa(pose, last_cmd_, goal, scanPoints(pose), dwa_);
            cmd = out.cmd;
            infeasible = !out.feasible;
        } else {
            cmd = pure_pursuit_.compute(pose).cmd;
        }
        // Blocked if pushing against something, or DWA is boxed in.
        const bool stalled = stuck_.update(t, infeasible ? 1.0 : cmd.v, infeasible ? 0.0 : measured_.v);
        if (stalled) {
            RCLCPP_WARN(get_logger(), "blocked at (%.2f, %.2f)", pose.x, pose.y);
            state_ = "blocked";
            stop();
            return;
        }
        publish(cmd);
    }

    // Scan endpoints in the map frame (EKF pose), limited to the DWA's reach.
    std::vector<cc::Point> scanPoints(const cc::Pose2& pose) const {
        std::vector<cc::Point> pts;
        if (!scan_) return pts;
        for (std::size_t i = 0; i < scan_->ranges.size(); ++i) {
            const double r = scan_->ranges[i];
            if (!std::isfinite(r) || r < scan_->range_min || r > obstacle_range_) continue;
            const double a = pose.theta + scan_->angle_min + i * scan_->angle_increment;
            pts.push_back({pose.x + r * std::cos(a), pose.y + r * std::sin(a)});
        }
        return pts;
    }

    void publish(const cc::VelocityCommand& c) {
        geometry_msgs::msg::Twist tw;
        tw.linear.x = c.v;
        tw.angular.z = c.w;
        cmd_pub_->publish(tw);
        last_cmd_ = c;
    }

    void stop() { publish({0.0, 0.0}); }

    bool use_dwa_ = false;
    double goal_tolerance_, dwa_lookahead_, obstacle_range_;
    cc::PurePursuit pure_pursuit_;
    cc::DwaParams dwa_;
    cc::StuckDetector stuck_;
    std::string state_ = "idle";
    std::vector<cc::Point> path_;
    std::optional<cc::Pose2> pose_;
    cc::VelocityCommand measured_, last_cmd_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalControllerNode>());
    rclcpp::shutdown();
    return 0;
}
