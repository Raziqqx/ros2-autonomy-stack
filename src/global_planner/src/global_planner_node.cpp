// global_planner: plans from the EKF pose to the current goal on the prior
// map merged with the live occupancy grid, and replans periodically so
// obstacles the LiDAR discovers (not in the prior map) are avoided.
//
// Subscribes:  /goal_pose (geometry_msgs/PoseStamped; an empty frame_id
//              cancels the goal), /ekf/pose, /map (live grid from grid_mapping)
// Publishes:   /plan (nav_msgs/Path; an EMPTY path means "no goal" or
//              "planning failed"), /planner/status (std_msgs/String:
//              ok | start_in_collision | goal_blocked | no_path | idle)
//
// Parameters:  planner (astar | thetastar), robot_radius, lethal_margin,
//              inflation_margin, replan_period, goal_tolerance, map_yaml,
//              live_occupied_threshold, live_ignore_radius.
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <grid_core/grid.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <planning_core/nav_planner.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace pc = planning_core;

class GlobalPlannerNode : public rclcpp::Node {
public:
    GlobalPlannerNode() : Node("global_planner") {
        const std::string algo = declare_parameter<std::string>("planner", "astar");
        if (algo != "astar" && algo != "thetastar")
            throw std::runtime_error("global_planner: planner must be astar or thetastar");
        params_.algorithm = algo == "thetastar" ? pc::Algorithm::ThetaStar : pc::Algorithm::AStar;
        const double radius = declare_parameter("robot_radius", 0.2);
        const double lethal_margin = declare_parameter("lethal_margin", 0.05);
        const double inflation_margin = declare_parameter("inflation_margin", 0.15);
        goal_tolerance_ = declare_parameter("goal_tolerance", 0.15);
        live_threshold_ = static_cast<int8_t>(declare_parameter("live_occupied_threshold", 65));
        const double live_ignore = declare_parameter("live_ignore_radius", 0.1);

        std::string err;
        auto map = grid_core::loadMap(declare_parameter<std::string>("map_yaml", ""), &err);
        if (!map) throw std::runtime_error("global_planner: " + err);
        prior_ = std::move(*map);
        const double res = prior_.geometry.resolution;
        params_.lethal_radius = (radius + lethal_margin) / res;
        params_.inflation_radius = (radius + inflation_margin) / res;
        live_ignore_cells_ = live_ignore / res;

        plan_pub_ = create_publisher<nav_msgs::msg::Path>("plan", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>("planner/status", 10);
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", rclcpp::QoS(1).transient_local().reliable(), [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
                if (m->header.frame_id.empty()) {
                    goal_.reset();
                    publishEmpty("idle");
                } else {
                    goal_ = *m;
                    plan();
                }
            });
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "ekf/pose", 20, [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
                pose_ = m->pose.pose;
            });
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(1).transient_local().reliable(),
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) { live_ = m; });
        timer_ = create_wall_timer(std::chrono::duration<double>(declare_parameter("replan_period", 1.0)),
                                   [this] { if (goal_) plan(); });
        RCLCPP_INFO(get_logger(), "global_planner: %s, lethal %.1f / inflation %.1f cells",
                    algo.c_str(), params_.lethal_radius, params_.inflation_radius);
    }

private:
    // Prior map OR live-occupied cells, on the prior map's geometry. Live hits
    // within `live_ignore_radius` of a prior obstacle are attributed to that
    // obstacle (localisation and range noise smear walls by a cell or so);
    // counting them would thicken every wall and make the robot appear in
    // collision near corners.
    pc::PlanningGrid buildGrid() const {
        const auto& g = prior_.geometry;
        pc::PlanningGrid grid{g.width, g.height, prior_.occupied};
        if (!live_) return grid;
        const pc::PlanningGrid near_prior = pc::inflate(grid, live_ignore_cells_);
        const auto& info = live_->info;
        for (int y = 0; y < g.height; ++y) {
            for (int x = 0; x < g.width; ++x) {
                const std::size_t i = static_cast<std::size_t>(y) * g.width + x;
                if (near_prior.blocked[i]) continue;
                double wx, wy;
                g.cellToWorld({x, y}, wx, wy);
                const int lx = static_cast<int>(std::floor((wx - info.origin.position.x) / info.resolution));
                const int ly = static_cast<int>(std::floor((wy - info.origin.position.y) / info.resolution));
                if (lx < 0 || ly < 0 || lx >= static_cast<int>(info.width) ||
                    ly >= static_cast<int>(info.height))
                    continue;
                if (live_->data[static_cast<std::size_t>(ly) * info.width + lx] >= live_threshold_)
                    grid.blocked[i] = 1;
            }
        }
        return grid;
    }

    void plan() {
        if (!pose_ || !goal_) return;
        const double gx = goal_->pose.position.x, gy = goal_->pose.position.y;
        if (std::hypot(gx - pose_->position.x, gy - pose_->position.y) < goal_tolerance_) {
            goal_.reset();  // reached: stop replanning, the controller holds position
            return;
        }
        const auto& g = prior_.geometry;
        const auto s = g.worldToCell(pose_->position.x, pose_->position.y);
        const auto e = g.worldToCell(gx, gy);
        const auto t0 = std::chrono::steady_clock::now();
        const pc::NavPlanResult r = pc::planOnMap(buildGrid(), {s.x, s.y}, {e.x, e.y}, params_);
        const double ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        if (r.status != pc::NavPlanStatus::Ok) {
            RCLCPP_WARN(get_logger(), "planning failed: %s", pc::toString(r.status));
            publishEmpty(pc::toString(r.status));
            return;
        }
        nav_msgs::msg::Path path;
        path.header.stamp = now();
        path.header.frame_id = "map";
        for (std::size_t i = 0; i < r.plan.path.size(); ++i) {
            geometry_msgs::msg::PoseStamped p;
            p.header = path.header;
            if (i + 1 == r.plan.path.size()) {
                p.pose = goal_->pose;  // exact goal, not its cell centre
            } else {
                g.cellToWorld({r.plan.path[i].x, r.plan.path[i].y}, p.pose.position.x, p.pose.position.y);
                p.pose.orientation.w = 1.0;
            }
            path.poses.push_back(p);
        }
        plan_pub_->publish(path);
        publishStatus("ok");
        RCLCPP_DEBUG(get_logger(), "plan: %zu poses, %.2f m, %zu expansions, %.1f ms",
                     path.poses.size(), r.plan.cost * g.resolution, r.plan.expansions, ms);
    }

    void publishEmpty(const std::string& status) {
        nav_msgs::msg::Path path;
        path.header.stamp = now();
        path.header.frame_id = "map";
        plan_pub_->publish(path);
        publishStatus(status);
    }

    void publishStatus(const std::string& s) {
        std_msgs::msg::String m;
        m.data = s;
        status_pub_->publish(m);
    }

    grid_core::StaticMap prior_;
    pc::NavPlanParams params_;
    double goal_tolerance_, live_ignore_cells_ = 2.0;
    int8_t live_threshold_;
    std::optional<geometry_msgs::msg::Pose> pose_;
    std::optional<geometry_msgs::msg::PoseStamped> goal_;
    nav_msgs::msg::OccupancyGrid::SharedPtr live_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
