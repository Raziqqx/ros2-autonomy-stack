// task_bt: mission execution with BehaviorTree.CPP v4 (trees/mission.xml).
//
// Leaves that talk to the stack:
//   NavigateTo  sends /goal_pose, then RUNNING until the EKF pose is within
//               goal_tolerance (SUCCESS) or the planner reports a failure,
//               the controller reports "blocked", or the timeout expires
//               (FAILURE, goal cancelled).
//   Recover     cancels the goal, backs up and rotates in place on /cmd_vel
//               (alternating direction), then SUCCESS.
// Every step is published on /task_bt/events (std_msgs/String, latched) so
// tests and the evaluation can count recoveries and see the outcome.
#include <behaviortree_cpp/bt_factory.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "task_bt/bt_common.hpp"

using task_bt::Pose2D;

namespace {

// State shared by the ROS callbacks and the BT leaves. Everything runs on one
// single-threaded executor, so no locking is needed.
struct Context {
    rclcpp::Node* node = nullptr;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub;
    std::optional<Pose2D> pose;
    std::string planner_status;     // latest, cleared when a goal is sent
    std::string controller_state;   // latest, cleared when a goal is sent
    double goal_tolerance = 0.15;

    void event(const std::string& e) {
        std_msgs::msg::String m;
        m.data = e;
        event_pub->publish(m);
        RCLCPP_INFO(node->get_logger(), "event: %s", e.c_str());
    }
    void sendGoal(const Pose2D& g) {
        geometry_msgs::msg::PoseStamped m;
        m.header.stamp = node->now();
        m.header.frame_id = "map";
        m.pose.position.x = g.x;
        m.pose.position.y = g.y;
        m.pose.orientation.z = std::sin(0.5 * g.theta);
        m.pose.orientation.w = std::cos(0.5 * g.theta);
        planner_status.clear();
        controller_state.clear();
        goal_pub->publish(m);
    }
    void cancelGoal() {
        geometry_msgs::msg::PoseStamped m;  // empty frame_id = cancel
        m.header.stamp = node->now();
        goal_pub->publish(m);
    }
    void drive(double v, double w) {
        geometry_msgs::msg::Twist t;
        t.linear.x = v;
        t.angular.z = w;
        cmd_pub->publish(t);
    }
    double now() const { return node->now().seconds(); }
};

class NavigateTo : public BT::StatefulActionNode {
public:
    NavigateTo(const std::string& n, const BT::NodeConfig& c, Context* ctx)
        : BT::StatefulActionNode(n, c), ctx_(ctx) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<Pose2D>("goal"), BT::InputPort<double>("timeout", 90.0, "s")};
    }

    BT::NodeStatus onStart() override {
        const auto g = getInput<Pose2D>("goal");
        if (!g) throw BT::RuntimeError("NavigateTo: ", g.error());
        goal_ = *g;
        timeout_ = getInput<double>("timeout").value_or(90.0);
        start_ = ctx_->now();
        ctx_->sendGoal(goal_);
        ctx_->event("navigate " + fmt(goal_.x) + " " + fmt(goal_.y));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (ctx_->pose && std::hypot(ctx_->pose->x - goal_.x, ctx_->pose->y - goal_.y) <
                              ctx_->goal_tolerance) {
            ctx_->cancelGoal();
            ctx_->event("goal_reached " + fmt(goal_.x) + " " + fmt(goal_.y));
            return BT::NodeStatus::SUCCESS;
        }
        const std::string& ps = ctx_->planner_status;
        if (!ps.empty() && ps != "ok" && ps != "idle") return fail("planner_failed " + ps);
        if (ctx_->controller_state == "blocked") return fail("blocked");
        if (ctx_->now() - start_ > timeout_) return fail("timeout");
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { ctx_->cancelGoal(); }

private:
    BT::NodeStatus fail(const std::string& why) {
        ctx_->cancelGoal();
        ctx_->event(why);
        return BT::NodeStatus::FAILURE;
    }
    static std::string fmt(double v) {
        char b[32];
        std::snprintf(b, sizeof(b), "%.2f", v);
        return b;
    }

    Context* ctx_;
    Pose2D goal_;
    double timeout_ = 90.0, start_ = 0.0;
};

class Recover : public BT::StatefulActionNode {
public:
    Recover(const std::string& n, const BT::NodeConfig& c, Context* ctx)
        : BT::StatefulActionNode(n, c), ctx_(ctx) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("backup_distance", 0.3, "m"),
                BT::InputPort<double>("rotate_angle", 1.57, "rad")};
    }

    BT::NodeStatus onStart() override {
        ctx_->cancelGoal();
        backup_time_ = getInput<double>("backup_distance").value_or(0.3) / kBackupSpeed;
        rotate_time_ = getInput<double>("rotate_angle").value_or(1.57) / kRotateSpeed;
        direction_ = -direction_;  // alternate left / right turns between attempts
        start_ = ctx_->now();
        ctx_->event("recovery_start");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        const double t = ctx_->now() - start_;
        if (t < backup_time_) {
            ctx_->drive(-kBackupSpeed, 0.0);
        } else if (t < backup_time_ + rotate_time_) {
            ctx_->drive(0.0, direction_ * kRotateSpeed);
        } else {
            ctx_->drive(0.0, 0.0);
            ctx_->event("recovery_done");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { ctx_->drive(0.0, 0.0); }

private:
    static constexpr double kBackupSpeed = 0.15;  // m/s
    static constexpr double kRotateSpeed = 0.8;   // rad/s
    Context* ctx_;
    double backup_time_ = 0.0, rotate_time_ = 0.0, start_ = 0.0, direction_ = -1.0;
};

Pose2D poseParam(rclcpp::Node& n, const std::string& name, std::vector<double> def) {
    const auto v = n.declare_parameter(name, def);
    if (v.size() != 3) throw std::runtime_error("task_bt: " + name + " must be [x, y, theta]");
    return {v[0], v[1], v[2]};
}

}  // namespace

class TaskBtNode : public rclcpp::Node {
public:
    TaskBtNode() : Node("task_bt") {
        ctx_.node = this;
        ctx_.goal_tolerance = declare_parameter("goal_tolerance", 0.15);
        // Latched: the planner always has the current goal, even if it
        // (re)starts after the goal was sent.
        ctx_.goal_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
            "goal_pose", rclcpp::QoS(1).transient_local().reliable());
        ctx_.cmd_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        ctx_.event_pub = create_publisher<std_msgs::msg::String>(
            "task_bt/events", rclcpp::QoS(100).transient_local().reliable());
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "ekf/pose", 20, [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
                const auto& q = m->pose.pose.orientation;
                ctx_.pose = Pose2D{m->pose.pose.position.x, m->pose.pose.position.y,
                                   std::atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)};
            });
        status_sub_ = create_subscription<std_msgs::msg::String>(
            "planner/status", 10,
            [this](std_msgs::msg::String::SharedPtr m) { ctx_.planner_status = m->data; });
        state_sub_ = create_subscription<std_msgs::msg::String>(
            "controller/state", 10,
            [this](std_msgs::msg::String::SharedPtr m) { ctx_.controller_state = m->data; });

        factory_.registerNodeType<NavigateTo>("NavigateTo", &ctx_);
        factory_.registerNodeType<Recover>("Recover", &ctx_);
        factory_.registerNodeType<task_bt::NextWaypoint>("NextWaypoint");
        std::string file = declare_parameter<std::string>("tree_file", "");
        if (file.empty())
            file = ament_index_cpp::get_package_share_directory("task_bt") + "/trees/mission.xml";
        factory_.registerBehaviorTreeFromFile(file);

        auto bb = BT::Blackboard::create();
        bb->set("goal", poseParam(*this, "goal", {10.2, 6.6, 0.0}));
        bb->set("dock", poseParam(*this, "dock", {1.5, 1.5, 0.0}));
        const auto wps = task_bt::posesFromFlat(
            declare_parameter("waypoints", std::vector<double>{5.0, 3.0, 0.0}));
        bb->set("waypoints", wps);
        bb->set("patrol_steps",
                static_cast<int>(wps.size() * declare_parameter("patrol_loops", 1)));
        tree_name_ = declare_parameter<std::string>("tree", "GoTo");
        tree_ = std::make_unique<BT::Tree>(factory_.createTree(tree_name_, bb));

        // Give the localiser and mapper a moment before the first goal.
        start_delay_ = declare_parameter("start_delay", 2.0);
        // Evaluation / tests: wait until this many nodes listen to the events,
        // so mission timing is measured from the real start.
        min_event_subscribers_ =
            static_cast<std::size_t>(declare_parameter("wait_for_event_subscribers", 0));
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { tick(); });
    }

private:
    void tick() {
        if (done_ || !ctx_.pose) return;
        if (!started_) {
            // Start once the planner listens for goals (and any required
            // event listeners are there), then wait start_delay so the
            // localiser and mapper have settled.
            if (ctx_.goal_pub->get_subscription_count() == 0 ||
                ctx_.event_pub->get_subscription_count() < min_event_subscribers_)
                return;
            if (ready_since_ < 0.0) ready_since_ = now().seconds();
            if (now().seconds() - ready_since_ < start_delay_) return;
            started_ = true;
            ctx_.event("mission_start " + tree_name_);
        }
        const BT::NodeStatus s = tree_->tickOnce();
        if (s == BT::NodeStatus::SUCCESS || s == BT::NodeStatus::FAILURE) {
            done_ = true;
            ctx_.event(s == BT::NodeStatus::SUCCESS ? "mission_success" : "mission_failure");
        }
    }

    Context ctx_;
    BT::BehaviorTreeFactory factory_;
    std::unique_ptr<BT::Tree> tree_;
    std::string tree_name_;
    double start_delay_ = 2.0, ready_since_ = -1.0;
    std::size_t min_event_subscribers_ = 0;
    bool started_ = false, done_ = false;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_, state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskBtNode>());
    rclcpp::shutdown();
    return 0;
}
