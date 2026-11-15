// Mission tree logic with mock NavigateTo / Recover leaves (no ROS).
#include <gtest/gtest.h>

#include <deque>

#include "task_bt/bt_common.hpp"

using task_bt::Pose2D;

namespace {

struct Script {
    std::deque<BT::NodeStatus> nav_results;  // consumed per NavigateTo tick
    std::vector<Pose2D> goals;               // goals NavigateTo was asked for
    int recoveries = 0;
};

class MockNavigate : public BT::SyncActionNode {
public:
    MockNavigate(const std::string& n, const BT::NodeConfig& c, Script* s)
        : BT::SyncActionNode(n, c), s_(s) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<Pose2D>("goal"), BT::InputPort<double>("timeout")};
    }
    BT::NodeStatus tick() override {
        s_->goals.push_back(getInput<Pose2D>("goal").value());
        if (s_->nav_results.empty()) return BT::NodeStatus::SUCCESS;
        const auto r = s_->nav_results.front();
        s_->nav_results.pop_front();
        return r;
    }

private:
    Script* s_;
};

class MockRecover : public BT::SyncActionNode {
public:
    MockRecover(const std::string& n, const BT::NodeConfig& c, Script* s)
        : BT::SyncActionNode(n, c), s_(s) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("backup_distance"), BT::InputPort<double>("rotate_angle")};
    }
    BT::NodeStatus tick() override {
        ++s_->recoveries;
        return BT::NodeStatus::SUCCESS;
    }

private:
    Script* s_;
};

BT::Tree makeTree(const std::string& id, Script& s) {
    BT::BehaviorTreeFactory f;
    f.registerNodeType<MockNavigate>("NavigateTo", &s);
    f.registerNodeType<MockRecover>("Recover", &s);
    f.registerNodeType<task_bt::NextWaypoint>("NextWaypoint");
    f.registerBehaviorTreeFromFile(TREE_FILE);
    auto bb = BT::Blackboard::create();
    bb->set("goal", Pose2D{5, 1, 0});
    bb->set("dock", Pose2D{1, 1, 0});
    bb->set("waypoints", std::vector<Pose2D>{{2, 2, 0}, {3, 3, 0}});
    bb->set("patrol_steps", 4);
    return f.createTree(id, bb);
}

}  // namespace

TEST(MissionTree, GoToSucceedsWithoutRecovery) {
    Script s;
    auto tree = makeTree("GoTo", s);
    EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(s.recoveries, 0);
    ASSERT_EQ(s.goals.size(), 1u);
    EXPECT_DOUBLE_EQ(s.goals[0].x, 5.0);
}

TEST(MissionTree, FailureTriggersRecoveryThenRetry) {
    Script s;
    s.nav_results = {BT::NodeStatus::FAILURE, BT::NodeStatus::FAILURE, BT::NodeStatus::SUCCESS};
    auto tree = makeTree("GoTo", s);
    EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(s.recoveries, 2);
    EXPECT_EQ(s.goals.size(), 3u);
}

TEST(MissionTree, GivesUpAfterFourAttempts) {
    Script s;
    s.nav_results.assign(10, BT::NodeStatus::FAILURE);
    auto tree = makeTree("Dock", s);
    EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
    EXPECT_EQ(s.goals.size(), 4u);
    EXPECT_DOUBLE_EQ(s.goals[0].x, 1.0);  // dock pose
}

TEST(MissionTree, PatrolVisitsWaypointsInOrderThenDocks) {
    Script s;
    s.nav_results = {BT::NodeStatus::SUCCESS, BT::NodeStatus::FAILURE};  // 2nd leg needs a retry
    auto tree = makeTree("Patrol", s);
    EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
    const std::vector<double> xs{2, 3, 3, 2, 3, 1};  // wp0, wp1 (failed), wp1, wp0, wp1, dock
    ASSERT_EQ(s.goals.size(), xs.size());
    for (std::size_t i = 0; i < xs.size(); ++i) EXPECT_DOUBLE_EQ(s.goals[i].x, xs[i]) << i;
    EXPECT_EQ(s.recoveries, 1);
}

TEST(Pose2D, ParsesFromXmlString) {
    const auto p = BT::convertFromString<Pose2D>("1.5;-2;0.25");
    EXPECT_DOUBLE_EQ(p.x, 1.5);
    EXPECT_DOUBLE_EQ(p.y, -2.0);
    EXPECT_DOUBLE_EQ(p.theta, 0.25);
    EXPECT_THROW(BT::convertFromString<Pose2D>("1;2"), BT::RuntimeError);
}
