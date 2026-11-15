// ROS-free parts of the task behaviour tree: the Pose2D port type and the
// NextWaypoint node (cycles through the patrol waypoints).
#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <string>
#include <vector>

namespace task_bt {

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

// Builds poses from a flat [x, y, theta, x, y, theta, ...] parameter.
inline std::vector<Pose2D> posesFromFlat(const std::vector<double>& v) {
    std::vector<Pose2D> out;
    for (std::size_t i = 0; i + 2 < v.size(); i += 3) out.push_back({v[i], v[i + 1], v[i + 2]});
    return out;
}

// Writes the next waypoint (in order, wrapping around) to `goal`.
class NextWaypoint : public BT::SyncActionNode {
public:
    NextWaypoint(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::vector<Pose2D>>("waypoints"), BT::OutputPort<Pose2D>("goal")};
    }

    BT::NodeStatus tick() override {
        const auto wps = getInput<std::vector<Pose2D>>("waypoints");
        if (!wps || wps->empty()) return BT::NodeStatus::FAILURE;
        setOutput("goal", (*wps)[next_ % wps->size()]);
        ++next_;
        return BT::NodeStatus::SUCCESS;
    }

private:
    std::size_t next_ = 0;
};

}  // namespace task_bt

namespace BT {
// "x;y;theta" in XML, e.g. goal="10.2;6.6;0".
template <>
inline task_bt::Pose2D convertFromString(StringView str) {
    const auto parts = splitString(str, ';');
    if (parts.size() != 3) throw RuntimeError("Pose2D expects \"x;y;theta\"");
    return {convertFromString<double>(parts[0]), convertFromString<double>(parts[1]),
            convertFromString<double>(parts[2])};
}
}  // namespace BT
