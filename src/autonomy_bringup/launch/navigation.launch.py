"""Full stack: sim2d + EKF localisation + mapping + global planner + local
controller + behaviour tree, optionally with the run recorder (evaluation)
and a rosbag recording.

Example:
  ros2 launch autonomy_bringup navigation.launch.py planner:=thetastar \\
      controller:=dwa tree:=Patrol
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = {
    "seed": "1",
    "planner": "astar",            # astar | thetastar
    "controller": "pure_pursuit",  # pure_pursuit | dwa
    "tree": "GoTo",                # GoTo | Dock | Patrol
    "goal": "10.2,6.6,0.0",
    "noise_scale": "1.0",          # multiplies every sim sensor error
    "clutter_count": "0",          # random boxes not in the prior map
    "drop_after_m": "-1.0",        # drop a wall ahead after this distance (<0: off)
    "record_dir": "",              # non-empty: run the recorder, stop when it exits
    "mission_timeout": "120.0",
    "record_bag": "false",
    "rviz": "false",
    "wait_for_events": "0",        # task_bt waits for this many event listeners
}


def _floats(text):
    return [float(v) for v in text.split(",")]


def _setup(context):
    a = {k: context.launch_configurations[k] for k in ARGS}
    share = FindPackageShare("autonomy_bringup").perform(context)
    map_yaml = f"{share}/maps/warehouse.yaml"
    sim_cfg, loc_cfg, nav_cfg = (f"{share}/config/{n}.yaml" for n in ("sim", "localization", "nav"))
    goal = _floats(a["goal"])
    dock = [1.5, 1.5, 0.0]
    # The recorder must see mission_start, so task_bt waits for it.
    wait_events = max(int(a["wait_for_events"]), 1 if a["record_dir"] else 0)
    actions = [
        Node(package="sim2d", executable="sim2d_node", name="sim2d", output="log",
             parameters=[sim_cfg, {"map_yaml": map_yaml, "seed": int(a["seed"]),
                                   "noise_scale": float(a["noise_scale"]),
                                   "clutter_count": int(a["clutter_count"]),
                                   "clutter_keepout": goal[:2] + dock[:2],
                                   "drop_after_m": float(a["drop_after_m"])}]),
        Node(package="ekf_localization", executable="ekf_localization_node",
             name="ekf_localization", output="log",
             parameters=[loc_cfg, {"map_yaml": map_yaml}]),
        Node(package="grid_mapping", executable="grid_mapping_node", name="grid_mapping",
             output="log", parameters=[loc_cfg]),
        Node(package="global_planner", executable="global_planner_node", name="global_planner",
             output="log", parameters=[nav_cfg, {"map_yaml": map_yaml, "planner": a["planner"]}]),
        Node(package="local_controller", executable="local_controller_node",
             name="local_controller", output="log",
             parameters=[nav_cfg, {"controller": a["controller"]}]),
        Node(package="task_bt", executable="task_bt_node", name="task_bt", output="screen",
             parameters=[nav_cfg, {"tree": a["tree"], "goal": goal, "dock": dock,
                                    "wait_for_event_subscribers": wait_events}]),
    ]
    if a["record_dir"]:
        recorder = Node(
            package="autonomy_bringup", executable="nav_recorder.py", name="nav_recorder",
            output="screen",
            parameters=[{"out_dir": a["record_dir"], "goal": goal[:2],
                         "timeout": float(a["mission_timeout"])}])
        actions += [recorder, RegisterEventHandler(OnProcessExit(
            target_action=recorder, on_exit=[EmitEvent(event=Shutdown(reason="run recorded"))]))]
        if a["record_bag"] == "true":
            actions.append(ExecuteProcess(cmd=[
                "ros2", "bag", "record", "-o", f"{a['record_dir']}/bag", "/ground_truth",
                "/ekf/pose", "/plan", "/scan", "/map", "/cmd_vel", "/task_bt/events",
                "/sim/world"], output="log"))
    if a["rviz"] == "true":
        actions.append(Node(package="rviz2", executable="rviz2",
                            arguments=["-d", f"{share}/config/autonomy.rviz"]))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()]
        + [OpaqueFunction(function=_setup)])
