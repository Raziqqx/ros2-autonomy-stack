"""Simulator + EKF localisation + occupancy-grid mapping."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("autonomy_bringup")
    map_yaml = LaunchConfiguration("map_yaml")
    sim_cfg = PathJoinSubstitution([share, "config", "sim.yaml"])
    loc_cfg = PathJoinSubstitution([share, "config", "localization.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_yaml", default_value=PathJoinSubstitution([share, "maps", "warehouse.yaml"])),
        DeclareLaunchArgument("seed", default_value="1"),
        DeclareLaunchArgument("use_icp", default_value="true"),
        DeclareLaunchArgument("use_imu", default_value="true"),
        Node(package="sim2d", executable="sim2d_node", name="sim2d", output="screen",
             parameters=[sim_cfg, {"map_yaml": map_yaml,
                                   "seed": LaunchConfiguration("seed")}]),
        Node(package="ekf_localization", executable="ekf_localization_node",
             name="ekf_localization", output="screen",
             parameters=[loc_cfg, {"map_yaml": map_yaml,
                                   "use_icp": LaunchConfiguration("use_icp"),
                                   "use_imu": LaunchConfiguration("use_imu")}]),
        Node(package="grid_mapping", executable="grid_mapping_node", name="grid_mapping",
             output="screen", parameters=[loc_cfg]),
    ])
