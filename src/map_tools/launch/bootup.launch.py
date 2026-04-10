from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim = LaunchConfiguration("use_sim")
    ws_host = LaunchConfiguration("ws_host")
    ws_port = LaunchConfiguration("ws_port")
    gps_topic = LaunchConfiguration("gps_topic")
    map_frame = LaunchConfiguration("map_frame")
    fromll_output_frame = LaunchConfiguration("fromll_output_frame")
    cmd_vel_safe_topic = LaunchConfiguration("cmd_vel_safe_topic")
    cmd_vel_final_topic = LaunchConfiguration("cmd_vel_final_topic")
    teleop_cmd_topic = LaunchConfiguration("teleop_cmd_topic")
    approx_fromll_fallback_enabled = LaunchConfiguration(
        "approx_fromll_fallback_enabled"
    )
    approx_fromll_datum_lat = LaunchConfiguration("approx_fromll_datum_lat")
    approx_fromll_datum_lon = LaunchConfiguration("approx_fromll_datum_lon")
    approx_fromll_datum_yaw_deg = LaunchConfiguration("approx_fromll_datum_yaw_deg")
    processes_file = LaunchConfiguration("processes_file")
    process_file_logging = LaunchConfiguration("process_file_logging")

    controller_launch = (
        Path(get_package_share_directory("controller_server"))
        / "launch"
        / "controller_server.launch.py"
    )
    controller_sim_launch = (
        Path(get_package_share_directory("controller_server"))
        / "launch"
        / "controller_server_sim.launch.py"
    )
    default_processes_file = (
        Path(get_package_share_directory("utilities")) / "config" / "process_list.json"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="false"),
            DeclareLaunchArgument("ws_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("ws_port", default_value="8766"),
            DeclareLaunchArgument("gps_topic", default_value="/gps/fix"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("fromll_output_frame", default_value="map"),
            DeclareLaunchArgument("cmd_vel_safe_topic", default_value="/cmd_vel_safe"),
            DeclareLaunchArgument("cmd_vel_final_topic", default_value="/cmd_vel_final"),
            DeclareLaunchArgument("teleop_cmd_topic", default_value="/cmd_vel_teleop"),
            DeclareLaunchArgument(
                "approx_fromll_fallback_enabled", default_value="false"
            ),
            DeclareLaunchArgument("approx_fromll_datum_lat", default_value="nan"),
            DeclareLaunchArgument("approx_fromll_datum_lon", default_value="nan"),
            DeclareLaunchArgument("approx_fromll_datum_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("processes_file", default_value=str(default_processes_file)),
            DeclareLaunchArgument("process_file_logging", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(controller_launch)),
                condition=UnlessCondition(use_sim),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(controller_sim_launch)),
                condition=IfCondition(use_sim),
            ),
            Node(
                package="navegacion_gps",
                executable="nav_command_server",
                name="nav_command_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim, value_type=bool),
                        "fromll_service": "/fromLL",
                        "fromll_service_fallback": "/navsat_transform/fromLL",
                        "fromll_wait_timeout_s": 2.0,
                        "fromll_output_frame": fromll_output_frame,
                        "map_frame": map_frame,
                        "gps_topic": gps_topic,
                        "cmd_vel_safe_topic": cmd_vel_safe_topic,
                        "cmd_vel_final_topic": cmd_vel_final_topic,
                        "forward_cmd_vel_safe_without_goal": True,
                        "brake_topic": cmd_vel_safe_topic,
                        "manual_cmd_topic": cmd_vel_safe_topic,
                        "teleop_cmd_topic": teleop_cmd_topic,
                        "brake_publish_count": 5,
                        "brake_publish_interval_s": 0.1,
                        "manual_cmd_timeout_s": 0.4,
                        "manual_watchdog_hz": 10.0,
                        "nav_telemetry_hz": 5.0,
                        "telemetry_topic": "/nav_command_server/telemetry",
                        "event_topic": "/nav_command_server/events",
                        "set_goal_service": "/nav_command_server/set_goal_ll",
                        "cancel_goal_service": "/nav_command_server/cancel_goal",
                        "brake_service": "/nav_command_server/brake",
                        "set_manual_mode_service": "/nav_command_server/set_manual_mode",
                        "get_state_service": "/nav_command_server/get_state",
                        "approx_fromll_fallback_enabled": ParameterValue(
                            approx_fromll_fallback_enabled,
                            value_type=bool,
                        ),
                        "approx_fromll_datum_lat": ParameterValue(
                            approx_fromll_datum_lat,
                            value_type=float,
                        ),
                        "approx_fromll_datum_lon": ParameterValue(
                            approx_fromll_datum_lon,
                            value_type=float,
                        ),
                        "approx_fromll_datum_yaw_deg": ParameterValue(
                            approx_fromll_datum_yaw_deg,
                            value_type=float,
                        ),
                        "approx_fromll_zero_threshold_m": 1.0e-3,
                        "approx_fromll_min_distance_for_fallback_m": 0.5,
                    }
                ],
            ),
            Node(
                package="map_tools",
                executable="web_zone_server",
                name="web_zone_server",
                output="screen",
                parameters=[
                    {
                        "ws_host": ws_host,
                        "ws_port": ws_port,
                        "gps_topic": gps_topic,
                        "map_frame": map_frame,
                        "teleop_cmd_topic": teleop_cmd_topic,
                    }
                ],
            ),
            Node(
                package="utilities",
                executable="process_executor",
                name="process_executor",
                output="screen",
                parameters=[
                    {
                        "processes_file": processes_file,
                        "file_logging": process_file_logging,
                    }
                ],
            ),
        ]
    )
