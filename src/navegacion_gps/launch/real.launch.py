import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, LocalSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_config_file_path(package_share_dir: str, filename: str) -> str:
    package_share_path = Path(package_share_dir)
    default_path = package_share_path / "config" / filename
    try:
        workspace_root = package_share_path.parents[3]
        source_path = workspace_root / "src" / "navegacion_gps" / "config" / filename
        if source_path.parent.exists():
            return str(source_path)
    except IndexError:
        pass
    return str(default_path)


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() == "true"


def _nav2_uses_map_frame(nav2_params_file: str) -> bool:
    use_map = True
    try:
        with open(nav2_params_file, "r", encoding="utf-8") as file_handle:
            data = yaml.safe_load(file_handle) or {}
    except Exception:
        return use_map

    candidate_paths = [
        ["bt_navigator", "ros__parameters", "global_frame"],
        ["behavior_server", "ros__parameters", "global_frame"],
        ["local_costmap", "local_costmap", "ros__parameters", "global_frame"],
        ["global_costmap", "global_costmap", "ros__parameters", "global_frame"],
    ]

    frames = []
    for path in candidate_paths:
        node = data
        valid = True
        for key in path:
            if isinstance(node, dict) and key in node:
                node = node[key]
            else:
                valid = False
                break
        if valid:
            frames.append(str(node).strip().lower())

    if frames:
        return any(frame == "map" for frame in frames)
    return use_map


def _validate_tf_configuration(context, nav2_params_file: str):
    ekf_local_enabled = _as_bool(LaunchConfiguration("ekf_local").perform(context))
    ekf_global_enabled = _as_bool(LaunchConfiguration("ekf_global").perform(context))
    ackermann_enabled = _as_bool(
        LaunchConfiguration("ackermann_odometry").perform(context)
    )
    nav2_requires_map = _nav2_uses_map_frame(nav2_params_file)

    if nav2_requires_map and (not ekf_global_enabled):
        raise RuntimeError(
            "Invalid TF configuration: nav2_no_map_params.yaml is using frame 'map', "
            "but ekf_global:=False removes map->odom publication. "
            "Para modo map: usar `ekf_global:=True`."
        )

    if (not ekf_local_enabled) and (not ackermann_enabled):
        raise RuntimeError(
            "Invalid TF configuration: ekf_local:=False and ackermann_odometry:=False "
            "leave no provider for odom->base_footprint. "
            "Si desactivas `ekf_local`, mantener `ackermann_odometry:=true`."
        )
    return []


def _on_rtk_gate_exit(context, delayed_start_actions):
    return_code = int(LocalSubstitution("event.returncode").perform(context))
    if return_code == 0:
        return list(delayed_start_actions)
    timeout_s = str(LaunchConfiguration("rtk_gate_timeout_s").perform(context))
    return [
        LogInfo(msg=f"RTK gate failed; shutting down launch (timeout_s={timeout_s})"),
        EmitEvent(event=Shutdown(reason=f"RTK gate timeout after {timeout_s}s")),
    ]


def _build_navigation_startup(
    context,
    delayed_start_actions,
    gps_topic,
    rtk_status_topic,
    fix_type_topic,
    required_fix_type,
):
    require_gate = _as_bool(
        LaunchConfiguration("require_rtk_before_navigation").perform(context)
    )
    if not require_gate:
        return [TimerAction(period=5.0, actions=list(delayed_start_actions))]

    rtk_gate_cmd = Node(
        package="navegacion_gps",
        executable="rtk_start_gate",
        name="rtk_start_gate",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"gps_topic": gps_topic},
            {"rtk_status_topic": rtk_status_topic},
            {"fix_type_topic": fix_type_topic},
            {"required_fix_type": required_fix_type},
            {
                "timeout_s": ParameterValue(
                    LaunchConfiguration("rtk_gate_timeout_s"),
                    value_type=float,
                )
            },
        ],
    )
    return [
        rtk_gate_cmd,
        RegisterEventHandler(
            OnProcessExit(
                target_action=rtk_gate_cmd,
                on_exit=[
                    OpaqueFunction(
                        function=_on_rtk_gate_exit,
                        kwargs={"delayed_start_actions": list(delayed_start_actions)},
                    )
                ],
            )
        ),
    ]


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")

    zones_geojson_path = _resolve_config_file_path(gps_wpf_dir, "no_go_zones.geojson")
    keepout_mask_image_path = _resolve_config_file_path(gps_wpf_dir, "keepout_mask.pgm")
    keepout_mask_yaml_path = _resolve_config_file_path(gps_wpf_dir, "keepout_mask.yaml")
    rl_params_file = _resolve_config_file_path(gps_wpf_dir, "dual_ekf_navsat_params.yaml")
    nav2_params_file = _resolve_config_file_path(gps_wpf_dir, "nav2_no_map_params.yaml")
    rviz_default = _resolve_config_file_path(gps_wpf_dir, "rviz_nav2_full.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_robot_state_publisher = LaunchConfiguration("use_robot_state_publisher")
    custom_urdf = LaunchConfiguration("custom_urdf")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")
    use_gazebo_utils = LaunchConfiguration("use_gazebo_utils")
    gps_topic = LaunchConfiguration("gps_topic")
    rtk_status_topic = LaunchConfiguration("rtk_status_topic")
    fix_type_topic = LaunchConfiguration("fix_type_topic")
    required_fix_type = LaunchConfiguration("required_fix_type")
    map_frame = LaunchConfiguration("map_frame")
    zones_manager = LaunchConfiguration("zones_manager")
    ackermann_odometry = LaunchConfiguration("ackermann_odometry")
    ekf_local = LaunchConfiguration("ekf_local")
    ekf_global = LaunchConfiguration("ekf_global")
    ukf = LaunchConfiguration("ukf")
    localization_filter_executable = PythonExpression(
        ["'ukf_node' if '", ukf, "'.lower() == 'true' else 'ekf_node'"]
    )
    resolved_map_frame = PythonExpression(
        [
            "('",
            map_frame,
            "'.strip().lower() if '",
            map_frame,
            "'.strip().lower() not in ('', 'auto') else ('map' if '",
            ekf_global,
            "'.lower() == 'true' else 'odom'))",
        ]
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock if true",
    )
    declare_use_robot_state_publisher_cmd = DeclareLaunchArgument(
        "use_robot_state_publisher",
        default_value="True",
        description="Publish TF using robot_state_publisher",
    )
    declare_custom_urdf_cmd = DeclareLaunchArgument(
        "custom_urdf",
        default_value=os.path.join(gps_wpf_dir, "models", "cuatri_real.urdf"),
        description="Path to custom URDF for TF tree",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Whether to start RVIZ",
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_default,
        description="Path to the RViz config file",
    )
    declare_use_navsat_cmd = DeclareLaunchArgument(
        "use_navsat",
        default_value="True",
        description="Deprecated: navsat_transform is always started in real.launch",
    )
    declare_use_collision_monitor_cmd = DeclareLaunchArgument(
        "use_collision_monitor",
        default_value="True",
        description="Whether to start collision monitor",
    )
    declare_use_gazebo_utils_cmd = DeclareLaunchArgument(
        "use_gazebo_utils",
        default_value="False",
        description="Whether to run gazebo_utils for frame normalization",
    )
    declare_gps_topic_cmd = DeclareLaunchArgument(
        "gps_topic",
        default_value="/gps/fix",
        description="GPS topic used by web console backend/gateway",
    )
    declare_rtk_status_topic_cmd = DeclareLaunchArgument(
        "rtk_status_topic",
        default_value="/gps/rtk_status",
        description="RTK status topic used to gate navigation startup",
    )
    declare_fix_type_topic_cmd = DeclareLaunchArgument(
        "fix_type_topic",
        default_value="/gps/fix_type",
        description="GPS fix_type topic used to gate navigation startup",
    )
    declare_required_fix_type_cmd = DeclareLaunchArgument(
        "required_fix_type",
        default_value="DGPS",
        description=(
            "Minimum GPS fix quality required to start localization and Nav2 "
            "(examples: 3D_FIX,3D,DGPS,RTK_FLOAT,RTK,RTK_FIX,RTK_FIXED)"
        ),
    )
    declare_map_frame_cmd = DeclareLaunchArgument(
        "map_frame",
        default_value="auto",
        description="Navigation frame: auto (map when ekf_global=true, odom when false) or explicit",
    )
    declare_zones_manager_cmd = DeclareLaunchArgument(
        "zones_manager",
        default_value="true",
        description="Enable zones_manager node",
    )
    declare_ackermann_odometry_cmd = DeclareLaunchArgument(
        "ackermann_odometry",
        default_value="true",
        description="Enable ackermann_odometry node",
    )
    declare_ekf_local_cmd = DeclareLaunchArgument(
        "ekf_local",
        default_value="true",
        description="Enable local robot_localization filter node",
    )
    declare_ekf_global_cmd = DeclareLaunchArgument(
        "ekf_global",
        default_value="true",
        description="Enable global robot_localization filter node",
    )
    declare_ukf_cmd = DeclareLaunchArgument(
        "ukf",
        default_value="True",
        description="Use UKF for local/global robot_localization filters; if false, use EKF",
    )
    declare_require_rtk_before_navigation_cmd = DeclareLaunchArgument(
        "require_rtk_before_navigation",
        default_value="true",
        description=(
            "Wait for the configured minimum GPS fix quality before starting "
            "localization and Nav2"
        ),
    )
    declare_rtk_gate_timeout_s_cmd = DeclareLaunchArgument(
        "rtk_gate_timeout_s",
        default_value="60.0",
        description="Wall-clock timeout in seconds for the RTK startup gate",
    )

    # Block 5 - Navigation
    nav2_only_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gps_wpf_dir, "launch", "nav2_only.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map_frame": resolved_map_frame,
            "use_keepout": PythonExpression(["'", zones_manager, "'.lower() == 'true'"]),
            "use_collision_monitor": use_collision_monitor,
            "use_rviz": use_rviz,
            "rviz_config": rviz_config,
            "use_robot_state_publisher": use_robot_state_publisher,
            "custom_urdf": custom_urdf,
        }.items(),
    )

    # Block 4 - Localization
    ekf_odom_cmd = Node(
        package="robot_localization",
        executable=localization_filter_executable,
        name="ekf_filter_node_odom",
        output="screen",
        condition=IfCondition(ekf_local),
        parameters=[
            rl_params_file,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[("odometry/filtered", "odometry/local")],
    )
    ackermann_odometry_cmd = Node(
        package="navegacion_gps",
        executable="ackermann_odometry",
        name="ackermann_odometry",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", ackermann_odometry, "'.lower() == 'true'"])
        ),
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"telemetry_topic": "/controller/drive_telemetry"},
            {"odom_topic": "/wheel/odometry"},
            {"periodic_log_enabled": False},
            {
                "publish_odom_tf": ParameterValue(
                    PythonExpression(["'", ekf_local, "'.lower() != 'true'"]),
                    value_type=bool,
                )
            },
        ],
    )
    ekf_map_cmd = Node(
        package="robot_localization",
        executable=localization_filter_executable,
        name="ekf_filter_node_map",
        output="screen",
        condition=IfCondition(ekf_global),
        parameters=[
            rl_params_file,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )
    navsat_transform_cmd = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        condition=IfCondition(ekf_global),
        parameters=[
            rl_params_file,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/local"),
        ],
    )
    # Block 6 - Navigation Support And Web
    zones_manager_cmd = Node(
        package="navegacion_gps",
        executable="zones_manager",
        name="zones_manager",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", zones_manager, "'.lower() == 'true'"])
        ),
        parameters=[
            {
                "fromll_service": "/fromLL",
                "fromll_service_fallback": "/navsat_transform/fromLL",
                "fromll_wait_timeout_s": 2.0,
                "load_map_service": "/keepout_filter_mask_server/load_map",
                "set_geojson_service": "/zones_manager/set_geojson",
                "get_state_service": "/zones_manager/get_state",
                "reload_from_disk_service": "/zones_manager/reload_from_disk",
                "map_frame": resolved_map_frame,
                "fromll_target_frame": resolved_map_frame,
                "geojson_file": zones_geojson_path,
                "mask_image_file": keepout_mask_image_path,
                "mask_yaml_file": keepout_mask_yaml_path,
                "buffer_margin_m": 0.8,
                "degrade_enabled": True,
                "degrade_radius_m": 1.5,
                "degrade_edge_cost": 40,
                "degrade_min_cost": 1,
                "degrade_use_l2": True,
                "mask_origin_mode": "explicit",
                "mask_origin_x": -150.0,
                "mask_origin_y": -150.0,
                "mask_width": 3000,
                "mask_height": 3000,
                "mask_resolution": 0.1,
            }
        ],
    )
    nav_snapshot_server_cmd = Node(
        package="navegacion_gps",
        executable="nav_snapshot_server",
        name="nav_snapshot_server",
        output="screen",
        parameters=[
            {
                "get_snapshot_service": "/nav_snapshot_server/get_nav_snapshot",
                "local_costmap_topic": "/local_costmap/costmap",
                "global_costmap_topic": "/global_costmap/costmap",
                "keepout_mask_topic": "/keepout_filter_mask",
                "local_footprint_topic": "/local_costmap/published_footprint",
                "stop_zone_topic": "/stop_zone",
                "collision_polygons_topic": "/collision_monitor/polygons",
                "scan_topic": "/scan",
                "plan_topic": "/plan",
                "base_frame": "base_footprint",
                "snapshot_extent_m": 30.0,
                "snapshot_size_px": 512,
                "snapshot_global_inset_px": 160,
                "snapshot_timeout_ms": 500,
            }
        ],
    )
    nav_observability_cmd = Node(
        package="navegacion_gps",
        executable="nav_observability",
        name="nav_observability",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "publish_hz": 2.0,
            }
        ],
    )

    # Block 7 - Optional Runtime Utilities
    gazebo_utils_cmd = Node(
        package="navegacion_gps",
        executable="gazebo_utils",
        name="gazebo_utils",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"imu_in_topic": "/imu/data_raw", "imu_out_topic": "/imu/data"},
            {"gps_in_topic": "/gps/fix_raw", "gps_out_topic": "/gps/fix"},
            {"lidar_in_topic": "/scan_3d_raw", "lidar_out_topic": "/scan_3d"},
            {"odom_in_topic": "/odom_raw", "odom_out_topic": "/odom"},
            {"imu_frame_id": "imu_link"},
            {"gps_frame_id": "gps_link"},
            {"lidar_frame_id": "lidar_link"},
            {"odom_frame_id": "odom"},
            {"base_link_frame_id": "base_footprint"},
            {"enable_cmd_vel_final_bridge": False},
        ],
        condition=IfCondition(use_gazebo_utils),
    )

    delayed_start_actions = [
        # Block 4 - Localization
        ackermann_odometry_cmd,
        ekf_odom_cmd,
        ekf_map_cmd,
        navsat_transform_cmd,
        # Block 5 - Navigation
        nav2_only_cmd,
        # Block 6 - Navigation Support And Web
        zones_manager_cmd,
        nav_snapshot_server_cmd,
        nav_observability_cmd,
        # Block 7 - Optional Runtime Utilities
        gazebo_utils_cmd,
    ]
    delayed_start_cmd = OpaqueFunction(
        function=_build_navigation_startup,
        kwargs={
            "delayed_start_actions": delayed_start_actions,
            "gps_topic": gps_topic,
            "rtk_status_topic": rtk_status_topic,
            "fix_type_topic": fix_type_topic,
            "required_fix_type": required_fix_type,
        },
    )

    ld = LaunchDescription()
    # Block 1 - Launch Arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_robot_state_publisher_cmd)
    ld.add_action(declare_custom_urdf_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_navsat_cmd)
    ld.add_action(declare_use_collision_monitor_cmd)
    ld.add_action(declare_use_gazebo_utils_cmd)
    ld.add_action(declare_gps_topic_cmd)
    ld.add_action(declare_rtk_status_topic_cmd)
    ld.add_action(declare_fix_type_topic_cmd)
    ld.add_action(declare_required_fix_type_cmd)
    ld.add_action(declare_map_frame_cmd)
    ld.add_action(declare_zones_manager_cmd)
    ld.add_action(declare_ackermann_odometry_cmd)
    ld.add_action(declare_ekf_local_cmd)
    ld.add_action(declare_ekf_global_cmd)
    ld.add_action(declare_ukf_cmd)
    ld.add_action(declare_require_rtk_before_navigation_cmd)
    ld.add_action(declare_rtk_gate_timeout_s_cmd)
    # Block 2 - Launch Validation
    ld.add_action(
        OpaqueFunction(
            function=lambda context: _validate_tf_configuration(context, nav2_params_file)
        )
    )
    ld.add_action(delayed_start_cmd)

    return ld
