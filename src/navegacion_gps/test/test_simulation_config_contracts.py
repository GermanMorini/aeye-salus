from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_localization_v2_launch_uses_fixed_dual_ekf_odom_node() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "localization_v2.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert "dual_ekf_navsat_params.yaml" in launch_contents
    assert 'name="ekf_filter_node_odom"' in launch_contents
    assert "condition=IfCondition(ekf_local)" in launch_contents
    assert 'DeclareLaunchArgument("ekf_local", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("ekf_global", default_value="False")' in launch_contents
    assert 'DeclareLaunchArgument("ukf", default_value="False")' in launch_contents
    assert 'executable="ackermann_odometry"' in launch_contents
    assert "publish_odom_tf = ParameterValue(" in launch_contents
    assert '"publish_odom_tf": publish_odom_tf' in launch_contents
    assert 'executable="pixhawk_odometry"' in launch_contents
    assert 'name="ekf_filter_node_map"' in launch_contents
    assert 'name="navsat_transform"' in launch_contents
    assert '("odometry/filtered", "/odometry/global")' in launch_contents
    assert "condition=IfCondition(ekf_global)" in launch_contents
    assert "localization_filter_executable = PythonExpression(" in launch_contents
    assert "\"'ukf_node' if '\"" in launch_contents
    assert "\"'.lower() == 'true' else 'ekf_node'\"" in launch_contents
    assert 'DeclareLaunchArgument("pixhawk_gps_topic", default_value="/gps/fix")' in launch_contents
    assert 'default_value="/odometry/pixhawk"' in launch_contents
    assert '"gps_topic": pixhawk_gps_topic' in launch_contents
    assert '"imu_topic": imu_topic' in launch_contents
    assert "pixhawk_input_odom_topic" not in launch_contents
    assert 'DeclareLaunchArgument(\n                "localization_params_file",' not in launch_contents
    assert 'DeclareLaunchArgument(\n                "ekf_node_name",' not in launch_contents


def test_nav_local_v2_does_not_use_frame_override_overlay() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "nav_local_v2.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert "nav2_no_map_local_frame_overrides.yaml" not in launch_contents
    assert "configured_nav2_frame_overrides" not in launch_contents
    assert "nav2_local_v2_keepout_overrides.yaml" not in launch_contents
    assert "nav2_local_v2_no_keepout_overrides.yaml" not in launch_contents
    assert "configured_nav2_overrides" not in launch_contents


def test_nav2_only_launch_disables_velocity_smoother() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "nav2_only.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert '"use_velocity_smoother": "False"' in launch_contents


def test_nav2_only_launch_defaults_keepout_mask_frame_to_odom() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "nav2_only.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "map_frame",' in launch_contents
    assert 'default_value="odom"' in launch_contents
    assert '"frame_id": resolved_map_frame' in launch_contents
    assert "not in ('', 'auto') else 'odom'" in launch_contents


def test_nav2_only_launch_exposes_use_keepout_toggle() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "nav2_only.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "use_keepout",' in launch_contents
    assert 'default_value="True"' in launch_contents
    assert "condition=IfCondition(use_keepout)" in launch_contents
    assert (
        '"local_costmap.local_costmap.ros__parameters.keepout_filter.enabled": use_keepout'
        in launch_contents
    )
    assert (
        '"global_costmap.global_costmap.ros__parameters.keepout_filter.enabled": use_keepout'
        in launch_contents
    )


def test_nav2_only_launch_delays_keepout_lifecycle_manager_start() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "nav2_only.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "keepout_lifecycle_start_delay_s",' in launch_contents
    assert 'default_value="2.0"' in launch_contents
    assert "keepout_lifecycle_cmd = TimerAction(" in launch_contents
    assert "period=keepout_lifecycle_start_delay_s" in launch_contents
    assert 'name="lifecycle_manager_keepout_filters"' in launch_contents


def test_keepout_filters_v2_launch_delays_keepout_lifecycle_manager_start() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "keepout_filters_v2.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n                "keepout_lifecycle_start_delay_s", default_value="2.0"' in launch_contents
    assert "TimerAction(" in launch_contents
    assert "period=keepout_lifecycle_start_delay_s" in launch_contents
    assert 'name="lifecycle_manager_keepout_filters"' in launch_contents


def test_nav2_no_map_direct_frame_values() -> None:
    nav2_config_path = PACKAGE_ROOT / "config" / "nav2_no_map_params.yaml"
    nav2_config_contents = nav2_config_path.read_text(encoding="utf-8")

    assert "bt_navigator:" in nav2_config_contents
    assert "behavior_server:" in nav2_config_contents
    assert "global_costmap:" in nav2_config_contents
    assert "global_frame: map" in nav2_config_contents


def test_nav2_no_map_follow_path_uses_less_aggressive_ackermann_tuning() -> None:
    nav2_config_path = PACKAGE_ROOT / "config" / "nav2_no_map_params.yaml"
    nav2_config_contents = nav2_config_path.read_text(encoding="utf-8")

    assert 'plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"' in (
        nav2_config_contents
    )
    assert "desired_linear_vel: 0.8" in nav2_config_contents
    assert "lookahead_dist: 2.0" in nav2_config_contents
    assert "min_lookahead_dist: 0.8" in nav2_config_contents
    assert "max_lookahead_dist: 2.5" in nav2_config_contents
    assert "lookahead_time: 1.2" in nav2_config_contents
    assert "use_velocity_scaled_lookahead_dist: true" in nav2_config_contents
    assert "min_approach_linear_velocity: 0.4" in nav2_config_contents
    assert "regulated_linear_scaling_min_speed: 0.4" in nav2_config_contents


def test_navigate_through_poses_prunes_passed_goals_with_radius_matching_goal_tolerance() -> None:
    nav2_config_path = PACKAGE_ROOT / "config" / "nav2_no_map_params.yaml"
    bt_xml_path = (
        PACKAGE_ROOT / "config" / "navigate_through_poses_w_replanning_and_recovery_no_spin.xml"
    )

    nav2_config_contents = nav2_config_path.read_text(encoding="utf-8")
    bt_xml_contents = bt_xml_path.read_text(encoding="utf-8")

    assert "xy_goal_tolerance: 1.2" in nav2_config_contents
    assert 'RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="1.2"' in bt_xml_contents


def test_navigate_to_pose_bt_only_uses_backup_for_follow_path_recovery() -> None:
    bt_xml_path = (
        PACKAGE_ROOT / "config" / "navigate_to_pose_w_replanning_and_recovery_no_spin.xml"
    )
    bt_xml_contents = bt_xml_path.read_text(encoding="utf-8")

    assert '<Sequence name="FollowPathRecoveryActions">' in bt_xml_contents
    assert '<BackUp backup_dist="2.0" backup_speed="1.2" />' in bt_xml_contents
    assert '<ReactiveFallback name="FallbackRecoveries">' not in bt_xml_contents
    assert '<Sequence name="WaitAndReplan">' not in bt_xml_contents


def test_navigate_through_poses_bt_only_uses_backup_for_follow_path_recovery() -> None:
    bt_xml_path = (
        PACKAGE_ROOT / "config" / "navigate_through_poses_w_replanning_and_recovery_no_spin.xml"
    )
    bt_xml_contents = bt_xml_path.read_text(encoding="utf-8")

    assert '<Sequence name="FollowPathRecoveryActions">' in bt_xml_contents
    assert '<BackUp backup_dist="2.0" backup_speed="1.2"/>' in bt_xml_contents
    assert '<ReactiveFallback name="RecoveryFallback">' not in bt_xml_contents
    assert '<RoundRobin name="RecoveryActions">' not in bt_xml_contents


def test_dual_ekf_local_uses_wheel_and_pixhawk_odometry_topics() -> None:
    ekf_config_path = PACKAGE_ROOT / "config" / "dual_ekf_navsat_params.yaml"
    ekf_config_contents = ekf_config_path.read_text(encoding="utf-8")

    assert "ekf_filter_node_odom:" in ekf_config_contents
    assert "/wheel/odometry" in ekf_config_contents
    assert "ekf_filter_node_map:" in ekf_config_contents
    assert "odom1: /odometry/gps" in ekf_config_contents


def test_sensores_real_launch_includes_datum_setter_node() -> None:
    launch_path = PACKAGE_ROOT.parent / "sensores" / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "datum_setter",' in launch_contents
    assert "default_value=\"true\"" in launch_contents
    assert 'executable="datum_setter"' in launch_contents
    assert '"set_datum_service": "/datum_setter/set_datum"' in launch_contents
    assert '"get_datum_service": "/datum_setter/get_datum"' in launch_contents
    assert '"datum_service": "/datum"' in launch_contents
    assert '"datum_service_fallback": "/navsat_transform/datum"' in launch_contents
    assert "condition=IfCondition(datum_setter)" in launch_contents


def test_real_launch_includes_ackermann_odometry_by_default() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "ackermann_odometry",' in launch_contents
    assert 'default_value="true"' in launch_contents
    assert 'executable="ackermann_odometry"' in launch_contents
    assert "PythonExpression([\"'\", ackermann_odometry, \"'.lower() == 'true'\"])" in launch_contents
    assert '"telemetry_topic": "/controller/drive_telemetry"' in launch_contents
    assert '"odom_topic": "/wheel/odometry"' in launch_contents
    assert '"publish_odom_tf": ParameterValue(' in launch_contents
    assert "PythonExpression([\"'\", ekf_local, \"'.lower() != 'true'\"])" in launch_contents


def test_real_launch_includes_zones_manager_toggle() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "zones_manager",' in launch_contents
    assert 'default_value="true"' in launch_contents
    assert 'executable="zones_manager"' in launch_contents
    assert "PythonExpression([\"'\", zones_manager, \"'.lower() == 'true'\"])" in launch_contents
    assert '"use_keepout": PythonExpression(["\'", zones_manager, "\'.lower() == \'true\'"])' in launch_contents


def test_real_launch_exposes_dual_ekf_toggles_and_no_controller_server() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "ekf_local",' in launch_contents
    assert 'default_value="true"' in launch_contents
    assert 'DeclareLaunchArgument(\n        "ekf_global",\n        default_value="true"' in launch_contents
    assert 'DeclareLaunchArgument(\n        "ukf",\n        default_value="True"' in launch_contents
    assert "condition=IfCondition(ekf_local)" in launch_contents
    assert "condition=IfCondition(ekf_global)" in launch_contents
    assert 'name="ekf_filter_node_odom"' in launch_contents
    assert 'name="ekf_filter_node_map"' in launch_contents
    assert 'name="navsat_transform"' in launch_contents
    assert "localization_filter_executable = PythonExpression(" in launch_contents
    assert "\"'ukf_node' if '\"" in launch_contents
    assert "\"'.lower() == 'true' else 'ekf_node'\"" in launch_contents
    assert "condition=IfCondition(use_navsat)" not in launch_contents
    assert 'controller_server' not in launch_contents
    assert 'executable="nav_command_server"' not in launch_contents
    assert 'web_zone_server' not in launch_contents
    assert 'no_go_editor.launch.py' not in launch_contents
    assert '"cmd_vel_final_topic": "/cmd_vel_final"' not in launch_contents
    assert '"forward_cmd_vel_safe_without_goal": True' not in launch_contents


def test_real_launch_removes_mapviz_support() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert "use_mapviz = LaunchConfiguration(" not in launch_contents
    assert 'DeclareLaunchArgument(\n        "use_mapviz",' not in launch_contents
    assert "mapviz_cmd = Node(" not in launch_contents
    assert "package=\"mapviz\"" not in launch_contents
    assert "ld.add_action(mapviz_cmd)" not in launch_contents


def test_sensores_real_launch_exposes_mavros_rtk_toggles() -> None:
    launch_path = PACKAGE_ROOT.parent / "sensores" / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "enable_rtk",\n        default_value="false"' in launch_contents
    assert (
        'DeclareLaunchArgument(\n        "enable_rtcm_tcp",\n        default_value="true"'
        in launch_contents
    )
    assert (
        'DeclareLaunchArgument(\n        "enable_rtk_source_manager",\n        default_value="false"'
        in launch_contents
    )
    assert 'DeclareLaunchArgument(\n        "rtcm_tcp_host",\n        default_value="127.0.0.1"' in launch_contents
    assert 'DeclareLaunchArgument(\n        "rtcm_tcp_port",\n        default_value="2102"' in launch_contents
    assert 'DeclareLaunchArgument(\n        "rtcm_topic",\n        default_value="/rtcm"' in launch_contents
    assert '"enable_rtk": enable_rtk' in launch_contents
    assert '"enable_rtcm_tcp": enable_rtcm_tcp' in launch_contents
    assert '"enable_rtk_source_manager": enable_rtk_source_manager' in launch_contents
    assert '"rtcm_tcp_host": rtcm_tcp_host' in launch_contents
    assert '"rtcm_tcp_port": rtcm_tcp_port' in launch_contents
    assert '"rtcm_topic": rtcm_topic' in launch_contents


def test_real_launch_auto_resolves_map_frame_from_ekf_global_toggle() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "map_frame",' in launch_contents
    assert 'default_value="auto"' in launch_contents
    assert "resolved_map_frame = PythonExpression(" in launch_contents
    assert "else ('map' if '" in launch_contents
    assert "'.lower() == 'true' else 'odom'))" in launch_contents
    assert '"map_frame": resolved_map_frame,' in launch_contents
    assert '"fromll_target_frame": resolved_map_frame,' in launch_contents
    assert '"fromll_output_frame": "map"' in launch_contents


def test_real_local_v2_launch_uses_map_output_frame_for_fromll() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real_local_v2.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert '"fromll_output_frame": "map"' in launch_contents
    assert '"map_frame": "map"' in launch_contents
    assert '"fromll_frame": "odom"' not in launch_contents


def test_real_launch_includes_tf_consistency_fail_fast_validation() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert "def _validate_tf_configuration(context, nav2_params_file: str):" in launch_contents
    assert "nav2_no_map_params.yaml is using frame 'map'" in launch_contents
    assert "Para modo map: usar `ekf_global:=True`." in launch_contents
    assert "ekf_local:=False and ackermann_odometry:=False" in launch_contents
    assert "Si desactivas `ekf_local`, mantener `ackermann_odometry:=true`." in launch_contents
    assert "OpaqueFunction(" in launch_contents
    assert "_validate_tf_configuration(context, nav2_params_file)" in launch_contents


def test_real_launch_starts_nav2_after_tf_providers_in_global_only_mode() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    nav2_index = launch_contents.index("        nav2_only_cmd,")
    ackermann_index = launch_contents.index("        ackermann_odometry_cmd,")
    ekf_map_index = launch_contents.index("        ekf_map_cmd,")
    navsat_index = launch_contents.index("        navsat_transform_cmd,")

    assert nav2_index > ackermann_index
    assert nav2_index > ekf_map_index
    assert nav2_index > navsat_index


def test_real_launch_no_longer_starts_sensor_stack() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "telemetry_backend",' not in launch_contents
    assert 'DeclareLaunchArgument(\n        "start_lidar",' not in launch_contents
    assert 'DeclareLaunchArgument(\n        "launch_web",' not in launch_contents
    assert 'executable="datum_setter"' not in launch_contents
    assert 'executable="gps_course_heading"' not in launch_contents
    assert 'package="pointcloud_to_laserscan"' not in launch_contents
    assert 'launch", "mavros.launch.py"' not in launch_contents
    assert 'launch", "pixhawk.launch.py"' not in launch_contents
    assert 'launch", "rs16.launch.py"' not in launch_contents


def test_sensores_real_launch_groups_real_sensor_stack() -> None:
    launch_path = PACKAGE_ROOT.parent / "sensores" / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "start_lidar",' in launch_contents
    assert 'DeclareLaunchArgument(\n        "launch_web",' in launch_contents
    assert 'package="pointcloud_to_laserscan"' in launch_contents
    assert 'executable="gps_course_heading"' in launch_contents
    assert 'executable="datum_setter"' in launch_contents
    assert 'launch", "mavros.launch.py"' in launch_contents
    assert 'launch", "rs16.launch.py"' in launch_contents


def test_dual_ekf_navsat_waits_for_runtime_datum() -> None:
    ekf_config_path = PACKAGE_ROOT / "config" / "dual_ekf_navsat_params.yaml"
    ekf_config_contents = ekf_config_path.read_text(encoding="utf-8")

    assert "wait_for_datum: true" in ekf_config_contents
    assert "\n    datum:" not in ekf_config_contents
