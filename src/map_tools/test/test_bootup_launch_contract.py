from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_bootup_launch_includes_controller_server_launch() -> None:
    launch_contents = (PACKAGE_ROOT / "launch" / "bootup.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'get_package_share_directory("controller_server")' in launch_contents
    assert 'IncludeLaunchDescription(' in launch_contents
    assert '"controller_server.launch.py"' in launch_contents
    assert '"controller_server_sim.launch.py"' in launch_contents
    assert "UnlessCondition(use_sim)" in launch_contents
    assert "IfCondition(use_sim)" in launch_contents


def test_bootup_launch_starts_nav_command_server_web_zone_server_and_process_executor() -> None:
    launch_contents = (PACKAGE_ROOT / "launch" / "bootup.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'package="navegacion_gps"' in launch_contents
    assert 'executable="nav_command_server"' in launch_contents
    assert 'name="nav_command_server"' in launch_contents
    assert '"forward_cmd_vel_safe_without_goal": True' in launch_contents
    assert 'package="map_tools"' in launch_contents
    assert 'executable="web_zone_server"' in launch_contents
    assert 'name="web_zone_server"' in launch_contents
    assert 'package="utilities"' in launch_contents
    assert 'executable="process_executor"' in launch_contents
    assert 'name="process_executor"' in launch_contents


def test_bootup_launch_exposes_expected_arguments() -> None:
    launch_contents = (PACKAGE_ROOT / "launch" / "bootup.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'DeclareLaunchArgument("use_sim", default_value="false")' in launch_contents
    assert 'DeclareLaunchArgument("ws_host", default_value="0.0.0.0")' in launch_contents
    assert 'DeclareLaunchArgument("ws_port", default_value="8766")' in launch_contents
    assert 'DeclareLaunchArgument("gps_topic", default_value="/gps/fix")' in launch_contents
    assert 'DeclareLaunchArgument("map_frame", default_value="map")' in launch_contents
    assert 'DeclareLaunchArgument("fromll_output_frame", default_value="map")' in launch_contents
    assert 'DeclareLaunchArgument("cmd_vel_safe_topic", default_value="/cmd_vel_safe")' in launch_contents
    assert 'DeclareLaunchArgument("cmd_vel_final_topic", default_value="/cmd_vel_final")' in launch_contents
    assert 'DeclareLaunchArgument("teleop_cmd_topic", default_value="/cmd_vel_teleop")' in launch_contents
    assert '"approx_fromll_fallback_enabled", default_value="false"' in launch_contents
    assert 'DeclareLaunchArgument("approx_fromll_datum_lat", default_value="nan")' in launch_contents
    assert 'DeclareLaunchArgument("approx_fromll_datum_lon", default_value="nan")' in launch_contents
    assert 'DeclareLaunchArgument("approx_fromll_datum_yaw_deg", default_value="0.0")' in launch_contents
    assert 'get_package_share_directory("utilities")' in launch_contents
    assert 'DeclareLaunchArgument("processes_file", default_value=str(default_processes_file))' in launch_contents
    assert 'DeclareLaunchArgument("process_file_logging", default_value="true")' in launch_contents
