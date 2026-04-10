import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_config_file_path(package_share_dir: str, package_name: str, filename: str) -> str:
    package_share_path = Path(package_share_dir)
    default_path = package_share_path / "config" / filename
    try:
        workspace_root = package_share_path.parents[3]
        source_path = workspace_root / "src" / package_name / "config" / filename
        if source_path.parent.exists():
            return str(source_path)
    except IndexError:
        pass
    return str(default_path)

def generate_launch_description():
    sensores_dir = get_package_share_directory("sensores")
    navegacion_gps_dir = get_package_share_directory("navegacion_gps")

    lidar_to_scan_params = _resolve_config_file_path(
        navegacion_gps_dir, "navegacion_gps", "pointcloud_to_laserscan.yaml"
    )
    lidar_default_config = _resolve_config_file_path(
        sensores_dir, "sensores", "rs16.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_lidar = LaunchConfiguration("start_lidar")
    launch_web = LaunchConfiguration("launch_web")
    use_pointcloud_to_laserscan = LaunchConfiguration("use_pointcloud_to_laserscan")
    enable_rtk = LaunchConfiguration("enable_rtk")
    enable_rtcm_tcp = LaunchConfiguration("enable_rtcm_tcp")
    enable_rtk_source_manager = LaunchConfiguration("enable_rtk_source_manager")
    rtcm_tcp_host = LaunchConfiguration("rtcm_tcp_host")
    rtcm_tcp_port = LaunchConfiguration("rtcm_tcp_port")
    rtcm_topic = LaunchConfiguration("rtcm_topic")
    lidar_config_path = LaunchConfiguration("lidar_config_path")
    gps_topic = LaunchConfiguration("gps_topic")
    rtk_status_topic = LaunchConfiguration("rtk_status_topic")
    datum_setter = LaunchConfiguration("datum_setter")
    enable_gps_course_heading = LaunchConfiguration("enable_gps_course_heading")
    gps_course_heading_enable_consistency_filters = LaunchConfiguration(
        "gps_course_heading_enable_consistency_filters"
    )
    gps_course_heading_enable_offset_compensation = LaunchConfiguration(
        "gps_course_heading_enable_offset_compensation"
    )
    gps_course_heading_gps_frame = LaunchConfiguration("gps_course_heading_gps_frame")
    gps_course_heading_transform_timeout_s = LaunchConfiguration(
        "gps_course_heading_transform_timeout_s"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock if true",
    )
    declare_start_lidar_cmd = DeclareLaunchArgument(
        "start_lidar",
        default_value="True",
        description="Start RS16 LiDAR driver",
    )
    declare_launch_web_cmd = DeclareLaunchArgument(
        "launch_web",
        default_value="False",
        description="Start sensores_web node",
    )
    declare_use_pointcloud_to_laserscan_cmd = DeclareLaunchArgument(
        "use_pointcloud_to_laserscan",
        default_value="True",
        description="Whether to start pointcloud_to_laserscan",
    )
    declare_enable_rtk_cmd = DeclareLaunchArgument(
        "enable_rtk",
        default_value="false",
        description="Enable MAVROS RTK bridge that feeds RTCM to the FCU",
    )
    declare_enable_rtcm_tcp_cmd = DeclareLaunchArgument(
        "enable_rtcm_tcp",
        default_value="true",
        description="Read RTCM corrections from a TCP source in MAVROS",
    )
    declare_enable_rtk_source_manager_cmd = DeclareLaunchArgument(
        "enable_rtk_source_manager",
        default_value="false",
        description="Enable RTK source manager when using MAVROS backend",
    )
    declare_rtcm_tcp_host_cmd = DeclareLaunchArgument(
        "rtcm_tcp_host",
        default_value="127.0.0.1",
        description="Host for the incoming RTCM TCP stream",
    )
    declare_rtcm_tcp_port_cmd = DeclareLaunchArgument(
        "rtcm_tcp_port",
        default_value="2102",
        description="Port for the incoming RTCM TCP stream",
    )
    declare_rtcm_topic_cmd = DeclareLaunchArgument(
        "rtcm_topic",
        default_value="/rtcm",
        description="Optional ROS topic carrying RTCM corrections",
    )
    declare_lidar_config_path_cmd = DeclareLaunchArgument(
        "lidar_config_path",
        default_value=lidar_default_config,
        description="Path to rs16 YAML config",
    )
    declare_gps_topic_cmd = DeclareLaunchArgument(
        "gps_topic",
        default_value="/gps/fix",
        description="GPS topic used by datum_setter and gps_course_heading",
    )
    declare_rtk_status_topic_cmd = DeclareLaunchArgument(
        "rtk_status_topic",
        default_value="/gps/rtk_status",
        description="RTK status topic used by datum_setter",
    )
    declare_datum_setter_cmd = DeclareLaunchArgument(
        "datum_setter",
        default_value="true",
        description="Enable datum_setter node",
    )
    declare_enable_gps_course_heading_cmd = DeclareLaunchArgument(
        "enable_gps_course_heading",
        default_value="true",
        description="Enable gps_course_heading",
    )
    declare_gps_course_heading_enable_consistency_filters_cmd = DeclareLaunchArgument(
        "gps_course_heading_enable_consistency_filters",
        default_value="true",
        description="Enable advanced gps_course_heading consistency filters",
    )
    declare_gps_course_heading_enable_offset_compensation_cmd = DeclareLaunchArgument(
        "gps_course_heading_enable_offset_compensation",
        default_value="true",
        description="Enable gps_course_heading antenna offset compensation via TF",
    )
    declare_gps_course_heading_gps_frame_cmd = DeclareLaunchArgument(
        "gps_course_heading_gps_frame",
        default_value="gps_link",
        description="GPS frame resolved against base_footprint to compensate antenna offset",
    )
    declare_gps_course_heading_transform_timeout_s_cmd = DeclareLaunchArgument(
        "gps_course_heading_transform_timeout_s",
        default_value="0.2",
        description="TF lookup timeout for gps_course_heading antenna offset compensation",
    )

    mavros_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensores_dir, "launch", "mavros.launch.py")
        ),
        launch_arguments={
            "launch_web": launch_web,
            "launch_legacy_compat": "true",
            "enable_rtk": enable_rtk,
            "enable_rtcm_tcp": enable_rtcm_tcp,
            "enable_rtk_source_manager": enable_rtk_source_manager,
            "rtcm_tcp_host": rtcm_tcp_host,
            "rtcm_tcp_port": rtcm_tcp_port,
            "rtcm_topic": rtcm_topic,
        }.items(),
    )

    camera_cmd = Node(
        package="sensores",
        executable="camara",
        name="camara",
        output="screen",
    )

    lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensores_dir, "launch", "rs16.launch.py")
        ),
        launch_arguments={"config_path": lidar_config_path}.items(),
        condition=IfCondition(start_lidar),
    )

    lidar_to_scan_cmd = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            lidar_to_scan_params,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"output_qos": "sensor_data"},
        ],
        remappings=[("cloud_in", "/scan_3d"), ("scan", "/scan")],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    start_lidar,
                    "'.lower() == 'true' and '",
                    use_pointcloud_to_laserscan,
                    "'.lower() == 'true'",
                ]
            )
        ),
    )

    gps_course_heading_cmd = Node(
        package="navegacion_gps",
        executable="gps_course_heading",
        name="gps_course_heading",
        output="screen",
        condition=IfCondition(enable_gps_course_heading),
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"gps_topic": gps_topic},
            {"odom_topic": "/odometry/local"},
            {"drive_telemetry_topic": "/controller/drive_telemetry"},
            {"output_topic": "/gps/course_heading"},
            {"debug_topic": "/gps/course_heading/debug"},
            {"base_frame": "base_footprint"},
            {
                "enable_consistency_filters": ParameterValue(
                    gps_course_heading_enable_consistency_filters,
                    value_type=bool,
                )
            },
            {
                "enable_offset_compensation": ParameterValue(
                    gps_course_heading_enable_offset_compensation,
                    value_type=bool,
                )
            },
            {"gps_frame": ParameterValue(gps_course_heading_gps_frame, value_type=str)},
            {
                "transform_timeout_s": ParameterValue(
                    gps_course_heading_transform_timeout_s,
                    value_type=float,
                )
            },
        ],
    )

    datum_setter_cmd = Node(
        package="navegacion_gps",
        executable="datum_setter",
        name="datum_setter",
        output="screen",
        parameters=[
            {
                "gps_topic": gps_topic,
                "imu_topic": "/imu/data",
                "rtk_status_topic": rtk_status_topic,
                "set_datum_service": "/datum_setter/set_datum",
                "get_datum_service": "/datum_setter/get_datum",
                "datum_service": "/datum",
                "datum_service_fallback": "/navsat_transform/datum",
                "imu_yaw_max_age_s": 1.0,
                "datum_wait_timeout_s": 2.0,
                "datum_call_timeout_s": 2.5,
                "datum_call_retries": 3,
                "datum_retry_delay_s": 0.15,
            }
        ],
        condition=IfCondition(datum_setter),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_start_lidar_cmd)
    ld.add_action(declare_launch_web_cmd)
    ld.add_action(declare_use_pointcloud_to_laserscan_cmd)
    ld.add_action(declare_enable_rtk_cmd)
    ld.add_action(declare_enable_rtcm_tcp_cmd)
    ld.add_action(declare_enable_rtk_source_manager_cmd)
    ld.add_action(declare_rtcm_tcp_host_cmd)
    ld.add_action(declare_rtcm_tcp_port_cmd)
    ld.add_action(declare_rtcm_topic_cmd)
    ld.add_action(declare_lidar_config_path_cmd)
    ld.add_action(declare_gps_topic_cmd)
    ld.add_action(declare_rtk_status_topic_cmd)
    ld.add_action(declare_datum_setter_cmd)
    ld.add_action(declare_enable_gps_course_heading_cmd)
    ld.add_action(declare_gps_course_heading_enable_consistency_filters_cmd)
    ld.add_action(declare_gps_course_heading_enable_offset_compensation_cmd)
    ld.add_action(declare_gps_course_heading_gps_frame_cmd)
    ld.add_action(declare_gps_course_heading_transform_timeout_s_cmd)
    ld.add_action(mavros_cmd)
    ld.add_action(camera_cmd)
    ld.add_action(lidar_cmd)
    ld.add_action(lidar_to_scan_cmd)
    ld.add_action(gps_course_heading_cmd)
    ld.add_action(datum_setter_cmd)
    return ld
