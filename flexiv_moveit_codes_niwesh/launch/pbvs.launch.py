import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Declare configurable parameters
    robot_sn_arg = DeclareLaunchArgument('robot_sn', default_value='Rizon4s-063387')
    marker_size_arg = DeclareLaunchArgument('marker_size', default_value='0.0775')  # Marker size in meters

    # Paths to configuration parameters
    pkg_share = FindPackageShare('flexiv_moveit_codes_niwesh')
    pbvs_config = PathJoinSubstitution([pkg_share, 'config', 'pbvs_controller_config.yaml'])
    aruco_config = PathJoinSubstitution([pkg_share, 'config', 'aruco_pose_config.yaml'])



    # ==========================================
    # 1. INTEL REALSENSE CAMERA LAUNCH
    # ==========================================
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ]),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false', # right now we only need color stream for aruco detection as we know the marker size and can estimate the pose from a single camera stream
            'rgb_camera.profile': '640x480x30'
        }.items()
    )

    # ==========================================
    # 2. FLEXIV BASE HARDWARE & MOVEIT CORE
    # ==========================================
    robot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('flexiv_bringup'), 'launch', 'rizon_moveit.launch.py'])
        ]),
        launch_arguments={
            'robot_sn': LaunchConfiguration('robot_sn'),
            'use_fake_hardware': 'false',
            'start_servo': 'true'  # Start the MoveIt servo node
        }.items()
    )

    # ==========================================
    # 3. EYE-IN-HAND CALIBRATED TRANSFORMATION BRIDGE
    # ==========================================
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_link',
        arguments=[
            '--x', '-0.125', '--y', '0.0', '--z', '-0.035',
            '--roll', '0.0', '--pitch', '-1.5708', '--yaw', '0.0',
            '--frame-id', [LaunchConfiguration('robot_sn'), '_link7'],
            '--child-frame-id', 'camera_link'
        ]
    )


    # ==========================================
    # 5. PERCEPTION LOGIC (ARUCO ESTIMATOR)
    # ==========================================
    aruco_node = Node(
        package='flexiv_moveit_codes_niwesh',
        executable='aruco_pose_estimator',
        name='aruco_pose_estimator',
        parameters=[aruco_config],
        output='screen'
    )

    # ==========================================
    # 6. PBVS VELOCITY GENERATION ENGINE
    # ==========================================
    # pbvs_node = Node(
    #     package='flexiv_moveit_codes_niwesh',
    #     executable='pbvs_controller_node',
    #     name='pbvs_controller',
    #     parameters=[pbvs_config],
    #     output='screen'
    # )


    # ==========================================
    # 7. SERVICE AUTOMATION LOGIC
    # ==========================================
    # Execute the service call command directly via shell string syntax
    trigger_servo_service = ExecuteProcess(
        cmd=['ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"'],
        shell=True,
        output='screen'
    )

    # Wrap the service call with a 5-second timer delay to allow MoveIt core components to load
    automate_servo_start = TimerAction(
        period=5.0,
        actions=[trigger_servo_service]
    )

    return LaunchDescription([
        robot_sn_arg,
        marker_size_arg,
        camera_launch,
        robot_bringup_launch,
        static_tf_node,
        aruco_node,
        # set_debug_logging,
        # pbvs_node
        automate_servo_start
    ])