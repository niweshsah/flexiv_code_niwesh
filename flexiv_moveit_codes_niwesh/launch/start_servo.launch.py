import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Dynamically generate the Robot Description (URDF)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("flexiv_description"), "urdf", "rizon.urdf.xacro"]), " ",
        "robot_sn:=Rizon4s-063387", " ",
        'rizon_type:=Rizon4s', ' ',
        "use_fake_hardware:=false"
    ])
    robot_description = {"robot_description": robot_description_content}

    # 2. Dynamically generate the Semantic Description (SRDF)
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("flexiv_moveit_config"), "srdf", "rizon.srdf.xacro"]), " ",
        "robot_sn:=Rizon4s-063387", " ",
        'rizon_type:=Rizon4s'
    ])
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # 3. Load the Servo Parameters
    servo_params = PathJoinSubstitution([
        FindPackageShare("flexiv_moveit_codes_niwesh"), 
        "config", 
        "flexiv_servo.yaml"
    ])

    # 4. Start the MoveIt Servo Node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="moveit_servo",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
    )

    return LaunchDescription([servo_node])