from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
#THIS IS THE LAUNCH FILE FOR THE CONTROL PACKAGE
def generate_launch_description():
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(get_package_share_directory("arduinobot_description"),"urdf", "arduinobot.urdf.xacro")
            ]
        ),
        value_type=str
    )
    #THIS PUBLISHES THE STATUS FOR THE ROBOT DESCRIPTION OF THE URDF FILE
    robot_state_publisher =Node(
        package ="robot_state_publisher",
        executable ="robot_state_publisher",
        parameters= [{"robot_description": robot_description}]
    )
    #THIS PUBLISHES THE POSITION OF THE JOINTS OF THE ROBOT WHEN MOVING
    joint_state_broadcaster_spawner =Node(
        package ="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    #THIS SPAWNS THE CONTROLLER IN THE ENVIRONMENT
    arm_controller_spawner =Node(
        package ="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    #THIS SPAWNS THE GRIPPER CONTROLLER IN THE ENVIRONMENT
    gripper_controller_spawner =Node(
        package ="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    #THIS LAUNCHES ALL THE PREVIOUSLY DEFINED MODULES
    return LaunchDescription([
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner
    ])