import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

#THIS LAUNCHER WILL OPEN ALL THE DEVELOPED MODULES FOR THIS IMPLEMENTATION
def generate_launch_description():
    gazebo = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("arduinobot_description"),
                "launch",
                "gazebo.launch.py"
            )
        )
 #THIS LAUNCHES THE CONTROLLER MODULE   
    controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("arduinobot_controller"),
                "launch",
                "controller.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items()
        )
#THIS LAUNCHES THE TRAJECTORY MODEL 
    moveit = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("arduinobot_moveit"),
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items()
        )
#THIS LAUNCHES THE REMOTE CONTROL MODULE 
    remote_interface = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("arduinobot_remote"),
                "launch",
                "remote_interface.launch.py"
            ),
        )
#THIS OPENS THE LIST OF MODULES DESCRIBED  
    return LaunchDescription([
        gazebo,
        controller,
        moveit,
        remote_interface,
    ])