from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Automatically find the lbr_bringup package
    bringup_launch_dir = PathJoinSubstitution(
        [FindPackageShare("lbr_bringup"), "launch"]
    )

    # Launch mock robot
    mock_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, "/hardware.launch.py"]),
        launch_arguments={"ctrl": "forward_position_controller", "model": "med7"}.items()
    )

    # Launch MoveIt Servo
    moveit_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, "/moveit_servo.launch.py"]),
        launch_arguments={"mode": "mock", "model": "med7"}.items()
    )


    return LaunchDescription([
        mock_launch,
        moveit_servo_launch,
        rviz_launch,
    ])
