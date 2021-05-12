from launch import LaunchDescription

import launch.actions
import launch_ros.actions


def generate_launch_description():
    dev = launch.substitutions.LaunchConfiguration(
        'dev', default='COM1')
    return LaunchDescription([
        launch.actions.LogInfo(
            msg="ROS2 start turtlebot_bringup minimal node."
        ),

        launch.actions.DeclareLaunchArgument(
            'dev', default_value='COM1',
            description='Device name of serial port'),

        launch_ros.actions.Node(
            package="turtlebot_bringup",
            executable="turtlebot2",
            output="screen",
            parameters=[{'DeviceSpecial': dev}]
        ),

        launch_ros.actions.Node(
            package="turtlebot_bringup",
            executable="teleop_node",
            output="screen",
        ),

        launch_ros.actions.Node(
            package="joy",
            executable="joy_node",
            output="screen",
        ),
    ])
