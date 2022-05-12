from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    To run this launch file, call:
      ros2 launch anyfeeder_connector anyfeeder_connector.launch.py
    To see the list of arguments and how to pass them:
      ros2 launch anyfeeder_connector anyfeeder_connector.launch.py --show-args

    :return:
    """
    device = LaunchConfiguration('device')
    device_argument = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyUSB0',
        description="Which device to use"
    )
    settling_time = LaunchConfiguration('settling_time')
    settling_time_argument = DeclareLaunchArgument(
        'settling_time',
        default_value='0.001',
        description="The node will wait: settling_delay after each action."
    )
    sim_node = Node(
        package='anyfeeder_connector',
        node_executable='anyfeeder_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'device': device,
            'settling_time': settling_time
        }]
    )

    return LaunchDescription([
        device_argument,
        settling_time_argument,
        sim_node
    ])
