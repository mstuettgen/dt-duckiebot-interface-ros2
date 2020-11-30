import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros

def generate_launch_description():

    ld = LaunchDescription()
    
    duckiebot_urdf = os.path.join(get_package_share_directory('duckiebot_interface'),
                        'urdf', 'duckiebot.urdf')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        node_name='robot_state_publisher',
        node_executable='robot_state_publisher',
        output='screen',
        arguments=[duckiebot_urdf]
    )
    
    ld.add_action(robot_state_publisher_node)
    return ld
