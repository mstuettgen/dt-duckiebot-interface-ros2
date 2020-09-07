import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros

def generate_launch_description():

    ld = LaunchDescription()

     
    # Load the URDF into a parameter
    urdf_dir = get_package_share_directory('duckiebot_interface')
    urdf_path = os.path.join(urdf_dir, 'urdf', 'duckiebot.urdf')
    urdf = open(urdf_path).read()
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        node_name='robot_state_publisher',
        node_executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf}]
    )   

    #start wheels driver node
    wheels_driver_node=Node(
        package = 'dagu_car',
        node_name = 'wheels_driver_node',
        node_executable = 'wheels_driver_node',
        output='screen'
    )
    
    #start LED emitter node
    led_emitter_node_config = os.path.join(
        get_package_share_directory('led_emitter'),
        'config/led_emitter_node',
        'LED_protocol.yaml'
    )
    
    led_emitter_node=Node(
        package = 'led_emitter',
        node_name = 'led_emitter_node',
        node_executable = 'led_emitter_node',
        output= 'screen',
        parameters = [led_emitter_node_config]
    )

    #start camera node
    camera_node=Node(
        package = 'camera_driver',
        node_name = 'camera_node',
        node_executable = 'camera_node',
        output = 'screen'
    )
    
#    ld.add_action(robot_state_publisher_node)
    ld.add_action(wheels_driver_node)
    ld.add_action(led_emitter_node)
    ld.add_action(camera_node)
    return ld
