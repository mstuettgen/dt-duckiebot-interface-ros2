import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('led_emitter'),
        'config/led_emitter_node',
        'LED_protocol.yaml'
        )
        
    node=Node(
        package = 'led_emitter',
        node_name = 'led_emitter_node',
        node_executable = 'led_emitter_node',
        output= 'screen',
        parameters = [config]
    )
    
    ld.add_action(node)
    return ld
