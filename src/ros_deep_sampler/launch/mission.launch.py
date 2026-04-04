from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    mission_node = Node(
        package='ros_deep_sampler',      
        executable='ros_deep_sampler_node',      
        #name='mission_node',
        output='screen',
        parameters=[
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([
        mission_node
    ])