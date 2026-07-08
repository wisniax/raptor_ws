from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    pkg_path = get_package_share_directory('ros_deep_sampler')
    urdf_path = os.path.join(pkg_path, 'urdf', 'sampler_model.urdf')
    yaml_path = os.path.join(pkg_path, 'config', 'minimal.yaml')
    bridge_yaml = os.path.join(pkg_path,"config","bridge.yaml")

    # 🔥 Read URDF and replace placeholder with absolute YAML path
    with open(urdf_path, 'r') as f:
        robot_desc = f.read().replace('__CONTROLLER_CONFIG_PATH__', yaml_path)


    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_yaml}"
        ],
        output="screen",
        )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'sampler', '-topic', 'robot_description'],
        output='screen'
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    platform_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["platform_controller"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    drill_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drill_position_controller"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    rotor_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rotor_velocity_controller"],
        output="screen",
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_desc}, yaml_path],
        output="screen",
    )

    return LaunchDescription([
    SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/jazzy/lib:{existing}'
    ),

    gz_sim,
    gz_bridge,

    rsp,

    spawn,
    control_node,

    joint_state_spawner,
    platform_controller_spawner,
    drill_controller_spawner,
    rotor_controller_spawner,
])