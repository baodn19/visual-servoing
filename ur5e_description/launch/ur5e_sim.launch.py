import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = 'ur5e_description'
    xacro_file_name = 'ur5e.xacro'

    # Get the package share directory
    # Result: .../install/ur5e_description/share/ur5e_description
    pkg_share = get_package_share_directory(package_name)
    
    # Process Xacro
    xacro_path = os.path.join(pkg_share, 'urdf', xacro_file_name)
    
    # --- FIX START ---
    # We want the parent folder of 'ur5e_description' so Gazebo can resolve 'model://ur5e_description'
    # pkg_share points to: .../share/ur5e_description
    # os.path.dirname gives: .../share
    install_dir = os.path.dirname(pkg_share)
    
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_dir
    )
    # --- FIX END ---

    # Robot Description
    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_path]), 
        value_type=str
    )
    
    # Nodes
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    world_path = os.path.join(pkg_share, 'worlds', 'ur5e_world.sdf')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'ur5e_robot',
                   '-z', '0.1'], 
        output='screen'
    )

    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen'
    )

    # Controller Spawners
    node_spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    node_spawn_jtc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        node_robot_state_publisher,
        gazebo,
        node_spawn_entity,
        node_ros_gz_bridge,
        node_spawn_jsb,
        node_spawn_jtc,
    ])