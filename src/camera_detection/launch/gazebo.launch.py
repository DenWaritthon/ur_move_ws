import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to URDF files
    camera_urdf_path = os.path.join(
        get_package_share_directory('camera_detection'), 'urdf', 'camera_box.urdf'
    )
    extra_box_urdf_path = os.path.join(
        get_package_share_directory('camera_detection'), 'urdf', 'extra_box.urdf'
    )

    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # Spawn camera box
    spawn_camera_box = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'camera_box',
            '-file', camera_urdf_path,
            '-x', '0', '-y', '0', '-z', '0.025'
        ],
        output='screen'
    )

    # Spawn extra box separately
    spawn_extra_box = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'extra_box',
            '-file', extra_box_urdf_path,
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        gazebo,
        spawn_camera_box,
        spawn_extra_box
    ])