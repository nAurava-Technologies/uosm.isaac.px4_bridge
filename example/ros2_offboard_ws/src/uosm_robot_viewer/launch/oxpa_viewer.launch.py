import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('uosm_robot_viewer')

    xacro_path = os.path.join(pkg_dir, 'urdf', 'oxpa.urdf.xacro')

    default_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'default_viewer.launch.py')
        ),
        launch_arguments={
            'xacro_file': xacro_path,
            'use_sim_time': 'false'
        }.items()
    )

    return LaunchDescription([default_launch])