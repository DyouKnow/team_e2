from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    TURTLEBOT3_MODEL = LaunchConfiguration('model')

    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value='waffle_pi',
        description='Turtlebot3 model type [burger, waffle, waffle_pi]'
    )

    #시뮬레이션 월드 경로 설정
    world_path = os.path.join(
        get_package_share_directory('simulation'),
        'worlds',
        'track_world.world'
    )

    #gazebo 환경 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': world_path}.items()
    )

    #Waffle Pi 소환
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'robot_state_publisher.launch.py'
            )
        ]),
        launch_arguments={'model': TURTLEBOT3_MODEL}.items()
    )

    return LaunchDescription([
        declare_model_arg,
        gazebo,
        robot_state_publisher
    ])