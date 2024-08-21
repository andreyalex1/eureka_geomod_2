from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eureka_geomod_2',
            executable='cwt_npkphcth_reader',
            name='cwt_npkphcth_reader',
            shell=True,
        ),
    ])