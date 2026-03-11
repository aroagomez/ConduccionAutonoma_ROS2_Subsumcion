import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):

    # Include Ground Segmentation launch
    ground_segmentation_launch = os.path.join(get_package_share_directory('road_perception'), 'launch', 'ground_segmentation_deeplabv3.launch.py')
    ground_segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ground_segmentation_launch)
    )

    # Road Following node
    road_following_node = Node(
        package="road_following",
        executable="road_following",
        parameters=[
            {"ground_image_topic": "/road_perception/ground"},
            {"cmd_vel_topic": "/cmd_vel"}   
        ],
        output='screen',
        emulate_tty=True,
    )
    return [ground_segmentation_launch, road_following_node]

def generate_launch_description():

    return LaunchDescription([
		OpaqueFunction(function=launch_setup)
	])
