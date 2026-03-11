import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
	# Get arguments
    sim_arg = LaunchConfiguration('simulation').perform(context).lower() == 'true'
    
    # Include Traffic Sign detection launch
    traffic_signs_launch = os.path.join(get_package_share_directory('road_perception'), 'launch', 'traffic_sign_detection.launch.py')
    traffic_signs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(traffic_signs_launch),
        launch_arguments={
            'simulation': 'True' if sim_arg else 'False'     
        }.items(),
    )

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
            {"cmd_vel_topic": "/road_following/cmd_vel"},   
        ],
        output='screen',
        emulate_tty=True,
    )

    # Autonomous Driving node
    autonomous_driving_node = Node(
        package="autonomous_driving",
        executable="autonomous_driving",
        parameters=[
            {'traffic_signs_topic': '/road_perception/traffic_signs'},
            {'road_following_vel_topic': '/road_following/cmd_vel'},
            {'cmd_vel_topic': '/cmd_vel'},
            {'road_perception_ground_topic': '/road_perception/ground'},
            {'odom': '/odom'},
            {"scan_topic": "scan"},
            {"lidar_type": "s2"},
            {'simulation': sim_arg},
        ],
        output='screen',
        emulate_tty=True,
    )
    return [
        traffic_signs_launch,
        ground_segmentation_launch,
        road_following_node,
        autonomous_driving_node
    ]

def generate_launch_description():
    declare_sim_arg = DeclareLaunchArgument(
        'simulation',
        default_value='True',
        description='Set to True for simulation, False for real robot'
    )
    
    return LaunchDescription([
        declare_sim_arg,
		OpaqueFunction(function=launch_setup)
	])
