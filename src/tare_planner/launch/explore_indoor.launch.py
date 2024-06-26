from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz')

    use_boundary_arg = DeclareLaunchArgument(
        'use_boundary',
        default_value='false',
        description='Use boundary for navigation')

    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('tare_planner'), '/explore.launch.py']),
        launch_arguments={
            'scenario': 'indoor',
            'rviz': LaunchConfiguration('rviz')
        }.items())

    navigation_boundary_node = Node(
        package='tare_planner',
        executable='navigationBoundary',
        name='navigationBoundary',
        output='screen',
        parameters=[{
            'boundary_file_dir': 
                get_package_share_directory('tare_planner') + '/data/boundary.ply',
            'sendBoundary': True,
            'sendBoundaryInterval': 2
        }],
        condition=IfCondition(LaunchConfiguration('use_boundary')))

    return LaunchDescription([
        rviz_arg,
        use_boundary_arg,
        explore_launch,
        navigation_boundary_node,
    ])
