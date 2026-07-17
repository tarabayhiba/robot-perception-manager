from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('perception_pipeline'), 'config', 'perception_pipeline.yaml'
    ])
    return LaunchDescription([
        Node(
            package='perception_pipeline',
            executable='perception_manager',
            name='perception_manager',
            namespace='perception',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='perception_pipeline',
            executable='camera_tf_broadcaster',
            name='camera_tf_broadcaster',
            namespace='perception',
            parameters=[config],
            output='screen',
        ),
    ])
