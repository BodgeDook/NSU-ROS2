from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'delay',
            default_value='5.0',
            description='Time delay in seconds for turtle2 to follow turtle1'
        ),
        
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        
        Node(
            package='turtle_time_follower',
            executable='spawn_turtle',
            name='spawn_turtle2',
            parameters=[{'turtle_name': 'turtle2', 'x': 5.0, 'y': 5.0}]
        ),
        
        Node(
            package='turtle_time_follower',
            executable='turtle_tf2_broadcaster',
            name='turtle1_tf2_broadcaster',
            parameters=[{'turtle_name': 'turtle1'}]
        ),
        
        Node(
            package='turtle_time_follower',
            executable='turtle_tf2_broadcaster',
            name='turtle2_tf2_broadcaster', 
            parameters=[{'turtle_name': 'turtle2'}]
        ),
        
        Node(
            package='turtle_time_follower',
            executable='turtle_time_follower',
            name='turtle_time_follower',
            parameters=[{'delay': LaunchConfiguration('delay')}]
        )
    ])
