from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launching arguments:
        DeclareLaunchArgument(
            'radius',
            default_value='2.0',
            description='Radius of carrot rotation around turtle1'
        ),
        DeclareLaunchArgument(
            'direction_of_rotation',
            default_value='1',
            description='Direction of rotation: 1 for clockwise, -1 for counter-clockwise'
        ),
        
        # Starting turtlesim:
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        
        # Spawning second turtle with delay to ensure turtlesim is ready:
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', 
                         '{x: 5.0, y: 5.0, theta: 0.0, name: "turtle2"}'],
                    output='screen'
                )
            ]
        ),
        
        # Turtle1 TF broadcaster:
        Node(
            package='turtle_tf2_lab4_ex02',
            executable='turtle_tf2_broadcaster',
            name='turtle1_tf2_broadcaster',
            output='screen'
        ),
        
        # Turtle2 TF broadcaster:
        Node(
            package='turtle_tf2_lab4_ex02',
            executable='turtle2_tf2_broadcaster',
            name='turtle2_tf2_broadcaster',
            output='screen'
        ),
        
        # Carrot TF broadcaster with parameters:
        Node(
            package='turtle_tf2_lab4_ex02',
            executable='carrot_tf2_broadcaster',
            name='carrot_tf2_broadcaster',
            output='screen',
            parameters=[{
                'radius': LaunchConfiguration('radius'),
                'direction_of_rotation': LaunchConfiguration('direction_of_rotation')
            }]
        ),
        
        # Turtle2 TF listener (follows carrot):
        Node(
            package='turtle_tf2_lab4_ex02',
            executable='turtle_tf2_listener',
            name='turtle_tf2_listener',
            output='screen'
        ),
        
        # Teleop key for turtle1:
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop_key',
            output='screen',
            prefix='xterm -e'
        )
    ])
