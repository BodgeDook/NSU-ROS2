#!/usr/bin/env python3
# three_turtles.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # three separate instances of turtlesim in different namespaces
    ld.add_action(Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        namespace='turtlesim1',
        output='screen',
    ))
    ld.add_action(Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        namespace='turtlesim2',
        output='screen',
    ))
    ld.add_action(Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        namespace='turtlesim3',
        output='screen',
    ))

    # mimic: turtlesim2 follows turtlesim1
    ld.add_action(Node(
        package='turtlesim',
        executable='mimic',
        name='mimic12',
        output='screen',
        remappings=[
            ('/input/pose', '/turtlesim1/turtle1/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        ],
    ))

    # mimic: turtlesim3 follows turtlesim2
    ld.add_action(Node(
        package='turtlesim',
        executable='mimic',
        name='mimic23',
        output='screen',
        remappings=[
            ('/input/pose', '/turtlesim2/turtle1/pose'),
            ('/output/cmd_vel', '/turtlesim3/turtle1/cmd_vel'),
        ],
    ))

    return ld
