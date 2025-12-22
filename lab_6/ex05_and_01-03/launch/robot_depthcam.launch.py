import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def get_rviz_config():
    config_content = """Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /RobotModel1
        - /TF1
        - /DepthCamera1
        - /IMU1
      Splitter Ratio: 0.5
    Tree Height: 527
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /2D Pose Estimate1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: odom
      Value: true
    - Class: rviz_default_plugins/RobotModel
      Description Topic:
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      Value: true
      Visual Enabled: true
      Collision Enabled: false
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Value: true
    - Class: rviz_default_plugins/Odometry
      Topic:
        Value: /odom
      Value: true
      Keep: 100
    - Class: rviz_default_plugins/Camera
      Enabled: true
      Name: DepthCamera
      Topic:
        Value: /depth/image_raw
      Value: true
      Visibility:
        Image: true
        Image Overlay: true
    - Class: rviz_imu_plugin/Imu
      Enabled: true
      Name: IMU
      Topic:
        Value: /imu
      Box: true
      Axes: true
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: odom
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
    - Class: rviz_default_plugins/SetInitialPose
    - Class: rviz_default_plugins/SetGoal
    - Class: rviz_default_plugins/PublishPoint
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Name: Current View
      Target Frame: odom
      Value: Orbit (rviz)
"""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False) as tmp:
        tmp.write(config_content)
    return tmp.name

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_depthcam')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot_depthcam.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={'gz_args': '-r -v 4 shapes.sdf'}.items(),
    )

    create = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'differential_robot', '-topic', 'robot_description', '-x', '-5.0', '-y', '0.0', '-z', '0.5'],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
            'publish_frequency': 30.0
        }]
    )

    bridge_params = os.path.join(pkg_share, 'config', 'depthcam_bridge.yaml')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_params,
            'use_sim_time': True
        }],
        output='screen'
    )

    depth_stop = Node(
        package='robot_depthcam',
        executable='depthcam_stop',
        name='depthcam_stop',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', get_rviz_config()],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        gazebo,
        create,
        robot_state_publisher,
        bridge,
        depth_stop,
        rviz,
    ])
