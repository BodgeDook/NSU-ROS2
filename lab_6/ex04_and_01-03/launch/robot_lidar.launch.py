import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
        - /LaserScan1
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
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      TF Prefix: ""
      Value: true
      Visual Enabled: true
      Collision Enabled: false
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 0.5
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree: {}
      Update Interval: 0
      Value: true
    - Class: rviz_default_plugins/Odometry
      Alpha: 1
      Axes Length: 1
      Axes Radius: 0.1
      Color: 255; 255; 0
      Covariance Color: 255; 255; 0
      Covariance Position: true
      Covariance Rotation: true
      Head Length: 0.3
      Head Radius: 0.2
      Name: Odometry
      Position Tolerance: 0.1
      Shaft Length: 0.1
      Shaft Radius: 0.05
      Shape: Arrow
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /odom
      Value: true
      Yaw Tolerance: 0.1
    - Alpha: 1
      Class: rviz_default_plugins/LaserScan
      Color: 255; 0; 0
      Decay Time: 0
      Enabled: true
      Name: LaserScan
      Position Transformer: XYZ
      Size (m): 0.1
      Style: Spheres
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Use Fixed Frame: true
      Value: true
    - Class: rviz_imu_plugin/Imu
      Enabled: true
      Name: IMU
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /imu
      Box: true
      Axes: true
      Alpha: 1
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
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /move_base_simple/goal
    - Class: rviz_default_plugins/PublishPoint
      Single Click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.785398
      Target Frame: odom
      Value: Orbit (rviz)
      Yaw: 0.785398
    Saved: ~
Window Geometry:
  Displays: {collapsed: false}
  Height: 927
  Hide Left Dock: false
  Hide Right Dock: false
  Selection: {collapsed: false}
  Tool Properties: {collapsed: false}
  Views: {collapsed: false}
  Width: 1200
  X: 100
  Y: 100
"""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False) as tmp:
        tmp.write(config_content)
    return tmp.name

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_lidar')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot_lidar.urdf.xacro')
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

    bridge_params = os.path.join(pkg_share, 'config', 'lidar_bridge.yaml')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_params,
            'use_sim_time': True
        }],
        output='screen'
    )

    lidar_stop = Node(
        package='robot_lidar',
        executable='lidar_stop',
        name='lidar_stop',
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
        lidar_stop,
        rviz,
    ])
