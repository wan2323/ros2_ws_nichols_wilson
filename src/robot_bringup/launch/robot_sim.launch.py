from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')
    urdf_path = os.path.join(bringup_share, 'urdf', 'robot.urdf')
    rviz_cfg  = os.path.join(bringup_share, 'rviz', 'simulation.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    odometry_node = Node(
        package='robot_simulator_cpp',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
    )

    controller_node = Node(
        package='robot_simulator_py',
        executable='controller_node',
        name='controller_node',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
    )

    return LaunchDescription([
        robot_state_pub,
        odometry_node,
        controller_node,
        rviz
    ])
