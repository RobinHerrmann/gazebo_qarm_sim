import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = 'qarm_v1'
    world_file_name = 'empty.world'

    # world
    world = os.path.join(get_package_share_directory(
        robot_name), 'worlds', world_file_name)

    # urdf
    xacro_file = os.path.join(get_package_share_directory(
        robot_name), 'urdf', 'qarm1.urdf.xacro')    
    xml = xacro.process_file(xacro_file).toxml()#.replace('"', '\\"')

    # rviz
    rviz = os.path.join(get_package_share_directory(
        robot_name), 'rviz', 'urdf.rviz')

    # Node initialization
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': xml}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz],
    )

    return LaunchDescription([
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        robot_state_publisher_node,
    ])