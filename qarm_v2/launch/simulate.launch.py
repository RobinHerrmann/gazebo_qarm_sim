import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
import xacro


def generate_launch_description():
    robot_name = 'qarm_v2'
    world_file_name = 'empty.world'

    # Parsing URDF.XACRO
    xacro_file = os.path.join(get_package_share_directory(
        robot_name), 'urdf', 'robot.urdf.xacro')    
    xml_qarm = xacro.process_file(xacro_file).toxml()#.replace('"', '\\"')

    # Configuring RVIZ
    rviz = os.path.join(get_package_share_directory(
        robot_name), 'rviz', 'urdf.rviz')

    # Configuring RVIZ Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz, '--title', 'QArm RVIZ'],
    )

    # Robot State Publisher Node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': xml_qarm,
        'use_sim_time': True}]
    )

    # Configuring Gazebo
    gazebo= ExecuteProcess(
        cmd=['gazebo', '-u', os.path.join(get_package_share_directory(robot_name), 'worlds', 'qarm_world_1.world'), "--verbose",
             "-s", "libgazebo_ros_factory.so",
             "-s", "libgazebo_ros_init.so", "--ros-args", "--params-file", os.path.join(get_package_share_directory(robot_name), 'config', 'params_init.yaml'), 
             ],
        output='screen'
    )

    # Spawning Robot Arm
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'qarm_v2'],
                    output='screen')

    # Joint State Broadcaster
    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Joint Trajectory Controller
    joint_arm_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_arm_effort_controller'],
        output='screen'
    )

    # Joint Effort Controller
    joint_gripper_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_gripper_effort_controller'],
        output='screen'
    )

    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[joint_arm_position_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[joint_gripper_position_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[rviz_node],
            )
        ),
    ])