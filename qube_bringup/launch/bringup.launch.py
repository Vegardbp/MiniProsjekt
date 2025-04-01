import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


package_name = 'qube_bringup'

def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory(package_name),"urdf","controlled_qube.urdf.xacro")
    robot_description_content = xacro.process_file(xacro_file).toxml()   

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}] # adds a URDF to the robot description
    )

    qube_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('qube_driver'),
                'launch',
                'qube_driver.launch.py'
            )
        ])
    )

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'rviz_config.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        Node(
            package='qube_controller',
            executable='qube_controller_node',
            name='qube_controller',
            output='screen',
            parameters=[{
                'kp': 1.0,    # Start with small values
                'ki': 0.0,
                'kd': 0.0,
                'setpoint': 1.0,  # Adjust this to your desired position
                'joint_name': 'motor_joint'
            }]
        ),
        node_robot_state_publisher,
        rviz_node,
        qube_driver_launch
    ])