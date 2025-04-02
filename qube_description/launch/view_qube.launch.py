import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

package_name = 'qube_description'

def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory(package_name),"urdf","qube.urdf.xacro")
    robot_description_content = xacro.process_file(xacro_file).toxml()   

    node_robot_state_publisher = Node( #Robot state publisher for å sende robot beskrivelsen til Rviz
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
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

    #join pub gui for å ha en gui til å kontrollere rotor vinkelen (nødvendig når den ikke er kontrollert av noe annet)
    joint_pub_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    #selve launchen
    return LaunchDescription([
        node_robot_state_publisher,
        rviz_node,
        joint_pub_gui
    ])