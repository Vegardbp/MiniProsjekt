import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

package_name = 'qube_bringup'

def generate_launch_description():
    # Deklarer launch-argumenter
    declared_arguments = [
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baudhastighet for seriell kommunikasjon'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyACM0',
            description='Enhetsfil for seriell kommunikasjon'
        ),
        DeclareLaunchArgument(
            'simulation',
            default_value='true',
            description='Skal simulering brukes? (true/false)'
        ),
    ]

    # Hent verdiene fra launch-argumentene
    baud_rate = LaunchConfiguration('baud_rate')
    device = LaunchConfiguration('device')
    simulation = LaunchConfiguration('simulation')

    # Definer xacro-filen
    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'controlled_qube.urdf.xacro'
    )

    # Prosesser xacro-filen dynamisk med Command
    robot_description_param = {
        'robot_description': Command([
            'xacro ', xacro_file,
            ' baud_rate:=', baud_rate,
            ' device:=', device,
            ' simulation:=', simulation
        ])
    }

    # Robot state publisher noden for å sende robotten til rviz
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param]
    )

    # Launch qube driveren fra Lars Ivar sitt bibliotek
    qube_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('qube_driver'),
                'launch',
                'qube_driver.launch.py'
            )
        ])
    )

    #Rviz
    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'rviz_config.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    #Pid kontroller for quben
    qube_controller = Node(
        package='qube_controller',
        executable='qube_controller_node',
        name='qube_controller',
        output='screen',
        parameters=[{
            'kp': 1.0,
            'ki': 0.0,
            'kd': 0.0,
            'setpoint': 1.0,
            'joint_name': 'motor_joint'
        }]
    )

    # Returner launch-beskrivelsen
    return LaunchDescription([
        *declared_arguments,
        qube_controller,
        node_robot_state_publisher,
        rviz_node,
        qube_driver_launch
    ])