from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyACM0',
        description='Serial port name'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Serial baud rate'
    )
    
    # Signal generator node
    signal_generator_node = Node(
        package='turbojet_control',
        executable='signal_generator',
        name='signal_generator',
        output='screen'
    )
    
    # Serial communication node
    serial_comm_node = Node(
        package='turbojet_control',
        executable='serial_comm',
        name='serial_comm',
        output='screen',
        parameters=[{
            'port_name': LaunchConfiguration('port_name'),
            'baud_rate': LaunchConfiguration('baud_rate')
        }]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the launch arguments
    ld.add_action(port_name_arg)
    ld.add_action(baud_rate_arg)
    
    # Add the nodes
    ld.add_action(signal_generator_node)
    ld.add_action(serial_comm_node)
    
    return ld