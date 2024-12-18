from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Definizione degli argomenti di lancio per l'IP e la porta
    ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='192.168.0.161',
        description='Robot IP address'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9559',
        description='Naoqi port number'
    )

    
    qi_unipa_sensor_node = Node(
        package='qi_unipa',
        executable='qi_unipa_sensor',  
        name='qi_unipa_sensor',
        output='screen',
        parameters=[{
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port')
        }]
    )
    
    qi_unipa_movement_node = Node(
        package='qi_unipa',
        executable='qi_unipa_movement',  
        name='qi_unipa_movement',
        output='screen',
        parameters=[{
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port')
        }]
    )

    qi_unipa_speech_node = Node(
        package='qi_unipa',
        executable='qi_unipa_speech',  
        name='qi_unipa_speech',
        output='screen',
        parameters=[{
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port')
        }]
    )
    qi_unipa_tracking_node = Node(
        package='qi_unipa',
        executable='qi_unipa_tracking',  
        name='qi_unipa_tracking',
        output='screen',
        parameters=[{
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port')
        }]
    )
    """
    qi_unipa_vision_node = Node(
        package='qi_unipa',
        executable='qi_unipa_vision',  
        name='qi_unipa_vision',
        output='screen',
        parameters=[{
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port')
        }]
    )
    """
    
    

    return LaunchDescription([
        ip_arg,
        port_arg,
        qi_unipa_sensor_node,
        qi_unipa_movement_node,
        qi_unipa_speech_node,
        qi_unipa_tracking_node,
        #qi_unipa_vision_node
    ])

