from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'device_index',
            default_value='0',
            description='Index of the iOS device to connect to (0 for first device, 1 for second, etc.)'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='',
            description='TF frame ID for published messages (default: device_{i}_camera_link)'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='record3d',
            description='ROS namespace for the node and topics'
        ),
        DeclareLaunchArgument(
            'world_frame',
            default_value='world',
            description='World/reference frame for TF tree'
        ),
        DeclareLaunchArgument(
            'apply_transform',
            default_value='true',
            description='Apply Y-up to Z-up coordinate transformation (iPhone to ROS)'
        ),

        # Launch the record3d node
        Node(
            package='record3d',
            executable='record3d_node',
            name='record3d_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'device_index': LaunchConfiguration('device_index'),
                'frame_id': LaunchConfiguration('frame_id'),
                'world_frame': LaunchConfiguration('world_frame'),
                'apply_transform': LaunchConfiguration('apply_transform'),
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])
