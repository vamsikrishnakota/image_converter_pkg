from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            parameters=[{
                'camera_name': 'default_cam',
                'framerate': 30.0,
                'width': 640,
                'height': 480,
                'video_device': '/dev/video0'
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw')
            ]
        ),
        Node(
            package='image_converter_pkg',
            executable='image_conversion_node',
            name='image_conversion_node',
            remappings=[
                ('/camera/image_raw', '/camera/image_raw'),
                ('converted_image', '/camera/converted_image')
            ]
        )
    ])
