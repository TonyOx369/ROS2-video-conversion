from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe', 
            name='usb_cam_node',
            output='screen',
            parameters=[
                {'video_device': '/dev/video1'},  
                {'framerate': 30.0},
                {'image_width': 640},
                {'image_height': 480},
                {'camera_name': 'usb_cam'},
                {'pixel_format': 'yuyv'}
            ]
        ),
    ])
