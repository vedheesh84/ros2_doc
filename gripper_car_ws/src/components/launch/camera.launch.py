from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [1280, 720],
                'camera_frame_id': 'camera_link_optical',
                'pixel_format': 'YUYV',  # or 'MJPEG' for higher FPS
                'output_encoding': 'rgb8',
                'framerate': 30.0,
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw'),
            ]
        )
    ])