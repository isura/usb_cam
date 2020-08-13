import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    camera_video_device = LaunchConfiguration('video_device'   , default='/dev/video0')
    camera_image_width  = LaunchConfiguration('image_width'    , default='640'        )
    camera_image_height = LaunchConfiguration('image_height'   , default='480'        )
    camera_pixel_format = LaunchConfiguration('pixel_format'   , default='yuyv'       )
    camera_frame_id     = LaunchConfiguration('camera_frame_id', default='usb_cam'    )
    camera_io_method    = LaunchConfiguration('io_method'      , default='mmap'       )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node',
        namespace=camera_frame_id,
        output='screen',
        parameters=[{ 'video_device'    : camera_video_device },
                    { 'image_width'     : camera_image_width  },
                    { 'image_height'    : camera_image_height },
                    { 'pixel_format'    : camera_pixel_format },
                    { 'camera_frame_id' : camera_frame_id     },
                    { 'io_method'       : camera_io_method    }]
    )

    showimage_node = Node(
        package='image_tools',
        executable='showimage',
        output='screen',
        parameters=[{ 'autosize' : True                 }],
        remappings=[( 'image'    , '/usb_cam/image_raw' )]
    )

    return LaunchDescription([ usb_cam_node, showimage_node ])