import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Find the share directory of this package to resolve the YAML calibration file path
    package_dir = get_package_share_directory('flir_boson_usb2')
    default_calibration_path = os.path.join(package_dir, 'example_calibrations', 'Boson640.yaml')

    # Declare launch arguments directly matching the ROS 1 <arg> tags
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='flir_boson',
        description='The ROS namespace for the camera nodes'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='boson_camera',
        description='The TF frame ID for the camera'
    )
    dev_arg = DeclareLaunchArgument(
        'dev', default_value='/dev/video0',
        description='The linux file descriptor location for the camera'
    )
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate', default_value='30.0',
        description='Valid values are 30.0 or 60.0 for Bosons'
    )
    video_mode_arg = DeclareLaunchArgument(
        'video_mode', default_value='YUV',
        description='Valid values are RAW16 or YUV'
    )
    zoom_enable_arg = DeclareLaunchArgument(
        'zoom_enable', default_value='False',
        description='Valid values are True or False'
    )
    sensor_type_arg = DeclareLaunchArgument(
        'sensor_type', default_value='Boson_640',
        description='Valid values are Boson_320 or Boson_640'
    )
    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url', default_value=f'file://{default_calibration_path}',
        description='Location of the camera calibration file'
    )

    # Define the node matching your old <node> tag configuration
    boson_camera_node = Node(
        package='flir_boson_usb2',
        executable='boson_camera_node',
        name='flir_boson_usb_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'frame_id': LaunchConfiguration('frame_id'),
            'dev': LaunchConfiguration('dev'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'video_mode': LaunchConfiguration('video_mode'),
            'zoom_enable': LaunchConfiguration('zoom_enable'),
            'sensor_type': LaunchConfiguration('sensor_type'),
            'camera_info_url': LaunchConfiguration('camera_info_url')
        }]
    )

    return LaunchDescription([
        namespace_arg,
        frame_id_arg,
        dev_arg,
        frame_rate_arg,
        video_mode_arg,
        zoom_enable_arg,
        sensor_type_arg,
        camera_info_url_arg,
        boson_camera_node
    ])
