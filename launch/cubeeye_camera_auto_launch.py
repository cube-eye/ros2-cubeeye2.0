import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# launch camera node and select the camera with [serial number] and run it with [type]
# ex) ros2 launch cubeeye_camera cubeeye_camera_auto_launch.py serialnumber:='nntab005098108' frametype:='6'
def generate_launch_description():
    serialnumber = LaunchConfiguration("serialnumber")
    frametype = LaunchConfiguration("frametype")

    # connect a camera with its serial number
    serialnumber_arg = DeclareLaunchArgument('autorun_serialnumber', default_value='nntab005098108')
    # run amplitude and depth by default
    frametype_arg = DeclareLaunchArgument('autorun_frametype', default_value='6')

    cubeeye_node = Node(
        package='cubeeye_camera',
        executable='cubeeye_camera_node',
        name='cubeeye_camera_node',
        output='screen',
        parameters=[
            {"autorun_onoff" : True},
            {"autorun_serialnumber" : serialnumber},
            {"autorun_frametype" : frametype}
        ]
    )

    return LaunchDescription(
        [
            serialnumber_arg,
            frametype_arg,
            cubeeye_node,
        ])
