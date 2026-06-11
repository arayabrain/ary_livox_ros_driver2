"""MID360 driver as a lifecycle node.

The node is auto-CONFIGURED on launch (SDK up, LiDAR discovered) but NOT
activated: the LiDAR stays in standby with the motor off. Activate it to start
scanning, e.g. from the recorder web UI or:

    ros2 lifecycle set /livox_lidar_publisher activate
    ros2 lifecycle set /livox_lidar_publisher deactivate
"""

import os

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

################### user configure parameters for ros2 start ###################
xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

sleep_on_shutdown = True   # set False to keep the LiDAR spinning after the node exits
activate_timeout_ms = 5000  # how long on_activate waits for the LiDAR to ack motor-on

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code},
    {"sleep_on_shutdown": sleep_on_shutdown},
    {"activate_timeout_ms": activate_timeout_ms},
]


def generate_launch_description():
    livox_driver = LifecycleNode(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_lifecycle_node',
        name='livox_lidar_publisher',
        namespace='',
        output='screen',
        parameters=livox_ros2_params,
    )

    # Configure on startup; stay 'inactive' (motor off) until activated.
    configure_on_start = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(livox_driver),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    return LaunchDescription([
        livox_driver,
        configure_on_start,
    ])
