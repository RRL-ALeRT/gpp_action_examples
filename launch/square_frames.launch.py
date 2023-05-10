from launch import LaunchDescription
from launch_ros.actions import Node

import math
import numpy as np


def quaternion_from_euler(ak):
    ai = 0
    aj = 0
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    q = [str(qq) for qq in q]
    print(q)
    return q


my_world = \
{
    "frame1": [[0.5, 0.0], 90.],
    "frame2": [[0.5, 0.5], 180.],
    "frame3": [[0.0, 0.5], -90.],
    "frame4": [[0.0, 0.0], 0.],
}


def generate_launch_description():
    tfs = []

    for frame, pose in my_world.items():
        position = [str(p) for p in pose[0]]
        rotation = pose[1]
        quat = quaternion_from_euler(np.deg2rad(rotation))
        node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[position[0], position[1], '0.0',
                       quat[0], quat[1], quat[2], quat[3],
                       'start_frame', frame],
        )
        tfs.append(node)
    
    start_frame_tf2 = Node(
        package='gpp_action_examples',
        executable='vision_start_frame_tf2',
        output='screen',
    )
    tfs.append(start_frame_tf2)

    start_frame_tf2 = Node(
        package='gpp_action_examples',
        executable='spot_body_pose',
        output='screen',
    )
    tfs.append(start_frame_tf2)
    
    start_frame_tf2 = Node(
        package='gpp_action_examples',
        executable='spot_body_pose',
        output='screen',
    )
    tfs.append(start_frame_tf2)

    spot_trajectory_to_frame_server = Node(
        package='gpp_action_examples',
        executable='gpp_trajectory.py',
        output='screen',
    )
    tfs.append(spot_trajectory_to_frame_server)

    audio_server = Node(
        package='gpp_action_examples',
        executable='audio_action_server.py',
        output='screen',
    )
    tfs.append(audio_server)

    return LaunchDescription(tfs)

