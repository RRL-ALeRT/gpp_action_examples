from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    spot_trajectory_to_frame_server = Node(
        package='gpp_action_examples',
        executable='gpp_trajectory.py',
        output='screen',
    )

    pose_saver = Node(
        package='gpp_action_examples',
        executable='pose_saver.py',
        output='screen',
    )

    spot_body_pose_service = Node(
        package='gpp_action_examples',
        executable='spot_body_pose',
        output='screen',
    )

    gologpp_agent = Node(
        package='gologpp_agent',
        executable='gologpp_agent',
        output='screen',
        parameters=[{'gpp_code': f"{get_package_share_directory('gpp_action_examples')}/resource/frame_runner.gpp"}],
    )

    return LaunchDescription([spot_trajectory_to_frame_server, pose_saver, spot_body_pose_service, gologpp_agent])
