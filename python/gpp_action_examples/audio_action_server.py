#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from gpp_action_examples_interface.action import PlayAudio
from playsound import playsound

from ament_index_python.packages import get_package_share_directory


class AudioServer(Node):
    def __init__(self):
        super().__init__('audio_server')
        self._as = ActionServer(self, PlayAudio, 'play_audio', self.execute_callback)

    def execute_callback(self, goal_handle):
        # Start playing the audio file
        audio_file_path = get_package_share_directory('gpp_action_examples') + '/sounds/' + goal_handle.request.audio_file + '.mp3'
        playsound(audio_file_path, True)

        # Set the result of the action
        result = PlayAudio.Result()
        result.success = True
        goal_handle.succeed()

        return result

def main(args=None):
    rclpy.init(args=args)
    server = AudioServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
