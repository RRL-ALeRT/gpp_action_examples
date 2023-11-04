#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from gpp_action_examples_interface.action import Durative
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory


class MoveCircleServer(Node):
    def __init__(self):
        super().__init__('move_circle_as')
        self._as = ActionServer(self, Durative, 'move_circle',
        execute_callback=self.execute_callback,
        callback_group=ReentrantCallbackGroup(),
        goal_callback=self.goal_callback,
        cancel_callback=self.cancel_callback
        )
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def goal_callback(self, goal_request):
        self.goal_done = False
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.goal_done = True
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 1.0
        cmd_vel_msg.angular.z = 1.0

        duration = goal_handle.request.duration

        if duration.sec <= 0:
            self.get_logger().info('Goal aborted')
            return Durative.Result()

        start_time = self.get_clock().now().to_msg()
        current_time = self.get_clock().now().to_msg()
        in_time = True

        while in_time:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Durative.Result()
            current_time = self.get_clock().now().to_msg()
            time_diff = current_time.sec - start_time.sec
            if time_diff >= duration.sec:
                in_time = False

            self.cmd_vel_pub_.publish(cmd_vel_msg)
        print(time_diff)
        print("Done")
        result = Durative.Result()
        result.success = True
        goal_handle.succeed()

        return result

def main(args=None):
    rclpy.init(args=args)
    server = MoveCircleServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(server, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
