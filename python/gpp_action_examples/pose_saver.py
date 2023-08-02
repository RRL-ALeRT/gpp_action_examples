#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Joy
from tf2_ros import StaticTransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from copy import deepcopy


class PoseSaverNode(Node):
    def __init__(self):
        super().__init__('pose_saver_node')
        self.subscription = self.create_subscription(Joy, '/joy_spot', self.button_callback, 1)
        self.subscription = self.create_subscription(Joy, '/joy_kinova', self.button_callback_kinova, 1)

        self.tf_buffer = Buffer()
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.frame_count = 0

        self.surity_flag = False
        self.tf_stamped_list = []

        self.tf_stamped = TransformStamped()

        self.height_pub = self.create_publisher(Float64, '/spot_height', 1)
        self.marker_pub = self.create_publisher(Marker, '/marker_single_frame', 1)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/marker_all_frames', 1)

        self.height = 0.0

        self.done = False

    def button_callback_kinova(self, msg):
        if msg.buttons[11] and not self.done:
            self.done = True
            self.height = 100.0
            height = Float64()
            height.data = self.height
            self.height_pub.publish(height)

    def button_callback(self, msg):
        if msg.buttons[3] == 1:
            self.height = 0.0
        elif msg.axes[7] != 0:
            self.height = min(msg.axes[7], 0.2)
            self.height = max(self.height, -0.2)

        if msg.axes[6] == -1:
            self.surity_flag = True

            from_frame_rel = 'body'
            to_frame_rel = 'map'

            try:
                t = self.tf_buffer.lookup_transform(
                    to_frame_rel,
                    from_frame_rel,
                    rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                return

            self.tf_stamped.header.stamp = self.get_clock().now().to_msg()
            self.tf_stamped.header.frame_id = to_frame_rel
            self.tf_stamped.child_frame_id = f"spot_{self.frame_count}"

            self.tf_stamped.transform.translation.x = t.transform.translation.x
            self.tf_stamped.transform.translation.y = t.transform.translation.y
            self.tf_stamped.transform.translation.z = t.transform.translation.z

            self.tf_stamped.transform.rotation.x = t.transform.rotation.x
            self.tf_stamped.transform.rotation.y = t.transform.rotation.y
            self.tf_stamped.transform.rotation.z = t.transform.rotation.z
            self.tf_stamped.transform.rotation.w = t.transform.rotation.w

            self.tf_static_broadcaster.sendTransform(self.tf_stamped)

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'new_pose'
            marker.id = 0
            marker.type = Marker.ARROW

            marker.pose.position.x = t.transform.translation.x
            marker.pose.position.y = t.transform.translation.y
            marker.pose.position.z = t.transform.translation.z

            marker.pose.orientation.x = t.transform.rotation.x
            marker.pose.orientation.y = t.transform.rotation.y
            marker.pose.orientation.z = t.transform.rotation.z
            marker.pose.orientation.w = t.transform.rotation.w

            marker.scale.x = 0.6
            marker.scale.y = 0.2
            marker.scale.z = 0.1

            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
    
            self.marker_pub.publish(marker)
            
        if msg.axes[6] == 1 and self.surity_flag:
            self.surity_flag = False
            self.tf_stamped_list.append(deepcopy(self.tf_stamped))
            self.frame_count += 1

            marker_array = MarkerArray()
            for i, t in enumerate(self.tf_stamped_list):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.ns = 'poses'
                marker.type = Marker.ARROW
                marker.id = i

                marker.pose.position.x = t.transform.translation.x
                marker.pose.position.y = t.transform.translation.y
                marker.pose.position.z = t.transform.translation.z

                marker.pose.orientation.x = t.transform.rotation.x
                marker.pose.orientation.y = t.transform.rotation.y
                marker.pose.orientation.z = t.transform.rotation.z
                marker.pose.orientation.w = t.transform.rotation.w

                marker.scale.x = 0.6
                marker.scale.y = 0.2
                marker.scale.z = 0.1

                marker.color.a = 0.8
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                marker_array.markers.append(marker)
        
            self.marker_array_pub.publish(marker_array)

            height = Float64()
            height.data = self.height
            self.height_pub.publish(height)


def main(args=None):
    rclpy.init(args=args)
    pose_saver_node = PoseSaverNode()
    rclpy.spin(pose_saver_node)
    pose_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
