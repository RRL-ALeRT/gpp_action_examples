#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import struct

H_UPPER_LIMIT = 0.5
H_RAMP_AND_GROUND = -0.3
H_GROUND = -0.6
START_BOX_SIZE_CUT = 6
GOING_FRONT = False
ROBOT_RADIUS = 1.2  # Specify the radius of the region to update (in meters)

class AvoidHolesMapper(Node):
    def __init__(self):
        super().__init__("AvoidHolesMapper")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.create_subscription(
            PointCloud2,
            '/octomap_point_cloud_centers',
            self.pointcloud_callback,
            1
        )
        self.occupancy_grid_publisher = self.create_publisher(
            OccupancyGrid,
            'map',
            1
        )

        self.resolution = 0.1  # Adjust the resolution as per your requirement
        width = int(30 / self.resolution) # Set the desired width of the grid map
        height = int(30 / self.resolution)  # Set the desired height of the grid map
        # Create a numpy array for the occupancy grid map
        self.grid_map = np.full((height, width), -1, dtype=np.float32)
        self.height_map = np.zeros((height, width), dtype=np.float32)

        self.first_time = True

    def parse_pointcloud2(self, msg):
        # Extract the field names and their offsets
        fields = msg.fields
        offsets = [field.offset for field in fields]
        point_step = msg.point_step

        # Extract the raw data from the message
        raw_data = msg.data

        # Iterate over the points and extract the (x, y, z) coordinates
        points = []
        for i in range(0, len(raw_data), point_step):
            x, y, z = struct.unpack_from('fff', raw_data, offset=i + offsets[0])
            points.append((x, y, z))

        return points

    def is_occupied(self, height_map, x, y):
        height = height_map[y, x]

        # # Walls for cells with height above H_UPPER_LIMIT
        # if H_UPPER_LIMIT < height:
        #     return True

        width = height_map.shape[1]
        height = height_map.shape[0]

        # Check if surrounding cells are present for cells between height H_RAMP_AND_GROUND and H_GROUND
        if H_GROUND <= height: 
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue  # Skip the current cell

                    nx = x + dx
                    ny = y + dy

                    # Check if the neighboring cell is within bounds
                    if 0 <= nx < width and 0 <= ny < height:
                        neighbor_height = height_map[ny, nx]

                        # Check if the neighboring cell height satisfies the condition
                        if H_GROUND <= neighbor_height <= H_RAMP_AND_GROUND:
                            return False

        return True

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                "body",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'{ex}')
            return None
        return [t.transform.translation.x, t.transform.translation.y]

    def pointcloud_callback(self, msg):
        points = self.parse_pointcloud2(msg)

        # Specify the resolution and ground height for 2D occupancy grid map
        self.resolution = 0.1  # Adjust the resolution as per your requirement
        ground_height = 0.0  # Set the height of the ground plane

        # Set the desired width and height of the occupancy grid map
        width = int(30 / self.resolution) # Set the desired width of the grid map
        height = int(30 / self.resolution)  # Set the desired height of the grid map

        # Calculate the origin of the occupancy grid map based on the desired width, height, and resolution
        origin = Pose()
        origin.position.x = -(width * self.resolution) / 2.0
        origin.position.y = -(height * self.resolution) / 2.0
        origin.position.z = ground_height

        # Iterate over each point and populate the occupancy grid map
        for point in points:
            x, y, z = point

            # Ignore point out of height limits
            if not -1 < z < 1:
                continue

            # Calculate the cell indices for the point
            cell_x = int((x - origin.position.x) / self.resolution)
            cell_y = int((y - origin.position.y) / self.resolution)

            # Check if the cell indices are within the grid map bounds
            if 0 <= cell_y < height and 0 <= cell_x < width:
                # Set the cell height
                self.height_map[cell_y, cell_x] = z - ground_height

        # Update the region around the robot's position
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return
        robot_x = int((robot_pose[0] - origin.position.x) / self.resolution)
        robot_y = int((robot_pose[1] - origin.position.y) / self.resolution)
        region_radius = ROBOT_RADIUS  # Specify the radius of the region to update (in meters)
        region_width = int(region_radius * 2 / self.resolution)
        region_height = int(region_radius * 2 / self.resolution)
        region_x_start = max(robot_x - region_width // 2, 0)
        region_x_end = min(region_x_start + region_width, width)
        region_y_start = max(robot_y - region_height // 2, 0)
        region_y_end = min(region_y_start + region_height, height)

        # First time set walls from 3 sides
        if self.first_time:
            self.first_time = False
            for y in range(region_y_start, region_y_end):
                for x in range(region_x_start, region_x_end):
                    if (
                        (x in range(region_x_start, region_x_start + START_BOX_SIZE_CUT) and GOING_FRONT) or  # Bottom wall
                        (x in range(region_x_end - START_BOX_SIZE_CUT, region_x_end) and not GOING_FRONT) or  # Top wall
                        y in range(region_y_start, region_y_start + START_BOX_SIZE_CUT) or # Right wall
                        y in range(region_y_end - START_BOX_SIZE_CUT, region_y_end) # Left wall
                    ):
                        self.height_map[y, x] = 1  # Set the height as 1, so that it'll be calculated as wall
                    else:
                        self.height_map[y, x] = H_RAMP_AND_GROUND

        # Iterate over the cells in the region and determine the occupancy
        for y in range(region_y_start, region_y_end):
            for x in range(region_x_start, region_x_end):
                if self.is_occupied(self.height_map, x, y):
                    self.grid_map[y, x] = 100  # Set the cell as occupied
                else:
                    self.grid_map[y, x] = 0  # Set the cell as unknown

        # Convert the numpy array to a 1D list and assign it to the occupancy grid message data field
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header = msg.header
        occupancy_grid_msg.header.frame_id = "map"
        occupancy_grid_msg.info.resolution = self.resolution
        occupancy_grid_msg.info.width = width
        occupancy_grid_msg.info.height = height
        occupancy_grid_msg.info.origin = origin
        occupancy_grid_msg.data = self.grid_map.flatten().astype(int).tolist()

        # Publish the 2D occupancy grid map
        self.occupancy_grid_publisher.publish(occupancy_grid_msg)


def main():
    rclpy.init()
    rclpy.spin(AvoidHolesMapper())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
