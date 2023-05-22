#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from gpp_action_examples_interface.action import Exploration

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
import tf2_ros
from visualization_msgs.msg import Marker

import numpy as np
import heapq, math, random
import scipy.interpolate as si

lookahead_distance = 0.4 # The distance to look ahead (in meters) for path planning.
speed = 0.5 # The maximum speed of the robot (in meters per second).
expansion_size = 6 # The factor by which obstacles are expanded during path planning.
target_error = 0.4 # The acceptable error margin (in meters) for reaching the target location.
robot_r = 0.4 # The safety distance around the robot (in meters) for local obstacle avoidance.


def euler_from_quaternion(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z


def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def astar(array, start, goal):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    open_set = {start}

    while oheap:
        current = heapq.heappop(oheap)[1]
        
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            data.reverse()
            return data
        
        close_set.add(current)
        
        if current in open_set:
            open_set.remove(current)
        
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            
            if neighbor[0] < 0 or neighbor[0] >= array.shape[0] or neighbor[1] < 0 or neighbor[1] >= array.shape[1]:
                continue
            
            if array[neighbor[0]][neighbor[1]] == 1:
                continue
            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in open_set:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
                open_set.add(neighbor)
    
    # If no path to goal was found, return closest path to goal
    if goal not in came_from:
        closest_node = None
        closest_dist = float('inf')
        for node in close_set:
            dist = heuristic(node, goal)
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            data = []
            while closest_node in came_from:
                data.append(closest_node)
                closest_node = came_from[closest_node]
            data.append(start)
            data.reverse()
            return data
    
    return False


def bspline_planning(array, sn):
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 2
        t = np.arange(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)

        x_list = list(x_tup)
        x_list[1] = np.pad(x_list[1], (0, 4), mode='constant')

        y_list = list(y_tup)
        y_list[1] = np.pad(y_list[1], (0, 4), mode='constant')

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)

        path = list(zip(rx, ry))
    except (TypeError, ValueError):
        path = array.tolist()

    return path


def pure_pursuit(current_x, current_y, current_heading, path, index, forward):
    global lookahead_distance
    closest_point = None
    forward = 1 if forward else -1
    v = forward * speed  # Set the speed to a negative value to make the robot go in reverse
    for i in range(index, len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(forward*(closest_point[1] - current_y), forward*(closest_point[0] - current_x))
        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(forward*(path[-1][1] - current_y), forward*(path[-1][0] - current_x))
        desired_steering_angle = target_heading - current_heading
        index = len(path) - 1
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if desired_steering_angle > math.pi / 6 or desired_steering_angle < -math.pi / 6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = (sign * math.pi / 4)
        v = 0.0
    return v, desired_steering_angle, index


def frontierB(matrix):
    for i, row in enumerate(matrix):
        for j, value in enumerate(row):
            if value == 0.0:
                if (i > 0 and matrix[i - 1][j] < 0) or \
                   (i < len(matrix) - 1 and matrix[i + 1][j] < 0) or \
                   (j > 0 and matrix[i][j - 1] < 0) or \
                   (j < len(row) - 1 and matrix[i][j + 1] < 0):
                    matrix[i][j] = 2

    return matrix


def assign_groups(matrix):
    group = 1
    groups = {}
    visited = set()
    stack = []

    def dfs(matrix, i, j, group, groups):
        stack.append((i, j))
        while stack:
            x, y = stack.pop()
            if (x, y) in visited:
                continue
            visited.add((x, y))
            if matrix[x][y] != 2:
                continue
            if group in groups:
                groups[group].append((x, y))
            else:
                groups[group] = [(x, y)]
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (-1, 1), (1, -1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < len(matrix) and 0 <= ny < len(matrix[0]):
                    stack.append((nx, ny))

        return group + 1

    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            if matrix[i][j] == 2 and (i, j) not in visited:
                group = dfs(matrix, i, j, group, groups)

    return matrix, groups


def fGroups(groups):
    sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
    top_five_groups = [g for g in sorted_groups[:5] if len(g[1]) > 2]    
    return top_five_groups


def calculate_centroid(x_coords, y_coords):
    return np.mean(x_coords, dtype=int), np.mean(y_coords, dtype=int)


#This function selects the one that is farther than target_error*2 from the top 5 groups and closest to the robot.
def findClosestGroup(matrix, groups, current, resolution, originX, originY):
    targetP = None
    distances = np.zeros(len(groups))
    paths = [None] * len(groups)
    score = np.zeros(len(groups))
    max_score = None  # max score index
    num_groups = len(groups)
    
    for i, (_, points) in enumerate(groups):
        middle = (np.mean([p[0] for p in points]), np.mean([p[1] for p in points]))
        path = astar(matrix, current, middle)
        path = [(p[1] * resolution + originX, p[0] * resolution + originY) for p in path]
        paths[i] = path
        distances[i] = pathLength(path)
    
    for i in range(num_groups):
        if distances[i] != 0:
            score[i] = len(groups[i][1]) / distances[i]
    
    for i in range(num_groups):
        if distances[i] > target_error * 3:
            if max_score is None or score[i] > score[max_score]:
                max_score = i
    
    if max_score is not None:
        targetP = paths[max_score]
    else:  # selects a random point as a target if groups are closer than target_error*2 distance.
        index = random.randint(0, num_groups - 1)
        target = groups[index][1]
        target = target[random.randint(0, len(target) - 1)]
        path = astar(matrix, current, target)
        targetP = [(p[1] * resolution + originX, p[0] * resolution + originY) for p in path]
    
    return targetP


def pathLength(path):
    for i in range(len(path)):
        path[i] = (path[i][0],path[i][1])
        points = np.array(path)
    differences = np.diff(points, axis=0)
    distances = np.hypot(differences[:,0], differences[:,1])
    total_distance = np.sum(distances)
    return total_distance

def costmap(data,width,height,resolution):
    data = np.array(data).reshape(height,width)
    wall = np.where(data == 100)
    for i in range(-expansion_size,expansion_size+1):
        for j in range(-expansion_size,expansion_size+1):
            if i  == 0 and j == 0:
                continue
            x = wall[0]+i
            y = wall[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100
    data = data*resolution
    return data


def localControl(scan):
    v = None
    w = None
    for i in range(60):
        if scan[i] < robot_r:
            v = 0.2
            w = -math.pi/4 
            break
    if v == None:
        for i in range(300,360):
            if scan[i] < robot_r:
                v = 0.2
                w = math.pi/4
                break
    return v,w


class navigationControl(Node):
    def __init__(self):
        super().__init__('Exploration')
        self.subscription = self.create_subscription(OccupancyGrid,'map',self.map_callback,1)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=rclpy.qos.QoSProfile(
                reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
                depth=1,
                durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
            )
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        
        self.path_publisher = self.create_publisher(Marker, 'path_marker', 1)
        self.frontier_publisher = self.create_publisher(Marker, 'frontier_marker', 1)

        # Initialize the transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.start_time = 0
        self.target_count = 0

        self._action_server = ActionServer(
            self,
            Exploration,
            'explore_frontier',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
    def cancel_callback(self, goal_handle):
        self.goal_done = True
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def goal_callback(self, goal_request):
        self.goal_done = False
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute a goal."""
        self.get_logger().info('Executing goal...')

        self.result = Exploration.Result()
        self.result.success = False

        robot_frame = goal_handle.request.robot_frame
        odom_frame = goal_handle.request.odom_frame
        forward = goal_handle.request.forward

        feedback_msg = Exploration.Feedback()
        feedback_msg.status = 'exploring'

        rate = self.create_rate(20)

        self.get_logger().info("DISCOVER MODE ACTIVE")
        self.pathGlobal = 0
        self.discovery = True

        while (not self.goal_done) and (rclpy.ok()):
            self.exp(robot_frame, odom_frame, forward)
            rate.sleep()

        goal_handle.succeed()

        return self.result
    
    def exp(self, robot_frame, odom_frame, forward):
        twist = Twist()

        if not hasattr(self,'map_data'):
            self.get_logger().warn("Could not get map data")
            return
        
        if not hasattr(self,'scan_data'):
            self.get_logger().warn("Could not get scan data")
            return

        try:
            # Get the transform from "map" frame to "odom" frame
            self.transform = self.tf_buffer.lookup_transform(odom_frame, robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))

            # Extract the robot's position and orientation in the "map" frame
            self.x = self.transform.transform.translation.x
            self.y = self.transform.transform.translation.y
            self.yaw = euler_from_quaternion(self.transform.transform.rotation.x,
                                                self.transform.transform.rotation.y,
                                                self.transform.transform.rotation.z,
                                                self.transform.transform.rotation.w)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            # Log a warning if the transform cannot be obtained
            self.get_logger().warn("Could not get transform from map to odom: {}".format(ex))
            return

        if self.discovery:
            if isinstance(self.pathGlobal, int) and self.pathGlobal == 0:
                column = int((self.x - self.originX)/self.resolution)
                row = int((self.y- self.originY)/self.resolution)
                self.exploration(self.data,self.width,self.height,self.resolution,column,row,self.originX,self.originY)
                self.path = self.pathGlobal
            else:
                self.path = self.pathGlobal
            if isinstance(self.path, int) and self.path == -1:
                self.get_logger().info("EXPLORATION COMPLETED")
                self.result.success = True
                self.goal_done = True
                return
            self.c = int((self.path[-1][0] - self.originX)/self.resolution) 
            self.r = int((self.path[-1][1] - self.originY)/self.resolution) 
            self.discovery = False
            self.i = 0

            if self.get_clock().now().to_msg().sec > self.start_time + 10:
                self.start_time = self.get_clock().now().to_msg().sec
                self.target_count = 0
            self.target_count += 1
            if self.target_count > 5:
                self.get_logger().info("5 new targets were created in 10 secs, stopping exploration")
                self.result.success = False
                self.goal_done = True
                return

            self.reset_time = self.get_clock().now().to_msg().sec + 10
            self.get_logger().info("NEW TARGET SET")

        #Route Tracking Block Start
        else:
            v = None
            if v == None:
                v, w,self.i = pure_pursuit(self.x,self.y,self.yaw,self.path,self.i,forward)

            if(abs(self.x - self.path[-1][0]) < target_error and abs(self.y - self.path[-1][1]) < target_error):
                v = 0.0
                w = 0.0
                self.discovery = True
                self.get_logger().info("TARGET REACHED")
                self.pathGlobal = 0
                self.scan_data = None
                self.map_data = None

            if(self.get_clock().now().to_msg().sec >= self.reset_time):
                v = 0.0
                w = 0.0
                self.discovery = True
                self.get_logger().info("TARGET RESETTED AFTER 10 SECS")
                self.pathGlobal = 0
                self.scan_data = None
                self.map_data = None

            twist.linear.x = v
            twist.angular.z = w
            self.publisher.publish(twist)
        #Route Follow Block End

    def target_callback(self):
        self.exploration(self.data,self.width,self.height,self.resolution,self.c,self.r,self.originX,self.originY)
        
    def scan_callback(self,msg):
        self.scan_data = msg
        self.scan = msg.ranges

    def map_callback(self,msg):
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data

    def exploration(self, data,width,height,resolution,column,row,originX,originY):
        data = costmap(data,width,height,resolution) #Expand the obstacles
        data[row][column] = 0 #Robot Instant Location
        data[data > 5] = 1 # 0 is a place to go, 100 is a definite obstacle
        data = frontierB(data) #Find boundary points
        data,groups = assign_groups(data) #Group boundary points
        groups = fGroups(groups) # Sort the groups from smallest to largest. Get the top 5 groups
        if len(groups) == 0: #Discovery complete if there is no group
            path = -1
        else: #Find the closest group if there is a group
            data[data < 0] = 1 #-0.05 ones unknown location. Mark as not allowed. 0 = can go, 1 = not go.
            path = findClosestGroup(data,groups,(row,column),resolution,originX,originY) #Find the nearest group
            if path != None: #Fix with BSpline if path exists
                path = bspline_planning(path,len(path)*5)
            else:
                path = -1
        self.pathGlobal = path

        # Publish path marker
        path_marker = Marker()
        path_marker.header.frame_id = 'map'
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.1  # Line width
        path_marker.color.a = 1.0  # Alpha
        path_marker.color.r = 0.0  # Red
        path_marker.color.g = 1.0  # Green
        path_marker.color.b = 0.0  # Blue

        if len(groups) != 0:
            # Add path points
            for point in path:
                pose = Pose()
                pose.position.x = point[0]
                pose.position.y = point[1]
                path_marker.points.append(pose.position)

            self.path_publisher.publish(path_marker)

        return

def main(args=None):
    rclpy.init(args=args)
    navigation_control = navigationControl()
    
    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(navigation_control, executor=executor)

    navigation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
