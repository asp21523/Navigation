#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from queue import PriorityQueue

class GlobalPlanner:
    def __init__(self):
        
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)

        self.map_received = False
        self.grid = None
        self.resolution = None
        self.origin = None
        self.width = 0
        self.height = 0

        self.start = None  # set from odom
        self.goal = None

        rospy.loginfo("A* Global Planner initialized. Waiting for map, odom, and goal...")

    def map_callback(self, msg):
        rospy.loginfo("Map received.")
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        data = np.array(msg.data).reshape((self.height, self.width))

        # Occupied = 1, Free = 0
        self.grid = np.where(data >= 100, 1, 0)
        self.map_received = True

    def odom_callback(self, msg):
        if not self.map_received:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        i = int((y - self.origin.position.y) / self.resolution)
        j = int((x - self.origin.position.x) / self.resolution)

        if 0 <= i < self.height and 0 <= j < self.width:
            self.start = (i, j)

    def goal_callback(self, msg):
        if not self.map_received:
            rospy.logwarn("Map not received yet.")
            return

        if self.start is None:
            rospy.logwarn("Start position not yet known from odom.")
            return

        gx = msg.pose.position.x
        gy = msg.pose.position.y

        j = int((gx - self.origin.position.x) / self.resolution)
        i = int((gy - self.origin.position.y) / self.resolution)

        if not (0 <= i < self.height and 0 <= j < self.width):
            rospy.logwarn("Goal position out of map bounds!")
            return

        self.goal = (i, j)

        rospy.loginfo(f"Planning path from {self.start} to {self.goal}")

        if self.grid[self.start[0], self.start[1]] != 0:
            rospy.logwarn("Start cell is occupied!")
            return
        if self.grid[self.goal[0], self.goal[1]] != 0:
            rospy.logwarn("Goal cell is occupied!")
            return

        self.plan()

    def get_neighbors(self, current, connectivity=8):
        x, y = current
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]
        neighbors = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.height and 0 <= ny < self.width:
                if self.grid[nx, ny] == 0:
                    neighbors.append((nx, ny))
        return neighbors

    def heuristic(self, a, b):
        # Octile distance for 8-connected grid
        dx = abs(a[1] - b[1])
        dy = abs(a[0] - b[0])
        D = 1
        D2 = np.sqrt(2)
        return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

    def plan(self):
        if not self.map_received or self.goal is None or self.start is None:
            rospy.logwarn("Map, start, or goal not available for planning.")
            return

        open_set = PriorityQueue()
        open_set.put((0, self.start))
        came_from = {}
        cost = {self.start: 0}

        while not open_set.empty():
            _, current = open_set.get()

            if current == self.goal:
                rospy.loginfo("Goal reached.")
                break

            for neighbor in self.get_neighbors(current):
                move_cost = np.sqrt(2) if abs(neighbor[0] - current[0]) + abs(neighbor[1] - current[1]) == 2 else 1
                new_cost = cost[current] + move_cost

                if neighbor not in cost or new_cost < cost[neighbor]:
                    cost[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, self.goal)
                    open_set.put((priority, neighbor))
                    came_from[neighbor] = current

        if self.goal in came_from:
            path = []
            node = self.goal
            while node != self.start:
                path.append(node)
                node = came_from[node]
            path.append(self.start)
            path.reverse()
            self.publish_path(path)
        else:
            rospy.logwarn("No path found to the goal.")

    def publish_path(self, path):
        ros_path = Path()
        ros_path.header.stamp = rospy.Time.now()
        ros_path.header.frame_id = "map"

        for (i, j) in path:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = j * self.resolution + self.origin.position.x + self.resolution / 2.0
            pose.pose.position.y = i * self.resolution + self.origin.position.y + self.resolution / 2.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)

        self.path_pub.publish(ros_path)
        rospy.loginfo(f"Path published with {len(path)} waypoints.")

if __name__ == '__main__':
    planner = GlobalPlanner()
    rospy.spin()
