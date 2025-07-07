#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import atan2, pi

class PotentialFieldPlanner:
    def __init__(self):
        rospy.init_node('potential_field_planner')

        self.robot_pos = np.zeros(2)
        self.robot_th = 0.0
        self.goal = None
        self.obstacles = []

        self.step_size = 0.2
        self.influence_radius = 0.22

        # Tuned gains for smoother motion
        self.k_att = 0.3  # was 0.5
        self.k_rep = 1.0  # was 1.0

        self.resolution = None
        self.origin = None
        self.grid = None

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/planned_path', Path, self.path_callback)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(10)
        self.run()

    def odom_callback(self, msg):
        self.robot_pos[0] = msg.pose.pose.position.x
        self.robot_pos[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_th = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def path_callback(self, msg):
        if msg.poses:
            self.goal = np.array([msg.poses[-1].pose.position.x, msg.poses[-1].pose.position.y])

    def map_callback(self, msg):
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.grid = data

        self.obstacles.clear()
        for i in range(data.shape[0]):
            for j in range(data.shape[1]):
                if data[i, j] >= 50:
                    ox = j * self.resolution + self.origin.position.x
                    oy = i * self.resolution + self.origin.position.y
                    self.obstacles.append(np.array([ox, oy]))

    def attractive_force(self, pos):
        if self.goal is None:
            return np.zeros(2)
        return self.k_att * (self.goal - pos)

    def repulsive_force(self, pos):
        force = np.zeros(2)
        for obs in self.obstacles:
            diff = pos - obs
            dist = np.linalg.norm(diff)
            if dist < self.influence_radius and dist > 0.01:
                rep = self.k_rep * (1.0 / dist - 1.0 / self.influence_radius) / (dist ** 2)
                force += rep * (diff / dist)
        return force

    def calculate_force(self):
        att = self.attractive_force(self.robot_pos)
        rep = self.repulsive_force(self.robot_pos)
        return att + rep

    def run(self):
        max_linear_speed = 0.15
        max_angular_speed = 1.0

        while not rospy.is_shutdown():
            if self.goal is not None:
                total_force = self.calculate_force()
                norm = np.linalg.norm(total_force)
                if norm > 0.01:
                    direction = total_force / norm
                    cmd = Twist()
                    cmd.linear.x = max_linear_speed  # capped speed
                    angle_to_goal = atan2(direction[1], direction[0])
                    angle_diff = self.wrap_angle(angle_to_goal - self.robot_th)
                    ang_vel = 1.0 * angle_diff
                    # Clamp angular velocity
                    ang_vel = max(min(ang_vel, max_angular_speed), -max_angular_speed)
                    cmd.angular.z = ang_vel
                    self.cmd_pub.publish(cmd)
                else:
                    self.cmd_pub.publish(Twist())  # stop if no force
            else:
                self.cmd_pub.publish(Twist())  # no goal
            self.rate.sleep()

    def wrap_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle <= -pi:
            angle += 2 * pi
        return angle

if __name__ == '__main__':
    try:
        PotentialFieldPlanner()
    except rospy.ROSInterruptException:
        pass

