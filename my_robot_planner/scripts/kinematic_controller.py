#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from collections import deque
from math import atan2, sin, cos, sqrt, pi

# Globals for robot pose and waypoints
x, y, th = 0.0, 0.0, 0.0
waypoints = deque()

def wrap_angle(angle):
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle

# Get robot pose from odometry
def odom_callback(msg):
    global x, y, th
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    (_, _, th) = euler_from_quaternion([q.x, q.y, q.z, q.w])

# Get path from global planner
def path_callback(msg):
    global waypoints
    waypoints.clear()
    for pose in msg.poses:
        xg = pose.pose.position.x
        yg = pose.pose.position.y
        waypoints.append((xg, yg, 0.0))  # Assume theta = 0
    rospy.loginfo(f"Received {len(waypoints)} waypoints.")

# Main controller node function
def controller():
    global x, y, th, waypoints

    
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/planned_path', Path, path_callback)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(20)  # 20 Hz control loop
    cmd = Twist()

    # Controller gains (tuned lower for smoother control)
    k_rho = 0.5
    k_alpha = 2.0
    k_beta = -0.8
    threshold = 0.2

    # Velocity limits
    max_linear_speed = 0.15
    max_angular_speed = 1.0

    while not rospy.is_shutdown():
        if waypoints:
            xg, yg, thg = waypoints[0]
            dx = xg - x
            dy = yg - y
            rho = sqrt(dx**2 + dy**2)

            if rho < threshold:
                rospy.loginfo(f"Reached waypoint: ({xg:.2f}, {yg:.2f})")
                waypoints.popleft()
                continue

            alpha = wrap_angle(atan2(dy, dx) - th)
            beta = wrap_angle(thg - th - alpha)

            v = k_rho * rho
            w = k_alpha * alpha + k_beta * beta

            # Clamp velocities
            v = max(min(v, max_linear_speed), -max_linear_speed)
            w = max(min(w, max_angular_speed), -max_angular_speed)

            cmd.linear.x = v
            cmd.angular.z = w
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        cmd_pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass

