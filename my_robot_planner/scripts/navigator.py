#!/usr/bin/env python3

import rospy
import threading

# Import your modules as packages if they're in the same directory
from global_planner import GlobalPlanner
from kinematic_controller import controller
from potential_fields import PotentialFieldPlanner

if __name__ == '__main__':
    try:
        # Initialize ROS once for the combined node
        rospy.init_node('navigation_manager', anonymous=True)

        # Launch each part of the navigation stack in separate threads
        rospy.loginfo("Starting Global Planner...")
        gp_thread = threading.Thread(target=GlobalPlanner)
        gp_thread.daemon = True
        gp_thread.start()

        rospy.sleep(1.0)

        rospy.loginfo("Starting Kinematic Controller...")
        ctrl_thread = threading.Thread(target=controller)
        ctrl_thread.daemon = True
        ctrl_thread.start()

        rospy.sleep(1.0)

        rospy.loginfo("Starting Potential Fields Planner...")
        pf_thread = threading.Thread(target=PotentialFieldPlanner)
        pf_thread.daemon = True
        pf_thread.start()

        # Spin ROS to keep everything alive
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation node shutting down.")
