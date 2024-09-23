#!/usr/bin/env python

import rospy
import actionlib
import tf2_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
import tf
import os

class DynamicPlannerNode:
    def __init__(self):
        rospy.init_node('dynamic_planner_node')
        rospy.loginfo('Initialized dynamic planner node')

        # Initialize the action client for move_base
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()

        # Initialize the goal message
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.orientation.w = 1.0  # Default orientation

        # Initialize start pose variables
        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        # Set up a TF listener to get the robot's current pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribe to the RViz goal topic
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # Open file for logging results
        log_file_path = "/home/glab/catkin_ws/src/teb_planner_hj/src/result_log.txt"  # Adjust the path as needed
        try:
            self.result_file = open(log_file_path, "a")
            rospy.loginfo('Opened %s for writing', log_file_path)
        except IOError:
            rospy.logerr("Failed to open file at %s", log_file_path)
            rospy.signal_shutdown("File opening failed.")

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            orientation = trans.transform.rotation
            _, _, yaw = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            return x, y, yaw
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Failed to get robot pose: %s" % e)
            return None

    def goal_callback(self, msg):
        rospy.loginfo('Received goal position from RViz: x=%f, y=%f, orientation_w=%f',
                      msg.pose.position.x,
                      msg.pose.position.y,
                      msg.pose.orientation.w)

        # Get the current robot pose
        start_pose = self.get_robot_pose()
        if start_pose:
            self.start_x, self.start_y, self.start_yaw = start_pose
            rospy.loginfo('Start position: x=%f, y=%f, yaw=%f', self.start_x, self.start_y, self.start_yaw)
            # Log the start position
            self.result_file.write("Start position: x: {}, y: {}, yaw: {}\n".format(self.start_x, self.start_y, self.start_yaw))
        else:
            rospy.logerr('Failed to get start pose')
            return

        # Move to the new goal
        self.move_to_goal(msg.pose.position.x, msg.pose.position.y, msg.pose.orientation.w)

    def move_to_goal(self, x, y, w):
        rospy.loginfo('Moving to goal position')
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.orientation.w = w
        self.move_client.send_goal(self.goal)

        # Wait for the result
        wait = self.move_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            self.result_file.write("Failed to connect to action server\n")
            rospy.signal_shutdown("Action server not available!")
        else:
            if self.move_client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo('Goal reached successfully!')
                if self.start_x is not None and self.start_y is not None:
                    self.result_file.write("Success: Start -> x: {}, y: {}, yaw: {} | Goal -> x: {}, y: {}, orientation_w: {}\n".format(
                        self.start_x, self.start_y, self.start_yaw, x, y, w))
                else:
                    self.result_file.write("Success: Goal -> x: {}, y: {}, orientation_w: {}\n".format(
                        x, y, w))
            else:
                rospy.loginfo('Failed to reach the goal.')
                if self.start_x is not None and self.start_y is not None:
                    self.result_file.write("Failure: Start -> x: {}, y: {}, yaw: {} | Goal -> x: {}, y: {}, orientation_w: {}\n".format(
                        self.start_x, self.start_y, self.start_yaw, x, y, w))
                else:
                    self.result_file.write("Failure: Goal -> x: {}, y: {}, orientation_w: {}\n".format(
                        x, y, w))

    def main(self):
        rospy.loginfo('Waiting for goals from RViz...')
        rospy.spin()  # Keep the node running

    def __del__(self):
        if hasattr(self, 'result_file') and not self.result_file.closed:
            self.result_file.close()
            rospy.loginfo('Closed result_log.txt')

if __name__ == '__main__':
    node = DynamicPlannerNode()
    node.main()
