#!/usr/bin/env python

import time
import rospy
import actionlib
import cv2
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry

class TestPlannerNode:
    def __init__(self):
        print('Hello test planner node')
        
        # Initialize publishers for markers
        self.s_marker_pub = rospy.Publisher("/t_s_marker", Marker, queue_size=2)
        self.g_marker_pub = rospy.Publisher("/t_g_marker", Marker, queue_size=2)
        self.r_marker_pub = rospy.Publisher("/t_r_marker", Marker, queue_size=2)
                
        # Initialize the action client for move_base
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()
        
        # Initialize the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'

        # Initialize goal coordinates (in corridor)
        # self.start_coords = (1.533775, 6.788731, 0.000002, -1.106530) 
        # self.goal_coords = (6.476081, 1.717010, 0.000000, 0.997706) 
        
        # coordinates (in ball ostacle)
        # self.start_coords = (-3.585799, 4.137397, 0.096234, 2.599909) 
        # self.goal_coords = (-0.623410, 4.627364, 0.000000, 0.956708) 

        # coordinates (in ball ostacle)
        self.start_coords = (-4.190331, 4.173361, 0.096251, 0.002489) 
        self.goal_coords = (-0.062500, 4.591092, 0.000000, 0.951909)

        # Trajectory tracking
        self.trajectory_points = []
        rospy.Subscriber("/odom", Odometry, self.odometry_callback)

    def odometry_callback(self, msg):
        # Append the robot's position to trajectory_points
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.trajectory_points.append((x, y))

    def save_trajectory_image(self, filename):
        # Create an empty white image
        image = np.ones((800, 800, 3), dtype=np.uint8) * 255
        scale_factor = 20  # Scaling factor to adjust coordinates to image size
        
        # Convert trajectory points to image coordinates and draw
        for i in range(1, len(self.trajectory_points)):
            x1, y1 = self.trajectory_points[i - 1]
            x2, y2 = self.trajectory_points[i]
            img_x1 = int(x1 * scale_factor + 400)
            img_y1 = int(y1 * scale_factor + 400)
            img_x2 = int(x2 * scale_factor + 400)
            img_y2 = int(y2 * scale_factor + 400)
            cv2.line(image, (img_x1, img_y1), (img_x2, img_y2), (0, 0, 255), 2)  # Red line for path

        cv2.imwrite(filename, image)
        rospy.loginfo("Trajectory image saved to {}".format(filename))

    def setMarker(self, x, y, w, mode):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        
        marker.type = 2
        marker.id = 0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        if mode == 1:
            marker.color.r = 1.0
            marker.color.g = 0.0
        elif mode == 0:
            marker.color.r = 0.0
            marker.color.g = 1.0
        elif mode == 2:
            marker.color.r = 1.0
            marker.color.g = 1.0

        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = w
        
        return marker        

    def teleport_to_start(self):
        # Set the model state for teleportation
        state_msg = ModelState()
        state_msg.model_name = 'former'
        state_msg.pose.position.x = self.start_coords[0]
        state_msg.pose.position.y = self.start_coords[1]
        state_msg.pose.position.z = self.start_coords[2]
        state_msg.pose.orientation.w = self.start_coords[3]

        # Call the service to set the model state
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state(state_msg)
        rospy.loginfo('Teleported to start position: x=%f, y=%f, z=%f, orientation_w=%f', self.start_coords[0], self.start_coords[1], self.start_coords[2], self.start_coords[3])

    def send_goal(self, goal_coords):
        # Set the goal position
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = goal_coords[0]
        self.goal.target_pose.pose.position.y = goal_coords[1]
        self.goal.target_pose.pose.position.z = goal_coords[2]
        self.goal.target_pose.pose.orientation.w = goal_coords[3]
        
        self.move_client.send_goal(self.goal)
        rospy.loginfo('Sent goal: x=%f, y=%f, z=%f, orientation_w=%f', goal_coords[0], goal_coords[1], goal_coords[2], goal_coords[3])
        
        # Publish markers
        self.s_marker_pub.publish(self.setMarker(goal_coords[0], goal_coords[1], goal_coords[3], 0))
        self.g_marker_pub.publish(self.setMarker(goal_coords[0], goal_coords[1], goal_coords[3], 0))

        # Track the start time for timing
        start_time = time.time()

        # Wait for the result
        wait = self.move_client.wait_for_result()
        elapsed_time = time.time() - start_time
        
        if not wait:
            rospy.logerr("Action server not available or timeout!")
            self.move_client.cancel_goal()
        else:
            if self.move_client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo('Goal reached!!')
                self.r_marker_pub.publish(self.setMarker(goal_coords[0], goal_coords[1], goal_coords[3], 1))
            else:
                rospy.loginfo('Failed to reach goal!!')

        rospy.logdebug("Time taken to reach goal: %.2f seconds", elapsed_time)
        return elapsed_time

    def main(self):
        rospy.loginfo("Node is running...")
        
        # Perform the teleport and goal-setting sequence 6 times
        for i in range(100):
            rospy.sleep(5)
            rospy.loginfo("Cycle %d/100: Teleporting to start and sending goal...", i + 1)
            
            # Clear previous trajectory points
            self.trajectory_points = []
            
            # Teleport to start position
            self.teleport_to_start()
            rospy.sleep(1)  # Small delay to ensure the robot is teleported before sending the goal
            
            # Send the robot to the goal
            time_taken = self.send_goal(self.goal_coords)
            
            # Save the trajectory as an image
            filename = "/home/glab/trajectory_image_cycle_{}.png".format(i + 1)
            self.save_trajectory_image(filename)
            
            rospy.loginfo("Cycle %d completed in %.2f seconds", i + 1, time_taken)

if __name__ == '__main__':
    rospy.init_node('test_planner_node')
    node = TestPlannerNode()
    node.main()
