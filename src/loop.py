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

        timeout = 100

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
            # Define sequences of start and goal positions
        sequences = [
        # Sequence 1
        {'start': (1.824033, 6.769977, 0.096273, -1.287503), 'goal': (6.768701, 1.057576, 0.000000, 0.773270)},
        {'start': (0.985526, 6.917296, 0.096226, 0.027548), 'goal': (5.232610, 1.684851, 0.000000, 0.999823)},
        {'start': (5.269928, 1.684734, 0.096243, 0.025992), 'goal': (1.989110, 5.906961, 0.000000, 0.716123)},
        # Sequence 2
        #{'start': (6.986666, 0.376367, 0.096228, -2.253656), 'goal': (-1.831612, 1.669143, 0.000000, -0.014315)},
        {'start': (6.788350, -0.089891, 0.096250, -3.096101), 'goal': (-1.868525, 1.650294, 0.000000, -0.014738)},
        {'start': (6.805717, 0.113735, 0.096264, -2.119764), 'goal': (-0.554213, 1.652943, 0.000000, 0.014982)},
        # Sequence 3
        {'start': (-0.197335, 1.673094, 0.096252, 3.128338), 'goal': (-5.710883, -1.709810, 0.000000, -0.011583)},
        {'start': (-2.574699, 1.653359, 0.096229, 3.141128), 'goal': (-7.020387, -1.723354, 0.000000, 0.603660)},
        {'start': (-3.642075, 1.760398, 0.096239, -3.131948), 'goal': (-6.064654, -1.638782, 0.000000, -0.077751)},
        # Sequence 4
        {'start': (-5.377285, -0.750267, 0.096239, -1.742384), 'goal': (-5.645093, -5.850183, 0.000000, 1.000000)},
        {'start': (-5.485104, -4.714368, 0.096235, 1.341129), 'goal': (-2.009097, -3.007130, 0.000000, 0.999909)},
        {'start': (-5.496230, -4.258798, 0.096249, 1.746675), 'goal': (-4.039498, -3.428050, 0.000000, 0.996223)},
        {'start': (-5.487377, -5.311635, 0.096243, 1.494585), 'goal': (-3.880702, -3.380883, 0.000000, 0.999839)},
       # Sequence 6
        {'start': (-2.563111, -3.040125, 0.096228, -0.078109), 'goal': (1.376936, -6.336720, 0.000000, 0.999945)},
        {'start': (1.504560, -6.190240, 0.096256, 0.578695), 'goal': (3.139581, -2.103034, 0.000000, 0.999866)},
        {'start': (-1.074328, -6.349617, 0.096226, 0.018165), 'goal': (3.127539, -0.420866, 0.000000, 0.999835)},
        {'start': (3.177052, -0.367005, 0.096233, 0.014759), 'goal': (3.895634, -5.610425, 0.000000, 0.998171)},
        # Sequence 7
        {'start': (1.430136, -6.335843, 0.096231, 0.057171), 'goal': (2.920930, -2.147175, 0.000000, 0.999424)},
        {'start': (3.687236, -1.241461, 0.096225, -1.591248), 'goal': (5.276931, -3.293713, 0.000000, 0.711566)},
        {'start': (4.203457, -0.809600, 0.096230, 0.235379), 'goal': (6.733112, -5.549973, 0.000000, -0.369820)}
        ]

        
        # Loop through each start-goal pair
        for i, seq in enumerate(sequences):
            rospy.sleep(5)
            rospy.loginfo("Cycle %d/%d: Teleporting to start and sending goal...", i + 1, len(sequences))
        
            # Update start and goal coordinates
            self.start_coords = seq['start']
            self.goal_coords = seq['goal']
        
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
