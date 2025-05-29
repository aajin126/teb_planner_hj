#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker

class testPlannerNode:
    def __init__(self):
        print('Hello test planner node')
        
        # Initialize publishers for markers
        self.s_marker_pub = rospy.Publisher("/t_s_marker", Marker, queue_size=2)
        self.g_marker_pub = rospy.Publisher("/t_g_marker", Marker, queue_size=2)
        self.r_marker_pub = rospy.Publisher("/t_r_marker", Marker, queue_size=2)
                
        # Initialize the action client for move_base
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()
        
        # Define start and goal coordinates
        self.start_coords = (-0.6847482919692993, -0.8590271472930908, 0.0037508010864257812)  # (x, y, yaw)
        self.goal_coords = (2.1583986282348633, -1.171940565109253, 0.003467559814453125)  # Fixed goal coordinates
        #self.goal_coords = (-5.234235763549805, -1.9670383930206299, 0.0025968551635742188) 
        #self.goal_coords = (3.6356163024902344, -5.514806747436523, 0.005099296569824219) 

        self.state_msg = ModelState()
        self.state_msg.model_name = 'former'
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        
        self.reached = 0

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

    def resetPose(self):
        rospy.loginfo('Reset position!')
        
        # Set the robot's start position
        s_x, s_y, s_w = self.start_coords
        self.state_msg.pose.position.x = s_x
        self.state_msg.pose.position.y = s_y
        self.state_msg.pose.orientation.w = s_w
        
        # Set the goal position
        g_x, g_y, g_w = self.goal_coords
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = g_x
        self.goal.target_pose.pose.position.y = g_y
        self.goal.target_pose.pose.orientation.w = g_w
        
    def main(self):
        try:
            result_f = open("b_list.txt", "a")
            rospy.loginfo('File is opened!!')
        except:
            rospy.logerr("Failed to open file..")
            return False
        
        while not rospy.is_shutdown():
            rospy.wait_for_service('/gazebo/set_model_state')
            user_input = input("Enter 'start' to reset the robot to start position or 'goal' to move to the goal: ")
            
            if user_input == 'start':
                self.resetPose()
                try:
                    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                    resp = set_state(self.state_msg)
                    
                #     self.move_client.send_goal(self.goal)
                #     rospy.loginfo('Sent goal: from %f %f %f to %f %f %f', 
                #         self.state_msg.pose.position.x, 
                #         self.state_msg.pose.position.y, 
                #         self.state_msg.pose.orientation.w, 
                #         self.goal.target_pose.pose.position.x, 
                #         self.goal.target_pose.pose.position.y, 
                #         self.goal.target_pose.pose.orientation.w)
                    
                #     self.s_marker_pub.publish(self.setMarker(self.state_msg.pose.position.x, self.state_msg.pose.position.y, self.state_msg.pose.orientation.w, 0))
                #     self.g_marker_pub.publish(self.setMarker(self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y, self.goal.target_pose.pose.orientation.w, 0))

                #     wait = self.move_client.wait_for_result()
                #     if not wait:
                #         rospy.logerr("Action server not available!")
                #         rospy.signal_shutdown("Action server not available!")
                #     else:
                #         if self.move_client.get_state() == GoalStatus.SUCCEEDED:
                #             rospy.loginfo('Goal reached!!')
                #             self.r_marker_pub.publish(self.setMarker(self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y, self.goal.target_pose.pose.orientation.w, 1))
                #         else:
                #             rospy.loginfo('Failed to reach the goal!!')
                #             result_f.write("{0} {1} {2} {3} {4} {5}\n".format(
                #                 self.state_msg.pose.position.x,
                #                 self.state_msg.pose.position.y,
                #                 self.state_msg.pose.orientation.w,
                #                 self.goal.target_pose.pose.position.x,
                #                 self.goal.target_pose.pose.position.y,
                #                 self.goal.target_pose.pose.orientation.w))
                #             self.r_marker_pub.publish(self.setMarker(self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y, self.goal.target_pose.pose.orientation.w, 2))
                    
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s" % e)
            elif user_input == 'goal':
                #self.resetPose()
                try:
                    self.move_client.send_goal(self.goal)
                    rospy.loginfo('Sent goal: from %f %f %f to %f %f %f',
                        self.state_msg.pose.position.x,
                        self.state_msg.pose.position.y,
                        self.state_msg.pose.orientation.w,
                        self.goal.target_pose.pose.position.x,
                        self.goal.target_pose.pose.position.y,
                        self.goal.target_pose.pose.orientation.w)

                    self.s_marker_pub.publish(self.setMarker(self.state_msg.pose.position.x, self.state_msg.pose.position.y, self.state_msg.pose.orientation.w, 0))
                    self.g_marker_pub.publish(self.setMarker(self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y, self.goal.target_pose.pose.orientation.w, 0))

                    wait = self.move_client.wait_for_result()
                    if not wait:
                        rospy.logerr("Action server not available!")
                        rospy.signal_shutdown("Action server not available!")
                    else:
                        if self.move_client.get_state() == GoalStatus.SUCCEEDED:
                            rospy.loginfo('Goal reached!!')
                            self.r_marker_pub.publish(self.setMarker(self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y, self.goal.target_pose.pose.orientation.w, 1))
                        else:
                            rospy.loginfo('Failed to reach the goal!!')
                            result_f.write("{0} {1} {2} {3} {4} {5}\n".format(
                                self.state_msg.pose.position.x,
                                self.state_msg.pose.position.y,
                                self.state_msg.pose.orientation.w,
                                self.goal.target_pose.pose.position.x,
                                self.goal.target_pose.pose.position.y,
                                self.goal.target_pose.pose.orientation.w))
                            self.r_marker_pub.publish(self.setMarker(self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y, self.goal.target_pose.pose.orientation.w, 2))
                    
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s" % e)
            else:
                rospy.logwarn("Invalid input. Please enter 'start' or 'goal'.")
                
        result_f.close()  
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('test_planner_node')
    node = testPlannerNode()
    node.main()
