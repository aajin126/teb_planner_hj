#!/usr/bin/env python

import rospy
import random

from std_msgs.msg import String, Int32
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from actionlib_msgs.msg import *

from visualization_msgs.msg import Marker

class testPlannerNode:
    def __init__(self):
        print('Hello test planer node')
        #rospy.Subscriber("/gazebo/model_states", Int32, self.messageCallback, queue_size=10)
        #self.set_state = rospy.Publisher("gazebo/set_model_state", Modelstate, queue_size=10)
        self.s_marker_pub = rospy.Publisher("/t_s_marker", Marker, queue_size = 2)
        self.g_marker_pub = rospy.Publisher("/t_g_marker", Marker, queue_size = 2)
        self.r_marker_pub = rospy.Publisher("/t_r_marker", Marker, queue_size = 2)
        
        self.move_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.move_client.wait_for_server()
        
        self.state_msg = ModelState()
        self.state_msg.model_name = 'former'
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        
        self.reached=0
    
    def setMarker(self, x, y, w, mode):
      marker = Marker()

      marker.header.frame_id = "/map"
      marker.header.stamp = rospy.Time.now()

      # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
      marker.type = 2
      marker.id = 0

      # Set the scale of the marker
      marker.scale.x = 0.2
      marker.scale.y = 0.2
      marker.scale.z = 0.2

      # Set the color
      if mode==1:
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

      # Set the pose of the marker
      marker.pose.position.x = x
      marker.pose.position.y = y
      marker.pose.position.z = 0
      marker.pose.orientation.x = 0.0
      marker.pose.orientation.y = 0.0
      marker.pose.orientation.z = 0.0
      marker.pose.orientation.w = w
      
      return marker
    
    def resetPose(self, str_set):
      rospy.loginfo('Reset position!')
      poses = str_set.split()
      (s_x, s_y, s_w) = float(poses[0]), float(poses[1]), float(poses[2])
      (g_x, g_y, g_w) = float(poses[3]), float(poses[4]), float(poses[5]) 
      
      self.state_msg.pose.position.x = s_x
      self.state_msg.pose.position.y = s_y
      self.state_msg.pose.orientation.w = s_w
      
      self.goal.target_pose.header.stamp = rospy.Time.now()
      self.goal.target_pose.pose.position.x = g_x
      self.goal.target_pose.pose.position.y = g_y
      self.goal.target_pose.pose.orientation.w = g_w
    
    def main(self):
      try:
        result_f = open("b_fail_list.txt","r")
        rospy.loginfo('file is opended!!')
      except:
        rospy.logerr("fail to open file..")
        return False
      
      while(1):
        rospy.wait_for_service('/gazebo/set_model_state')
        str_sg = result_f.readline()
        
        if not str_sg:
          break
        
        self.resetPose(str_sg)
        
        try:
          set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
          resp = set_state( self.state_msg )
          
          self.move_client.send_goal(self.goal)
          rospy.loginfo('send goal: from %f %f %f to %f %f %f', self.state_msg.pose.position.x, self.state_msg.pose.position.y, self.state_msg.pose.orientation.w, self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y, self.goal.target_pose.pose.orientation.w)
          
          self.s_marker_pub.publish(self.setMarker(self.state_msg.pose.position.x, self.state_msg.pose.position.y, self.state_msg.pose.orientation.w,0))
          self.g_marker_pub.publish(self.setMarker(self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y, self.goal.target_pose.pose.orientation.w,0))
          
          wait = self.move_client.wait_for_result()
          if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
          else:
            if self.move_client.get_state() == GoalStatus.SUCCEEDED:
              rospy.loginfo('Goal reached!!')
              self.r_marker_pub.publish(self.setMarker(self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y, self.goal.target_pose.pose.orientation.w,1))
            else:
              rospy.loginfo('failed again!!')
              self.r_marker_pub.publish(self.setMarker(self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y, self.goal.target_pose.pose.orientation.w,2))
              
        except rospy.ServiceException as e:
          print("Service call failed: %s" % e)
      result_f.close()  
      rospy.spin()

if __name__ == '__main__':
    rospy.init_node('test_planner_node')
    node = testPlannerNode()
    node.main()