#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft

def move_to_goal(x, y, yaw):
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    
    # 목표 위치 설정
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = x
    goal.pose.position.y = y
    
    # 회전 각도(yaw)를 quaternion으로 변환
    quaternion = tft.quaternion_from_euler(0, 0, yaw)
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]

    # 퍼블리셔에 구독자가 있을 때까지 기다린 후 목표 좌표를 퍼블리시
    while not rospy.is_shutdown():
        connections = goal_publisher.get_num_connections()
        if connections > 0:
            goal_publisher.publish(goal)
            rospy.loginfo("Goal published!")
            return True
        rospy.sleep(0.1)  # 잠깐 대기 후 다시 시도

    return False

if __name__ == '__main__':
    rospy.init_node('move_base_goal_sender', anonymous=True)

    start_coords = (-0.832, 4.780, 0.0)  # 시작 좌표 (x, y, yaw)
    goal_coords = (-3.930, 4.148, 0.0)   # 목표 좌표 (x, y, yaw)

    try:
        while not rospy.is_shutdown():
            user_input = input("Enter 'initial' to move the robot to the initial position: ").strip().lower()

            if user_input == "initial":
                rospy.loginfo("Moving the robot to the initial position...")
                reached_start = move_to_goal(*start_coords)  # 시작 위치로 이동
                if reached_start:
                    rospy.sleep(2)  # 시작 위치에서 2초 대기
                    reached_goal = move_to_goal(*goal_coords)  # 목표 위치로 이동

                    if reached_goal:
                        rospy.loginfo("The robot has reached the goal: {}".format(goal_coords))
                        rospy.loginfo("Process completed. Awaiting further instructions.")
                    else:
                        rospy.logwarn("Failed to reach the goal. Please check the system.")
                else:
                    rospy.logwarn("Failed to reach the initial position. Please check the system.")
            else:
                rospy.loginfo("Unknown command. Please enter 'initial' to move the robot.")
    
    except rospy.ROSInterruptException:
        pass
