#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import time 

from pathlen_calc import PathVisualizer


def reset_robot_pose(start_pose):
    rospy.wait_for_service("/gazebo/set_model_state")
    try:
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        state_msg = ModelState()
        state_msg.model_name = "former" 
        state_msg.pose = start_pose
        state_msg.reference_frame = "world"
        result = set_state(state_msg)
        rospy.loginfo("Robot reset to start pose: %s", result.success)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def goto_goal(client, pose):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # 사용 중인 좌표계에 맞게 변경
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = pose

    client.send_goal(goal)
    rospy.loginfo("Goal sent: position=(%.3f, %.3f)", pose.position.x, pose.position.y)

    # 목표 도착 여부를 기다림 (타임아웃 필요시 설정 가능)
    client.wait_for_result()
    return client.get_state()

def main():
    rospy.init_node("sequential_goal_sender")

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server.")

    path_logger = PathVisualizer()

    # 로봇이 시작할 기본 pose 정의
    start_pose = Pose(
        position=Point(x=-0.001371830701828003, y=-0.43494153022766113, z=0.0040073394775390625),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )

    # 이동할 goal 목록
    goals = [
        #Pose(position=Point(2.0118680000305176, -1.1605618000030518, 0.0008535385131835938), orientation=Quaternion(0, 0, 0, 1)),
        #Pose(position=Point(3.6787428855895996, -4.710432052612305, 0.0008535385131835938), orientation=Quaternion(0, 0, 0, 1)),
        #Pose(position=Point(4.22443151473999, -6.129032135009766, 0.004008293151855469), orientation=Quaternion(0, 0, 0, 1)),
        #Pose(position=Point(7.020925521850586, -4.3737311363220215, 0.0008535385131835938), orientation=Quaternion(0, 0, 0, 1)),
        #Pose(position=Point(6.056573390960693, -1.581486463546753, 0.0013828277587890625), orientation=Quaternion(0, 0, 0, 1)),
        Pose(position=Point(-0.34913772344589233, -6.272071838378906, 0.0008535385131835938), orientation=Quaternion(0, 0, 0, 1)),
        Pose(position=Point(-7.090811729431152, -5.745904922485352, 0.0023775100708007812), orientation=Quaternion(0, 0, 0, 1)),
        Pose(position=Point(-6.302387714385986, -0.7282567024230957, 0.0008535385131835938), orientation=Quaternion(0, 0, 0, 1)),
        Pose(position=Point(-3.7159152030944824, -2.111722469329834, 0.002437591552734375), orientation=Quaternion(0, 0, 0, 1)),
        Pose(position=Point(-3.0558886528015137, -5.786387920379639, 0.00397491455078125), orientation=Quaternion(0, 0, 0, 1)),
        Pose(position=Point(-0.7093909978866577, -1.9743623733520508, 0.00397491455078125), orientation=Quaternion(0, 0, 0, 1))
    ]

    rate = rospy.Rate(10)  # 1 Hz 대기
    rospy.loginfo("Resetting robot to start pose...")
    #reset_robot_pose(start_pose)
    
    for idx, goal_pose in enumerate(goals):
        rospy.sleep(10.0)

        rospy.loginfo("Sending goal %d", idx + 1)

        path_logger.set_goal_index(idx)
        path_logger.start()
        start_time = time.time()

        state = goto_goal(client, goal_pose)

        travel_time = time.time() - start_time
        distance = path_logger.stop()

        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached goal %d successfully.", idx + 1)
            rospy.loginfo("Path length: %.2f meters", distance)
            rospy.loginfo("Travel time: %.2f seconds", travel_time)

            with open("path_lengths.txt", "a") as f:
                f.write("Goal %d: %.2f meters, %.2f seconds\n" % (idx + 1, distance, travel_time))
        else:
            rospy.logwarn("Failed to reach goal %d. State: %d", idx + 1, state)

        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

