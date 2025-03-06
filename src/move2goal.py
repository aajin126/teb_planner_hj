#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus

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

    # move_base 액션 클라이언트 초기화
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server.")

    # 주어진 goal pose들을 리스트에 정의 (순서대로)
    goals = [
        #Pose(Point(2.075901508331299, -0.9431991577148438, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        Pose(Point(2.116427421569824, -1.095625877380371, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        # Pose(Point(2.916426658630371, -2.345625877380371, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-0.0835728645324707, -0.017227649688720703, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(1.240919589996338, -2.427267551422119, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-1.3590803146362305, -3.077267646789551, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-1.5740985870361328, -3.6431994438171387, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-1.5835728645324707, -3.6672279834747314, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-1.5835728645324707, -3.6456260681152344, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-3.1590805053710938, -2.7772674560546875, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-3.3090806007385254, -2.627267837524414, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-3.509080410003662, -1.9272675514221191, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-4.25908088684082, -2.327267646789551, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-3.009080410003662, -1.5272674560546875, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-3.509080410003662, -0.9772677421569824, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-3.4835729598999023, -0.9456257820129395, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-3.509080410003662, -0.9772677421569824, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-5.159080505371094, -2.327267646789551, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-4.965807914733887, -3.326188564300537, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-5.322455406188965, -3.22623872756958, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-5.415807723999023, -4.576188564300537, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-5.409080505371094, -4.577267646789551, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-4.922454833984375, -2.8262386322021484, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-5.365808010101318, -2.576188564300537, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-5.37245512008667, -2.5762386322021484, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-5.672454833984375, -5.626238822937012, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-5.172454833984375, -5.72623872756958, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        # Pose(Point(-4.909080505371094, -5.877267837524414, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
    ]

    rate = rospy.Rate(30)  # 1 Hz, 각 goal 사이에 잠시 대기

    for idx, pose in enumerate(goals):
        rospy.loginfo("Sending goal %d", idx + 1)
        state = goto_goal(client, pose)

        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached goal %d successfully.", idx + 1)
        else:
            rospy.logwarn("Failed to reach goal %d. State: %d", idx + 1, state)

        # 다음 goal 전 잠시 대기 (원하는 시간으로 조절)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
