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
        position=Point(x=0.035176217555999756, y=0.0025000572204589844, z=0.004619598388671875),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )

    # 이동할 goal 목록
    goals = [
        Pose(position=Point(-5.376387596130371, -1.9829909801483154, 0.0025796890258789062), orientation=Quaternion(0, 0, 0, 1)),
        Pose(position=Point(-7.038274765014648, -6.100484848022461, 0.004405975341796875), orientation=Quaternion(0, 0, 0, 1)),
        Pose(position=Point(2.0859322547912598, -5.926054954528809, 0.004673004150390625), orientation=Quaternion(0, 0, 0, 1)),
        Pose(position=Point(2.153135299682617, -1.359513759613037, 0.0032796859741210938), orientation=Quaternion(0, 0, 0, 1)),
        Pose(position=Point(-3.689016342163086, -1.930342197418213, 0.00272369384765625), orientation=Quaternion(0, 0, 0, 1)),
        Pose(position=Point(3.6924867630004883, -5.551314830780029, 0.005063056945800781), orientation=Quaternion(0, 0, 0, 1)),
        Pose(position=Point(5.42404842376709, -1.186535358428955, 0.0034856796264648438), orientation=Quaternion(0, 0, 0, 1)),
        Pose(position=Point(5.483254432678223, -5.260541915893555, 0.0053730010986328125), orientation=Quaternion(0, 0, 0, 1)),
        Pose(position=Point(-5.36610221862793, -3.011920690536499, 0.0015506744384765625), orientation=Quaternion(0, 0, 0, 1)),
        Pose(position=Point(-0.5889233350753784, -6.269580841064453, 0.004302024841308594), orientation=Quaternion(0, 0, 0, 1)),
    ]

    rate = rospy.Rate(10)  # 1 Hz 대기

    for idx, goal_pose in enumerate(goals):
        rospy.loginfo("Resetting robot to start pose...")
        reset_robot_pose(start_pose)
        rospy.sleep(1.0)

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

        rospy.sleep(1.0)
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

