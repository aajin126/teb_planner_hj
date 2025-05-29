#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import os

from std_msgs.msg import Int32

class PathVisualizer:
    def __init__(self, save_dir = "log"):
        rospy.init_node('path_visualizer', anonymous=True)
        self.current_goal_id = 0
        rospy.Subscriber("/current_goal_id", Int32, self.goal_callback)
        self.save_dir = save_dir
        self.path = []
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def goal_callback(self, msg):
        if self.path:  # 이전 goal 경로 저장
            self.save_path_plot(goal_id=self.current_goal_id)
            self.path.clear()  # 다음 경로 저장을 위해 초기화
        self.current_goal_id = msg.data

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.path.append((pos.x, pos.y))

    def save_path_plot(self, goal_id=None):
        if not self.path:
            rospy.logwarn("No path recorded.")
            return

        x_vals, y_vals = zip(*self.path)
        plt.figure(figsize=(8, 6))
        plt.plot(x_vals, y_vals, marker='o', linestyle='-')
        plt.title(f"Robot Path to Goal {goal_id}")
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.grid(True)
        plt.axis('equal')

        directory = os.path.dirname(self.save_dir)
        if directory and not os.path.exists(directory):
            os.makedirs(directory)

        filename = f"goal_{goal_id:02d}_path.png" if goal_id is not None else "robot_path.png"
        filepath = os.path.join(directory, filename)
        plt.savefig(filepath)
        plt.close()
        rospy.loginfo(f"Path saved to {filepath}")

if __name__ == '__main__':
    try:
        visualizer = PathVisualizer(save_dir = "log")
        rospy.loginfo("Recording robot path... Press Ctrl+C to stop.")
        rospy.spin()
        visualizer.save_path_plot()
    except rospy.ROSInterruptException:
        pass
