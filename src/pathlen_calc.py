#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math
import matplotlib.pyplot as plt

class PathLengthLogger:
    def __init__(self):
        self.prev_pos = None
        self.total_distance = 0.0
        self.active = False

        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def start(self):
        self.prev_pos = None
        self.total_distance = 0.0
        self.active = True

    def stop(self):
        self.active = False
        return self.total_distance

    def odom_callback(self, msg):
        if not self.active:
            return

        pos = msg.pose.pose.position
        current_pos = (pos.x, pos.y)

        if self.prev_pos is not None:
            dx = current_pos[0] - self.prev_pos[0]
            dy = current_pos[1] - self.prev_pos[1]
            dist = math.sqrt(dx ** 2 + dy ** 2)
            self.total_distance += dist

        self.prev_pos = current_pos

class PathVisualizer(PathLengthLogger):
    def __init__(self, goal_index=0):
        super().__init__()
        self.path_points = []
        self.goal_index = goal_index

    def set_goal_index(self, idx):
        self.goal_index = idx

    def start(self):
        super().start()
        self.path_points = []

    def stop(self):
        distance = super().stop()
        self.save_path_plot()
        return distance

    def odom_callback(self, msg):
        super().odom_callback(msg)
        if self.active:
            pos = msg.pose.pose.position
            self.path_points.append((pos.x, pos.y))

    def save_path_plot(self):
        if not self.path_points:
            rospy.logwarn("No path points to plot.")
            return

        xs, ys = zip(*self.path_points)
        plt.figure()
        plt.plot(xs, ys, marker='o', markersize=2, linestyle='-')
        plt.title(f'Robot Path to Goal {self.goal_index + 1}')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.axis('equal')
        plt.legend()
        plt.grid(True)

        filename = f'path_{self.goal_index + 1}.png'
        plt.savefig(filename)
        plt.close()
        rospy.loginfo(f"Saved path visualization as {filename}")