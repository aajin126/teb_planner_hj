#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

class PathVisualizer {
public:
    PathVisualizer() {
        ros::NodeHandle nh;
        path_sub = nh.subscribe("move_base/NavfnROS/plan", 10, &PathVisualizer::pathCallback, this);
        marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& path) {
        visualization_msgs::Marker points;
        points.header.frame_id = "map";
        points.header.stamp = ros::Time::now();
        points.ns = "path_visualizer";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.type = visualization_msgs::Marker::LINE_STRIP;
        points.scale.x = 0.05;
        points.color.g = 1.0f;
        points.color.a = 1.0;

        for (const auto& pose : path->poses) {
            points.points.push_back(pose.pose.position);
        }

        marker_pub.publish(points);
    }

private:
    ros::Publisher marker_pub;
    ros::Subscriber path_sub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_visualizer");
    PathVisualizer visualizer;
    ros::spin();
    return 0;
}
