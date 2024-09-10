#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // Include for tf2 conversion functions
#include <cmath>
#include <limits>

class ClosestObstacleVisualizer {
public:
    ClosestObstacleVisualizer() : nh_("~"), tf_buffer_(new tf2_ros::Buffer()), tf_listener_(*tf_buffer_) {
        odom_sub_ = nh_.subscribe("/odom", 1, &ClosestObstacleVisualizer::odomCallback, this);
        costmap_sub_ = nh_.subscribe("/move_base/local_costmap/costmap", 1, &ClosestObstacleVisualizer::costmapCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("closest_obstacle_line_marker", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber costmap_sub_;
    geometry_msgs::Point robot_position_;
    geometry_msgs::Point left_closest_obstacle_;
    geometry_msgs::Point right_closest_obstacle_;
    nav_msgs::OccupancyGrid::ConstPtr occupancy_grid_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void visualizeClosestObstacleLine(const geometry_msgs::Point& robot_position, const geometry_msgs::Point& closest_obstacle_point)
    {
        visualization_msgs::Marker line;
        line.header.frame_id = "map"; // Assuming the frame_id is "map"
        line.header.stamp = ros::Time::now();
        line.ns = "closest_obstacle_line";
        line.action = visualization_msgs::Marker::ADD;
        line.id = 0;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.01; // Line width
        line.color.g = 1.0; // Green color
        line.color.a = 1.0; // Fully opaque

        line.points.push_back(robot_position);
        line.points.push_back(closest_obstacle_point);

        marker_pub_.publish(line);
    }

    void visualizePoint(const geometry_msgs::Point& point, const std::string& ns, int id, const std_msgs::ColorRGBA& color)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map"; // Assuming the frame_id is "map"
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = point;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1; // Marker size
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color = color; // Color of the marker

        marker_pub_.publish(marker);
    }

    // Function to visualize closest obstacles on left and right sides
    void visualizeClosestObstacles()
    {
        // Visualize the closest obstacle on the left side
        visualizePoint(left_closest_obstacle_, "closest_obstacles", 0, createColorRGBA(0, 1, 0, 1)); // Green color
        
        // Visualize the closest obstacle on the right side
        visualizePoint(right_closest_obstacle_, "closest_obstacles", 1, createColorRGBA(0, 1, 0, 1)); // Green color
    }

    // Function to create a ColorRGBA message
    std_msgs::ColorRGBA createColorRGBA(float r, float g, float b, float a)
    {
        std_msgs::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }

    void findClosestObstacle()
    {
        double min_left_distance = std::numeric_limits<double>::max();
        double min_right_distance = std::numeric_limits<double>::max();
    
        // Define the search range around the robot (5.0 meters)
        double search_range = 5.0;

        // Calculate the indices of the cells around the robot within the search range
        int min_index_x = std::max(0, static_cast<int>((robot_position_.x - occupancy_grid_->info.origin.position.x - search_range) / occupancy_grid_->info.resolution));
        int max_index_x = std::min(static_cast<int>(occupancy_grid_->info.width), static_cast<int>((robot_position_.x - occupancy_grid_->info.origin.position.x + search_range) / occupancy_grid_->info.resolution));
        int min_index_y = std::max(0, static_cast<int>((robot_position_.y - occupancy_grid_->info.origin.position.y - search_range) / occupancy_grid_->info.resolution));
        int max_index_y = std::min(static_cast<int>(occupancy_grid_->info.height), static_cast<int>((robot_position_.y - occupancy_grid_->info.origin.position.y + search_range) / occupancy_grid_->info.resolution));

        // Get the robot's orientation
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_->lookupTransform("map", "base_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            return;
        }
        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w);
        tf2Scalar roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
        // Robot's direction vector
        geometry_msgs::Vector3 robot_direction;
        robot_direction.x = cos(yaw); // x component of the direction vector
        robot_direction.y = sin(yaw); // y component of the direction vector
    
        // Iterate over the cells within the search range
        for (int i = min_index_x; i < max_index_x; ++i)
        {
            for (int j = min_index_y; j < max_index_y; ++j)
            {
                int index = i + j * occupancy_grid_->info.width;

                if (occupancy_grid_->data[index] > 0) // Occupied cell
                {
                    double obstacle_x = occupancy_grid_->info.origin.position.x + i * occupancy_grid_->info.resolution;
                    double obstacle_y = occupancy_grid_->info.origin.position.y + j * occupancy_grid_->info.resolution;
                    double distance = sqrt(pow(robot_position_.x - obstacle_x, 2) + pow(robot_position_.y - obstacle_y, 2));
                
                    // Calculate the vector from robot to obstacle
                    geometry_msgs::Vector3 robot_to_obstacle;
                    robot_to_obstacle.x = obstacle_x - robot_position_.x;
                    robot_to_obstacle.y = obstacle_y - robot_position_.y;
                
                    // Determine the cross product between robot's direction vector and robot-to-obstacle vector
                    double cross_product = robot_direction.x * robot_to_obstacle.y - robot_direction.y * robot_to_obstacle.x;
                    if (cross_product > 0) // Left side
                    {
                        if (distance < min_left_distance)
                        {
                            min_left_distance = distance;
                            left_closest_obstacle_.x = obstacle_x;
                            left_closest_obstacle_.y = obstacle_y;
                            left_closest_obstacle_.z = 0.0;
                        }
                    }
                    else // Right side
                    {
                        if (distance < min_right_distance)
                        {
                            min_right_distance = distance;
                            right_closest_obstacle_.x = obstacle_x;
                            right_closest_obstacle_.y = obstacle_y;
                            right_closest_obstacle_.z = 0.0;
                        }
                    }
                }
            }
        }
    
    // Print the distances and positions of the closest obstacles on the left and right sides
    ROS_INFO("Closest obstacle distance on the left: %.2f, position: (%.2f, %.2f)", min_left_distance, left_closest_obstacle_.x, left_closest_obstacle_.y);
    ROS_INFO("Closest obstacle distance on the right: %.2f, position: (%.2f, %.2f)\n", min_right_distance, right_closest_obstacle_.x, right_closest_obstacle_.y);

    // Visualize the closest obstacle on the left and right sides
    visualizeClosestObstacleLine(robot_position_, left_closest_obstacle_);
    visualizeClosestObstacleLine(robot_position_, right_closest_obstacle_);

    visualizeClosestObstacles();
    }

    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        // Debugging message to confirm callback invocation
        // ROS_INFO("costmapCallback called.");

        // Store the occupancy grid message
        occupancy_grid_ = msg;
        
        // Find the closest obstacles on the left and right sides
        findClosestObstacle();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // Debugging message to confirm callback invocation
        // ROS_INFO("odomCallback called.");
        
        robot_position_ = msg->pose.pose.position;
        // Call costmapCallback to update closest obstacle distances
        if (occupancy_grid_)
            findClosestObstacle();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "closest_obstacle_visualizer");
    ClosestObstacleVisualizer visualizer;
    ros::spin();
    return 0;
}
