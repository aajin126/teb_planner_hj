#include "../inc/visualization.h"
#include "../inc/optimal_planner.h"
#include "teb_local_planner/FeedbackMsg.h"

namespace teb_local_planner
{

    TebVisualization::TebVisualization() : initialized_(false)
    {
    }

    TebVisualization::TebVisualization(const TebConfig& cfg) : initialized_(false)
    {
        initialize(cfg);
    }

    void TebVisualization::initialize(const TebConfig& cfg)
    {
        if (initialized_)
        ROS_WARN("TebVisualization already initialized. Reinitalizing...");
  
        // set config
        cfg_ = &cfg;
  
        initialized_ = true; 
    }

    void TebVisualization::publishGlobalPlan(const std::vector<PoseStamped>& global_plan) const
    {
    }

    void TebVisualization::publishLocalPlan(const std::vector<PoseStamped>& local_plan) const
    {
    }

    void TebVisualization::publishLocalPlanAndPoses(const TimedElasticBand& teb) const
    {
        if ( printErrorWhenNotInitialized() )
            return;
  
        // create path msg
        nav_msgs::Path teb_path;
        teb_path.header.frame_id = cfg_->map_frame;
        teb_path.header.stamp = ros::Time::now();
    
        // create pose_array (along trajectory)
        geometry_msgs::PoseArray teb_poses;
        teb_poses.header.frame_id = teb_path.header.frame_id;
        teb_poses.header.stamp = teb_path.header.stamp;
    
        // fill path msgs with teb configurations
        for (int i=0; i < teb.sizePoses(); i++)
        {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = teb_path.header.frame_id;
        pose.header.stamp = teb_path.header.stamp;
        pose.pose.position.x = teb.Pose(i).x();
        pose.pose.position.y = teb.Pose(i).y();
        pose.pose.position.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*teb.getSumOfTimeDiffsUpToIdx(i);
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(teb.Pose(i).theta());
        teb_path.poses.push_back(pose);
        teb_poses.poses.push_back(pose.pose);
        }
        local_plan_pub_.publish(teb_path);
        teb_poses_pub_.publish(teb_poses);
    }

    void TebVisualization::publishInfeasibleRobotPose(const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model)
    {
        //publishRobotFootprintModel(current_pose, robot_model, "InfeasibleRobotPoses", toColorMsg(0.5, 0.8, 0.0, 0.0));
    }


    void TebVisualization::publishObstacles(const ObstContainer& obstacles) const
    {
    }


    void TebVisualization::publishViaPoints(const std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& via_points, const std::string& ns) const
    {
    }

    void TebVisualization::publishTebContainer(const TebOptPlannerContainer& teb_planner, const std::string& ns)
    {
    }

    void TebVisualization::publishFeedbackMessage(const std::vector< boost::shared_ptr<TebOptimalPlanner> >& teb_planners,
                                                  unsigned int selected_trajectory_idx, const ObstContainer& obstacles)
    {
    }

    void TebVisualization::publishFeedbackMessage(const TebOptimalPlanner& teb_planner, const ObstContainer& obstacles)
    {
    }

    bool TebVisualization::printErrorWhenNotInitialized() const
    {
    }

} // namespace teb_local_planner