#ifndef TEB_VISUALIZATION_H
#define TEB_VISUALIZATION_H

// teb stuff
#include "../inc/teb_config.h"
#include "../inc/timed_elastic_band.h"
#include "../inc/robot_footprint_model.h"

// boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

// std
#include <iterator>

// ros stuff
#include <ros/publisher.h>
#include <base_local_planner/goal_functions.h>

// messages
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

namespace teb_local_planner
{

    class TebOptimalPlanner; //!< Forward Declaration

/**
 * @class TebVisualization
 * @brief Visualize stuff from the teb_local_planner
 */
    class TebVisualization
    {
    public:

        /**
         * @brief Default constructor
         * @remarks do not forget to call initialize()
         */
        TebVisualization();

        /**
         * @brief Constructor that initializes the class and registers topics
         * @param nh local ros::NodeHandle
         * @param cfg const reference to the TebConfig class for parameters
         */
        TebVisualization(const TebConfig& cfg);

        /**
         * @brief Initializes the class and registers topics.
         *
         * Call this function if only the default constructor has been called before.
         * @param nh local ros::NodeHandle
         * @param cfg const reference to the TebConfig class for parameters
         */
        void initialize(const TebConfig& cfg);


        /** @name Publish to topics */
        //@{

        /**
         * @brief Publish a given global plan to the ros topic \e ../../global_plan
         * @param global_plan Pose array describing the global plan
         */
        void publishGlobalPlan(const std::vector<PoseStamped>& global_plan) const;

        /**
         * @brief Publish a given local plan to the ros topic \e ../../local_plan
         * @param local_plan Pose array describing the local plan
         */
        void publishLocalPlan(const std::vector<PoseStamped>& local_plan) const;

        /**
         * @brief Publish Timed_Elastic_Band related stuff (local plan, pose sequence).
         *
         * Given a Timed_Elastic_Band instance, publish the local plan to  \e ../../local_plan
         * and the pose sequence to  \e ../../teb_poses.
         * @param teb const reference to a Timed_Elastic_Band
         */
        void publishLocalPlanAndPoses(const TimedElasticBand& teb) const;

        /**
         * @brief Publish the robot footprints related to infeasible poses
         *
         * @param current_pose Current pose of the robot
         * @param robot_model Subclass of BaseRobotFootprintModel
         */
        void publishInfeasibleRobotPose(const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model);

        /**
         * @brief Publish obstacle positions to the ros topic \e ../../teb_markers
         * @todo Move filling of the marker message to polygon class in order to avoid checking types.
         * @param obstacles Obstacle container
         */
        void publishObstacles(const ObstContainer& obstacles) const;

        /**
         * @brief Publish via-points to the ros topic \e ../../teb_markers
         * @param via_points via-point container
         */
        void publishViaPoints(const std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& via_points, const std::string& ns = "ViaPoints") const;

        /**
         * @brief Publish multiple Tebs from a container class (publish as marker message).
         *
         * @param teb_planner Container of boost::shared_ptr< TebOptPlannerPtr >
         * @param ns Namespace for the marker objects
         */
        void publishTebContainer(const std::vector< boost::shared_ptr<TebOptimalPlanner> >& teb_planner, const std::string& ns = "TebContainer");

        /**
         * @brief Publish a feedback message (multiple trajectory version)
         *
         * The feedback message contains the all planned trajectory candidates (e.g. if planning in distinctive topologies is turned on).
         * Each trajectory is composed of the sequence of poses, the velocity profile and temporal information.
         * The feedback message also contains a list of active obstacles.
         * @param teb_planners container with multiple tebs (resp. their planner instances)
         * @param selected_trajectory_idx Idx of the currently selected trajectory in \c teb_planners
         * @param obstacles Container of obstacles
         */
        void publishFeedbackMessage(const std::vector< boost::shared_ptr<TebOptimalPlanner> >& teb_planners, unsigned int selected_trajectory_idx, const ObstContainer& obstacles);

        /**
         * @brief Publish a feedback message (single trajectory overload)
         *
         * The feedback message contains the planned trajectory
         * that is composed of the sequence of poses, the velocity profile and temporal information.
         * The feedback message also contains a list of active obstacles.
         * @param teb_planner the planning instance
         * @param obstacles Container of obstacles
         */
        void publishFeedbackMessage(const TebOptimalPlanner& teb_planner, const ObstContainer& obstacles);


    protected:

        /**
         * @brief Small helper function that checks if initialize() has been called and prints an error message if not.
         * @return \c true if not initialized, \c false if everything is ok
         */
        bool printErrorWhenNotInitialized() const;

        ros::Publisher global_plan_pub_; //!< Publisher for the global plan
        ros::Publisher local_plan_pub_; //!< Publisher for the local plan
        ros::Publisher teb_poses_pub_; //!< Publisher for the trajectory pose sequence
        ros::Publisher teb_marker_pub_; //!< Publisher for visualization markers
        ros::Publisher feedback_pub_; //!< Publisher for the feedback message for analysis and debug purposes
        
        const TebConfig* cfg_; //!< Config class that stores and manages all related parameters

        bool initialized_; //!< Keeps track about the correct initialization of this class


    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

//! Abbrev. for shared instances of the TebVisualization
    typedef boost::shared_ptr<TebVisualization> TebVisualizationPtr;
//! Abbrev. for shared instances of the TebVisualization (read-only)
    typedef boost::shared_ptr<const TebVisualization> TebVisualizationConstPtr;


} // namespace teb_local_planner

#endif //TEB_VISUALIZATION_H
