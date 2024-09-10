#include <teb_local_planner/homotopy_class_planner.h>
#include <teb_local_planner/h_signature.h>

namespace teb_local_planner
{
  
  
template<typename BidirIter, typename Fun>
EquivalenceClassPtr HomotopyClassPlanner::calculateEquivalenceClass(BidirIter path_start, BidirIter path_end, Fun fun_cplx_point, const ObstContainer* obstacles,
                                                                    boost::optional<TimeDiffSequence::iterator> timediff_start, boost::optional<TimeDiffSequence::iterator> timediff_end)
{
  if(cfg_->obstacles.include_dynamic_obstacles)
  {
    HSignature3d* H = new HSignature3d(*cfg_);
    H->calculateHSignature(path_start, path_end, fun_cplx_point, obstacles, timediff_start, timediff_end);
    return EquivalenceClassPtr(H);
  }
  else
  {
    HSignature* H = new HSignature(*cfg_);
    H->calculateHSignature(path_start, path_end, fun_cplx_point, obstacles);
    return EquivalenceClassPtr(H);
  }
}


template<typename BidirIter, typename Fun>
TebOptimalPlannerPtr HomotopyClassPlanner::addAndInitNewTeb(BidirIter path_start, BidirIter path_end, Fun fun_position, double start_orientation, double goal_orientation, const geometry_msgs::Twist* start_velocity, bool free_goal_vel)
{
  TebOptimalPlannerPtr candidate = TebOptimalPlannerPtr( new TebOptimalPlanner(*cfg_, obstacles_, robot_model_));

  candidate->teb().initTrajectoryToGoal(path_start, path_end, fun_position, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta,
                                 cfg_->robot.acc_lim_x, cfg_->robot.acc_lim_theta, start_orientation, goal_orientation, cfg_->trajectory.min_samples,
                                 cfg_->trajectory.allow_init_with_backwards_motion);

  if (start_velocity)
    candidate->setVelocityStart(*start_velocity);

  EquivalenceClassPtr H = calculateEquivalenceClass(candidate->teb().poses().begin(), candidate->teb().poses().end(), getCplxFromVertexPosePtr, obstacles_,
                                                    candidate->teb().timediffs().begin(), candidate->teb().timediffs().end());

  
  if (free_goal_vel)
    candidate->setVelocityGoalFree();

  if(addEquivalenceClassIfNew(H))
  {
    tebs_.push_back(candidate);
    return tebs_.back();
  }

  // If the candidate constitutes no new equivalence class, return a null pointer
  return TebOptimalPlannerPtr();
}
  
} // namespace teb_local_planner