#include "force_local_planner/traj_generator.h"

namespace force_local_planner
{

/**
 * Whether this generator can create more trajectories
 */
bool TrajectoryGenerator::hasMoreTrajectories() 
{
  return false;
}

/**
 * Create and return the next sample trajectory
 */
bool TrajectoryGenerator::nextTrajectory(base_local_planner::Trajectory &comp_traj) {
  return false;
}

/**
 * @param pos current position of robot
 * @param vel desired velocity for sampling
 */
bool TrajectoryGenerator::generateTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel,
                                             base_local_planner::Trajectory& traj)
{
  traj.cost_ = -1;
  traj.resetPoints();

  int num_steps = ceil(sim_time_ / sim_granularity_);
  double dt = sim_time_ / num_steps;
  traj.time_delta_ = dt;

  for(int i = 0; i < num_steps; ++i)
  {
    traj.addPoint(pos[0], pos[1], pos[2]);

    pos = computeNewPositions(pos, vel, dt);
  }

  return true;
}


Eigen::Vector3f TrajectoryGenerator::computeNewPositions(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, double dt) 
{
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}

} // namespace force_local_planner
