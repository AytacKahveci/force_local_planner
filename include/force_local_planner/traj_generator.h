#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include <base_local_planner/trajectory_sample_generator.h>
#include <Eigen/Core>

namespace force_local_planner
{

/**
 * Generates trajectories with the assumption of robot goes constant velocity
 * during the simulation time
*/
class TrajectoryGenerator: public base_local_planner::TrajectorySampleGenerator
{
public:
  TrajectoryGenerator()
  {}

  ~TrajectoryGenerator()
  {}

  void setParameters(double sim_time, double sim_granularity)
  {
    sim_time_ = sim_time;
    sim_granularity_ = sim_granularity;
  };

  bool generateTrajectory(
    Eigen::Vector3f pos,
    Eigen::Vector3f vel,
    base_local_planner::Trajectory& traj
  );

  bool hasMoreTrajectories();

  bool nextTrajectory(base_local_planner::Trajectory &traj);

  static Eigen::Vector3f computeNewPositions(const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel, double dt);

protected:
  unsigned int next_sample_index_;

  Eigen::Vector3f pos_;
  Eigen::Vector3f vel_;

  double sim_time_, sim_granularity_;
};

} // namespace force_local_planner

#endif