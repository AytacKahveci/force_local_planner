#ifndef FORCE_LOCAL_PLANNER_H_
#define FORCE_LOCAL_PLANNER_H_

#include <stdio.h>
#include <angles/angles.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

#include <costmap_2d/costmap_2d_ros.h>
#include "force_local_planner/laser_model.h"
#include "force_local_planner/diffdrive_model.h"
#include "force_local_planner/traj_generator.h"

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>


namespace force_local_planner
{

/**
 * @brief Local planner implementation
 * 
 * In each control cycle, attractive and repulsive forces are computed.
 * When the resultant force is applied to the dynamic model of the robot,
 * linear and angular velocites computed and send to the robot.
*/
class ForceLocalPlanner : public nav_core::BaseLocalPlanner
{

public:
  ForceLocalPlanner();

  ~ForceLocalPlanner();

  void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS* costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

  bool computeVelocityCommands(geometry_msgs::TwistStamped &cmd_vel, std::string &message);

  void saturateVelocities(double& v, double& w);

  bool isGoalReached();

  bool checkTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel);

  bool pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose,
                       std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot=1);

  bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                           const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap,
                           const std::string& global_frame, double max_plan_length, std::vector<geometry_msgs::PoseStamped>& transformed_plan,
                           int* current_goal_idx = nullptr, geometry_msgs::TransformStamped* tf_plan_to_global = nullptr);

  double distancePoints2D(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
  {
    return std::sqrt( std::pow(p2.x-p1.x,2) + std::pow(p2.y-p1.y,2) );
  }

private:
  ros::Publisher force_a_pub_; //< Attractive force publisher
  ros::Publisher force_r_pub_; //< Repulsive force publisher
  ros::Publisher force_res_pub_; //< Resultant force publisher

  ros::Publisher global_plan_pub_; // Global plan publisher
  ros::Publisher local_plan_pub_; // Local plan publisher

  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;
  tf2_ros::Buffer* tf_;
  boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;
  boost::shared_ptr<LaserModel> laser_model_;
  boost::shared_ptr<DiffdriveModel> diff_model_;

  ros::Time prev_time_;
  Eigen::Vector3d state_; //< @brief theta, theta_dot, right_wheel_angular_velocity
  Eigen::Vector3d prev_state_;
  Eigen::Vector2d u_; //< @brief F*sin(theta_f), F*cos(theta_f)
  double mass_; // Mass of the robot in kg
  double Izz_; // Inertia moment around a vertical axis z in kgm^2
  double b_; // Viscous friction coefficient
  double d_; // Wheel_seperation / 2.0
  double r_; // Wheel radius
  double h_; // Distance to the center of mass from frontal surface of the robot which resultant force is applied to
  
  boost::shared_ptr<base_local_planner::ObstacleCostFunction> obstacle_costs_;
  base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
  force_local_planner::TrajectoryGenerator generator_;
  
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<geometry_msgs::PoseStamped> local_plan_;

  base_local_planner::OdometryHelperRos odom_helper_;

  std::vector<geometry_msgs::Point> footprint_spec_;
  double robot_inscribed_radius_;
  double robot_circumscribed_radius_;

  std::string global_frame_;
  std::string robot_base_frame_;
  std::string name_;

  bool initialized_ = false;
  bool goal_reached_;
  geometry_msgs::Twist last_cmd_;
  geometry_msgs::Twist robot_vel_;
  double max_linear_vel_;
  double min_linear_vel_;
  double max_angular_vel_;
  double min_angular_vel_;
  double global_plan_prune_distance_ = 1.0;
  double max_global_plan_lookahead_dist_ = 8.0;
  double xy_goal_tolerance_ = 0.5;
  double yaw_tolerance_ = 0.2;
  double k1_ = 1.0; // Attractive force coefficient
  double k1_close_ = 1.0; // Attractive force coefficient used when it is close to goal
  double k2_ = 0.5; // Repulsive force coefficient
  double dmax_ = 1.0; // Max distance to obstacle for creating repulsive forces;
  double sim_time_ = 3.0;
  double sim_granularity_ = 0.1;
};

} // namespace force_local_planner

#endif