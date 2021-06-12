// pluginlib macros
#include <pluginlib/class_list_macros.h>
#include "force_local_planner/force_local_planner.h"

PLUGINLIB_EXPORT_CLASS(force_local_planner::ForceLocalPlanner, nav_core::BaseLocalPlanner)

namespace force_local_planner
{

ForceLocalPlanner::ForceLocalPlanner() :
  goal_reached_(false), initialized_(false)
{
}


ForceLocalPlanner::~ForceLocalPlanner()
{
}


void ForceLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if(!initialized_)
  {
    name_ = name;
    ros::NodeHandle nh("~/" + name);

    if(!nh.getParam("mass", mass_))
      throw std::runtime_error("mass parameter could not found in param server");
    if(!nh.getParam("Izz", Izz_))
      throw std::runtime_error("Izz parameter could not found in param server");
    if(!nh.getParam("b", b_))
      throw std::runtime_error("b parameter could not found in param server");
    if(!nh.getParam("wheel_seperation", d_))
      throw std::runtime_error("d parameter could not found in param server");
    d_ /= 2.0;
    if(!nh.getParam("wheel_radius", r_))
      throw std::runtime_error("r parameter could not found in param server");
    if(!nh.getParam("h", h_))
      throw std::runtime_error("h parameter could not found in param server");
    std::string odom_topic;
    if(!nh.getParam("odom_topic", odom_topic))
      throw std::runtime_error("odom_topic parameter could not found in param server");

    if(!nh.getParam("max_linear_vel", max_linear_vel_))
      throw std::runtime_error("max_linear_vel parameter could not found in param server");
    max_linear_vel_ = fabs(max_linear_vel_);
    if(!nh.getParam("min_linear_vel", min_linear_vel_))
    {
      min_linear_vel_ = -1.0*fabs(max_linear_vel_);
    }
    else
      min_linear_vel_ = -1.0*fabs(min_linear_vel_);

    if(!nh.getParam("max_angular_vel", max_angular_vel_))
      throw std::runtime_error("max_angular_vel parameter could not found in param server");
    max_angular_vel_ = fabs(max_angular_vel_);
    if(!nh.getParam("min_angular_vel", min_angular_vel_))
    {
      min_angular_vel_ = -1.0*fabs(max_angular_vel_);
    }
    else
      min_angular_vel_ = -1.0*fabs(min_angular_vel_);

    nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.1);
    nh.param("yaw_tolerance", yaw_tolerance_, 0.1);
    nh.param("global_plan_prune_distance", global_plan_prune_distance_, 0.2);
    nh.param("max_global_plan_lookahead_dist", max_global_plan_lookahead_dist_, 8.0);
    nh.param("k1", k1_, 1.0);
    nh.param("k1_close", k1_close_, 1.0);
    nh.param("k2", k2_, 0.5);
    nh.param("min_obstacle_dist", dmax_, 2.0);
    nh.param("sim_time", sim_time_, 3.0);
    nh.param("sim_granularity", sim_granularity_, 0.1);
    ROS_INFO_NAMED(name_, "Odom topic: %s, mass: %f, Izz: %f, d: %f, r: %f", odom_topic.c_str(), mass_, Izz_, d_, r_);
    
    force_a_pub_ = nh.advertise<geometry_msgs::PoseArray>("/attractive_force", 1);
    force_r_pub_ = nh.advertise<geometry_msgs::PoseArray>("/repulsive_force", 1);
    force_res_pub_ = nh.advertise<geometry_msgs::PoseArray>("/resultant_force", 1);

    global_plan_pub_ = nh.advertise<nav_msgs::Path>("/global_plan", 1);
    local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);

    // Create footprint model
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_);

    // Create costmap model
    costmap_ = costmap_ros_->getCostmap();
    costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);
    laser_model_ = boost::make_shared<LaserModel>(*costmap_, 15.0);

    // Create robot dynamic model
    diff_model_.reset(new DiffdriveModel(mass_, Izz_, b_, d_, r_, h_));

    // Set traj generator params
    generator_.setParameters(sim_time_, sim_granularity_);
    obstacle_costs_.reset(new base_local_planner::ObstacleCostFunction(costmap_));
    obstacle_costs_->setParams(max_linear_vel_, 1.0, 1.0);
    obstacle_costs_->setFootprint(footprint_spec_);
    std::vector<base_local_planner::TrajectoryCostFunction*> critics;
    critics.push_back(obstacle_costs_.get());
    std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back(&generator_);

    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);

    global_frame_ = costmap_ros_->getGlobalFrameID();
    robot_base_frame_ = costmap_ros_->getBaseFrameID();
    odom_helper_.setOdomTopic(odom_topic);

    // Initial states
    prev_state_ << 0, 0, 0;
    initialized_ = true;
  }
}


bool ForceLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  // check if plugin is initialized
  if(!initialized_)
  {
    ROS_ERROR("[force_local_planner] has not been initialized, please call initialize() before using this planner");
    return false;
  }

  // store the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // reset goal_reached_ flag
  goal_reached_ = false;
  
  return true;
}


bool ForceLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  std::string message;
  geometry_msgs::TwistStamped cmd_vel_stamped;
  bool outcome = computeVelocityCommands(cmd_vel_stamped, message);
  cmd_vel = cmd_vel_stamped.twist;
  return outcome;
}


bool ForceLocalPlanner::computeVelocityCommands(geometry_msgs::TwistStamped &cmd_vel, std::string &message)
{
  if(!initialized_)
  {
    ROS_ERROR("force_local_planner has not been initialized, please call initialize() before using this planner");
    message = "force_local_planner has not been initialized";
    return false;
  }

  ros::Time now = ros::Time::now();
  double dt = (now - prev_time_).toSec();
  prev_time_ = now;
  if(dt == 0.0)
    dt = 1e-4;

  static uint32_t seq = 0;
  cmd_vel.header.seq = seq++;
  cmd_vel.header.stamp = ros::Time::now();
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
  goal_reached_ = false;

  // Get robot pose
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);

  // Get robot odometry
  nav_msgs::Odometry base_odom;
  odom_helper_.getOdom(base_odom);

  // prune global plan to cut off parts of the past (spatially before the robot)
  pruneGlobalPlan(*tf_, robot_pose, global_plan_, global_plan_prune_distance_);

  // Transform global plan to the frame of interest (w.r.t. the local costmap)
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  int goal_idx;
  geometry_msgs::TransformStamped tf_plan_to_global;
  if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, global_frame_, max_global_plan_lookahead_dist_, 
                           transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    message = "Could not transform the global plan to the frame of the controller";
    return false;
  }

  // check if global goal is reached
  geometry_msgs::PoseStamped global_goal;
  tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
  double dx = global_goal.pose.position.x - robot_pose.pose.position.x;
  double dy = global_goal.pose.position.y - robot_pose.pose.position.y;
  double dist_to_goal = std::sqrt(dx*dx+dy*dy);
  double delta_orient = angles::normalize_angle( tf2::getYaw(global_goal.pose.orientation) - tf2::getYaw(robot_pose.pose.orientation));
  if(fabs(dist_to_goal) < xy_goal_tolerance_
    && fabs(delta_orient) < yaw_tolerance_)
  {
    goal_reached_ = true;
    return true;
  }

  // Return false if the transformed global plan is empty
  if (transformed_plan.empty())
  {
    ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
    message = "Transformed plan is empty";
    return false;
  }

  // Get current goal point (last point of the transformed plan)
  geometry_msgs::PoseStamped robot_goal;
  robot_goal = transformed_plan.back();

  // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
  if (transformed_plan.size()==1) // plan only contains the goal
  {
    transformed_plan.insert(transformed_plan.begin(), geometry_msgs::PoseStamped()); // insert start (not yet initialized)
  }
  transformed_plan.front() = robot_pose; // update start


  // Calculate Repulsive forces
  std::vector<std::pair<double,double>> obstacles;
  std::vector<Eigen::Vector2d> repulsive_forces;
  laser_model_->getMinObstacles(robot_pose, obstacles);
  for(auto obstacle : obstacles)
  {
    double angle = obstacle.first;
    double min_rho = obstacle.second;

    if(min_rho < dmax_)
    {
      double F = k2_ * pow(1.0/min_rho - 1.0/dmax_, 2);
      double F_theta = M_PI + angle;
      Eigen::Vector2d Fr(F*cos(F_theta), F*sin(F_theta));
      repulsive_forces.push_back(Fr);
    }
  }

  // Calculate Attractive force
  double F_a;
  double theta_a;
  if(fabs(dist_to_goal) < 2.0)
    F_a = k1_close_;
  else
    F_a = k1_;
    
  double theta = tf::getYaw(robot_pose.pose.orientation);
  if(fabs(std::sqrt(dx*dx+dy*dy)) < xy_goal_tolerance_)
  {
    if(tf::getYaw(robot_goal.pose.orientation) >= 0.0)
      theta_a = M_PI_2;
    else
      theta_a = -M_PI_2;
  }
  else
    theta_a = atan2(robot_goal.pose.position.y - robot_pose.pose.position.y, robot_goal.pose.position.x - robot_pose.pose.position.x) - theta;

  Eigen::Vector2d Fa(F_a*cos(theta_a), F_a*sin(theta_a));
  
  // Calculate Resultant force
  Eigen::Vector2d Fr = Fa;
  for(auto f : repulsive_forces)
  {
    Fr += f;
  }

  u_ << Fr(1), Fr(0);

  double theta_dot = base_odom.twist.twist.angular.z;
  diff_model_->dynamics(prev_state_, u_, state_);
  double w_right_wheel = state_(2);

  double w_left_wheel = w_right_wheel - 2.0*state_(1)*d_/r_;
  double v = (w_right_wheel+w_left_wheel)*r_ / 2.0;
  double w = state_(1);

  saturateVelocities(v, w);

  Eigen::Vector3f current_pose(robot_pose.pose.position.x, robot_pose.pose.position.y, theta);
  Eigen::Vector3f vel(v, 0, w);
  bool feasible = true;
  if(!checkTrajectory(current_pose, vel))
  {
    v = 0.0;
    w = 0.0;
    feasible = false;
  }

  cmd_vel.twist.linear.x = v;
  cmd_vel.twist.angular.z = w;

  ROS_DEBUG("Theta: %f, Theta_dot: %f", theta, theta_dot);
  ROS_DEBUG("V: %f, W: %f", v, w);
  ROS_DEBUG("Prev State: %f - %f - %f", prev_state_(0), prev_state_(1), prev_state_(2));
  ROS_DEBUG("State: %f - %f - %f", state_(0), state_(1), state_(2));
  prev_state_ = state_;

  // Publish visualization msgs
  geometry_msgs::PoseArray forces;
  forces.header.frame_id = robot_base_frame_;
  forces.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped force_pose;
  force_pose.header.frame_id = robot_base_frame_;
  force_pose.header.stamp = ros::Time::now();

  // Publish Attractive force
  force_pose.pose.position.x = h_;
  force_pose.pose.position.y = 0;
  force_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_a);
  forces.poses.push_back(force_pose.pose);
  force_a_pub_.publish(forces);

  // Publish Repulsive forces
  forces.poses.clear();
  for(auto p : repulsive_forces)
  {
    force_pose.header.stamp = ros::Time::now();
    force_pose.pose.position.x = h_;
    force_pose.pose.position.y = 0;
    auto F = p / p.norm();
    double Fr_theta = atan2(F(1), F(0));
    force_pose.pose.orientation = tf::createQuaternionMsgFromYaw(Fr_theta);
    forces.poses.push_back(force_pose.pose);
  }
  force_r_pub_.publish(forces);

  // Publish Resultant forces
  forces.poses.clear();
  {
    force_pose.header.stamp = ros::Time::now();
    force_pose.pose.position.x = h_;
    force_pose.pose.position.y = 0;
    auto F = Fr / Fr.norm();
    double F_theta = atan2(F(1), F(0));
    force_pose.pose.orientation = tf::createQuaternionMsgFromYaw(F_theta);
    forces.poses.push_back(force_pose.pose);
  }
  force_res_pub_.publish(forces);

  // Publish Global plan
  nav_msgs::Path global_path;
  global_path.header.frame_id = global_frame_;
  global_path.header.stamp = ros::Time::now();
  for(auto p : global_plan_)
  {
    global_path.poses.push_back(p);
  }
  base_local_planner::publishPlan(global_plan_, global_plan_pub_);

  return feasible;
}


void ForceLocalPlanner::saturateVelocities(double& v, double& w)
{
  if(v > max_linear_vel_)
    v = max_linear_vel_;
  else if(v < min_linear_vel_)
    v = min_linear_vel_;

  if(w > max_angular_vel_)
    w = max_angular_vel_;
  else if(w < min_angular_vel_)
    w = min_angular_vel_;
}


bool ForceLocalPlanner::isGoalReached()
{
  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    return true;
  }
  return false;
}


bool ForceLocalPlanner::checkTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel)
{
  base_local_planner::Trajectory traj;
  generator_.generateTrajectory(pos, vel, traj);
  double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
  //if the trajectory is a legal one... the check passes
  if(cost >= 0) {
    return true;
  }
  ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel[0], vel[1], vel[2], cost);

  //otherwise the check fails
  return false;
}



bool ForceLocalPlanner::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose, 
                                        std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
  if (global_plan.empty())
    return true;
  
  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
    geometry_msgs::PoseStamped robot;
    tf2::doTransform(global_pose, robot, global_to_plan_transform);
    
    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
    
    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
         erase_end = it;
         break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;
    
    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}


bool ForceLocalPlanner::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                  const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                  std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::TransformStamped* tf_plan_to_global)
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try 
  {
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp,
                                                                                  plan_pose.header.frame_id, ros::Duration(3.0));

    //let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                           // located on the border of the local costmap
    

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;
    
    //we need to loop to a point on the plan that is within a certain distance of the robot
    bool robot_reached = false;
    for(int j=0; j < (int)global_plan.size(); ++j)
    {
      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (new_sq_dist > sq_dist_threshold)
        break;  // force stop if we have reached the costmap border

      if (robot_reached && new_sq_dist > sq_dist)
        break;

      if (new_sq_dist < sq_dist) // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
        if (sq_dist < 0.05)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
          robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
      }
    }
    
    geometry_msgs::PoseStamped newer_pose;
    
    double plan_length = 0; // check cumulative Euclidean distance along the plan
    
    //now we'll transform until points are outside of our distance threshold
    while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
    {
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      
      // caclulate distance to previous pose
      if (i>0 && max_plan_length>0)
        plan_length += distancePoints2D(global_plan[i-1].pose.position, global_plan[i].pose.position);

      ++i;
    }
        
    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
    // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty())
    {
      tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);
      
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
    }
    else
    {
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }
    
    // Return the transformation from the global plan to the global planning frame if desired
    if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
  }
  catch(tf::LookupException& ex)
  {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ConnectivityException& ex) 
  {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex) 
  {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    return false;
  }

  return true;
}

} // namespace force_local_planner
