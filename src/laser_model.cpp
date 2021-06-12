#include "force_local_planner/laser_model.h"

namespace force_local_planner
{
LaserModel::LaserModel(const costmap_2d::Costmap2D& ma, const double& angle_res) : costmap_(ma), angle_res_(angle_res)
{}

LaserModel::~LaserModel()
{}

void LaserModel::getObstacles(const geometry_msgs::PoseStamped& robot_pose, 
                              std::vector<std::vector<std::pair<double, double>>>& obstacles)
{
  double yaw = tf::getYaw(robot_pose.pose.orientation);

  int num_regions = 2.0 * M_PI / (angle_res_ * M_PI / 180.0);
  obstacles.clear();
  obstacles.resize(num_regions);
  for(int i=0; i<costmap_.getSizeInCellsX(); ++i)
  {
    for(int j=0; j<costmap_.getSizeInCellsY(); ++j)
    {
      unsigned char cost = costmap_.getCost(i, j);
      double x, y;
      costmap_.mapToWorld(i, j, x, y);
      //ROS_INFO("XX: %f - YY: %f, cost: %d", x, y, cost);
      if(cost == costmap_2d::LETHAL_OBSTACLE)
      {
        double th = angles::normalize_angle_positive(atan2(y - robot_pose.pose.position.y, x - robot_pose.pose.position.x) - yaw);
        double rho = hypotf(x - robot_pose.pose.position.x, y - robot_pose.pose.position.y);

        int ind = floor(th / (angle_res_ * M_PI / 180.0));
        obstacles[ind].push_back(std::make_pair(th, rho));
      }
    }
  }
}

void LaserModel::getMinObstacles(const geometry_msgs::PoseStamped& robot_pose, 
                                 std::vector<std::pair<double, double>>& obstacles)
{
  std::vector<std::vector<std::pair<double, double>>> all_obstacles;
  getObstacles(robot_pose, all_obstacles);

  obstacles.clear();
  for(int i=0; i<all_obstacles.size(); ++i)
  {
    if(!all_obstacles[i].empty())
    {
      double angle = (i+1) * 15.0 * M_PI / 180.0;
      double min_rho = 1e99;
      for(auto pair : all_obstacles[i])
      {
        if(pair.second < min_rho)
        {
          angle = pair.first;
          min_rho = pair.second;
        }
      }

      obstacles.push_back(std::make_pair(angle, min_rho));
    }
  }
}

} // namespace force_local_planner
