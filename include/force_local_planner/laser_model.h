#ifndef LASER_MODEL_H_
#define LASER_MODEL_H_
#include <math.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

#include <ros/ros.h>
#include <angles/angles.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>


namespace force_local_planner
{
/** @brief Simulate costmap as it is laser scan
 *  
 *  It divides the costmap to the regions with the given angle resolution.
 *  Then it returns minimum range of the obstacle in the robot frame.
 * 
*/
class LaserModel
{
public:
  /**
   * @param[in] costmap: Costmap object
   * @param[in] angle_res: Angle resolution which costmap is divided with
  */
  LaserModel(const costmap_2d::Costmap2D& costmap, const double& angle_res);

  ~LaserModel();

  /** @brief Get obstacles in the divided regions
   * 
   * @param[in] robot_pose: Robot pose in the map frame
   * @param[out] obstacles: Obstacles in regions
  */
  void getObstacles(const geometry_msgs::PoseStamped& robot_pose, 
                    std::vector<std::vector<std::pair<double, double>>>& obstacles);

  /** @brief Get min obstacles in the divided regions
   * 
   * @param[in] robot_pose: Robot pose in the map frame
   * @param[out] obstacles: Obstacles in regions
  */
  void getMinObstacles(const geometry_msgs::PoseStamped& robot_pose, 
                       std::vector<std::pair<double, double>>& obstacles);

private:
  const costmap_2d::Costmap2D& costmap_; ///< @brief Allows access of costmap obstacle information
  double angle_res_;

private:
  int getIndice(const double& angle)
  {

  }
};

} // namespace force_local_planner
#endif