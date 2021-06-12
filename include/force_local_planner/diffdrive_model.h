#ifndef DIFFDRIVE_MODEL_H
#define DIFFDRIVE_MODEL_H

#include <Eigen/Dense>

namespace force_local_planner
{

/**
 * @brief Differential drive model
*/
class DiffdriveModel
{
public:
  /**
   * @param[in] mass: Mass of the robot in kg
   * @param[in] Izz: Inertia moment around a vertical axis z in kgm^2
   * @param[in] b: Viscous friction coefficient
   * @param[in] d: Wheel seperation in m
   * @param[in] r: Wheel radius in m
   * @param[in] h: Distance to the center of mass of the robot in m
  */
  DiffdriveModel(double mass, double Izz, double b, double d, double r, double h)
    : mass_(mass), Izz_(Izz), b_(b), d_(d), r_(r), h_(h)
  {
    double a1 = -2.0*b_*pow(d_,2)/(r_*Izz_);
    double a2 = a1*d_/r_ + 2.0*b_*d_/(mass_*pow(r_,2));
    double a3 = -2.0*b_/(mass_*r_);

    double b1 = h_/Izz_;
    double b2 = b1*d_/r_;
    double b3 = 1.0/(mass_*r_);

    H_.resize(3, 3);
    H_ << 0, 1, 0, 
          0, a1, 0,
          0, a2, a3;

    G_.resize(3, 2);
    G_ << 0, 0,
          b1, 0,
          b2, b3;
  }

  /**
   * @brief Apply differential drive robot dynamics
   * 
   * @param[in] x: State of the robot(Xk) (theta, theta_dot, right_wheel_angular_vel)
   * @param[in] u: Force applied to the front of the robot
   * @param[out] xn: Next state of the robot(Xk+1) with given x and u
  */
  void dynamics(const Eigen::Vector3d& x, const Eigen::Vector2d& u, Eigen::Vector3d& xn)
  {
    xn = H_ * x + G_ * u;
  }

private:
  double mass_;
  double Izz_;
  double b_;
  double d_;
  double r_;
  double h_;

  Eigen::MatrixXd H_;
  Eigen::MatrixXd G_;
};

} // namespace force_local_planner


#endif