#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  /////////////////////////////////////////////////////////////////////////////
  double step_distance = max_step;
  const double E = f(z);
  Eigen::VectorXd new_z = z - step_distance * dz;
  proj_z(new_z);

  while (f(new_z) > E) {
		  step_distance *= 0.5;
		  new_z = z - step_distance * dz;
		  proj_z(new_z);
  }

  return step_distance;
  /////////////////////////////////////////////////////////////////////////////
}
