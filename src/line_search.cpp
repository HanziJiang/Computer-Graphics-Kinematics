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
  const double energy = f(z);
  Eigen::VectorXd new_z = z - step_distance * dz;
  proj_z(new_z);
  int num_step = 0;
  while (f(new_z) > energy && num_step < 40) {
		  step_distance /= 2;
		  new_z = z - step_distance * dz;
		  proj_z(new_z);
      num_step++;
  }

  return step_distance;
  /////////////////////////////////////////////////////////////////////////////
}
