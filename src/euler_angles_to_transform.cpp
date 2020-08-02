#include "euler_angles_to_transform.h"

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  /////////////////////////////////////////////////////////////////////////////
  // convert angles from degrees to radians
  const double angle_1 = xzx.x() * M_PI / 180.0;
  const double angle_2 = xzx.y() * M_PI / 180.0;
  const double angle_3 = xzx.z() * M_PI / 180.0;

  return Eigen::Affine3d (
    Eigen::AngleAxisd(angle_3, Eigen::Vector3d::UnitX()) * 
    Eigen::AngleAxisd(angle_2,  Eigen::Vector3d::UnitZ()) * 
    Eigen::AngleAxisd(angle_1, Eigen::Vector3d::UnitX())
    );
  /////////////////////////////////////////////////////////////////////////////
}
