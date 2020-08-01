#include "transformed_tips.h"
#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton, 
  const Eigen::VectorXi & b)
{
  /////////////////////////////////////////////////////////////////////////////
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > T;
  forward_kinematics(skeleton, T);
  
  Eigen::Vector3d tips(b.size() * 3);
  Eigen::Vector4d a_tip;
  for (int i = 0; i < b.size(); i++) {
    a_tip = T[b[i]] * skeleton[b[i]].rest_T * Eigen::Vector4d(skeleton[b[i]].length, 0.0, 0.0, 1.0);
    tips[i * 3] = a_tip.x() / a_tip.w();
		tips[i * 3 + 1] = a_tip.y() / a_tip.w();
		tips[i * 3 + 2] = a_tip.z() / a_tip.w();
  }

  return tips;
  /////////////////////////////////////////////////////////////////////////////
}
