#include "linear_blend_skinning.h"

void linear_blend_skinning(
  const Eigen::MatrixXd & V,
  const Skeleton & skeleton,
  const std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T,
  const Eigen::MatrixXd & W,
  Eigen::MatrixXd & U)
{
  /////////////////////////////////////////////////////////////////////////////
  const int V_size = V.size();
  U.resize(V_size, 3);
  Eigen::Vector4d pose;
  
  for (int i = 0; i < V_size; i++) {
    pose = Eigen::Vector4d::Zero();
    for (int j = 0; j < skeleton.size(); j++) {
      if (skeleton[j].weight_index > 0) {
        pose += T[j] * Eigen::Vector4d(V(i, 0), V(i, 1), V(i, 2), 1.0) * W(i, skeleton[j].weight_index);
      }
    }
    U.row(i) = Eigen::Vector3d(pose.x()/pose.w(), pose.y()/ pose.w(), pose.z()/ pose.w()); 
  }
  /////////////////////////////////////////////////////////////////////////////
}
