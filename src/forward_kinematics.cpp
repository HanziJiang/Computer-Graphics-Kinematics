#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function


Eigen::Affine3d get_pose(const int i, const Skeleton skeleton) {
  const int parent_index = skeleton[i].parent_index;
  
  // Do nothing if bone is root
  if (parent_index < 0) return Eigen::Affine3d::Identity();

  const Bone bone = skeleton[i];

  Eigen::Affine3d parent_pose = get_pose(parent_index, skeleton);
  
  return Eigen::Affine3d(parent_pose * bone.rest_T * euler_angles_to_transform(bone.xzx) * bone.rest_T.inverse());
}

void forward_kinematics(
  const Skeleton & skeleton,
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T)
{
  /////////////////////////////////////////////////////////////////////////////
  T.resize(skeleton.size(),Eigen::Affine3d::Identity());

  for (int i = 0; i < skeleton.size(); i++) {
    T[i] = get_pose(i, skeleton);
  }
  /////////////////////////////////////////////////////////////////////////////
}
