#include "copy_skeleton_at.h"
Skeleton copy_skeleton_at(
  const Skeleton & skeleton, 
  const Eigen::VectorXd & A)
{
  /////////////////////////////////////////////////////////////////////////////
  Skeleton copy = skeleton;
  const int size = skeleton.size();
  Eigen::Vector3d angle;
  for (int i = 0; i < size; i++) {
    Bone bone = copy[i];
    angle = Eigen::Vector3d(A[i * 3], A[i * 3 + 1], A[i * 3 + 2]);
    // TODO: check range
    // if (angle < bone.xzx_min) bone.xzx = bone.xzx_min;
    // else if (angle > bone.xzx_max) bone.xzx = bone.xzx_max;
    // else bone.xzx = angle;
    bone.xzx = angle;
  }
  return copy;
  /////////////////////////////////////////////////////////////////////////////
}
