#include "copy_skeleton_at.h"
Skeleton copy_skeleton_at(
  const Skeleton & skeleton, 
  const Eigen::VectorXd & A)
{
  /////////////////////////////////////////////////////////////////////////////
  Skeleton skeleton_copy = skeleton;
  for (int i = 0; i < skeleton.size(); i++){
    skeleton_copy[i].xzx = Eigen::Vector3d(A[i * 3], A[i * 3 + 1], A[i * 3 + 2]);
  }
  return skeleton_copy;
  /////////////////////////////////////////////////////////////////////////////
}
