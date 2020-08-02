#include "copy_skeleton_at.h"
Skeleton copy_skeleton_at(
  const Skeleton & skeleton, 
  const Eigen::VectorXd & A)
{
  /////////////////////////////////////////////////////////////////////////////
  Skeleton copy = skeleton;
  int skeleton_size = skeleton.size();
  for (int i = 0; i < skeleton_size; i++){
    copy[i].xzx = Eigen::Vector3d(A[i * 3], A[i * 3 + 1], A[i * 3 + 2]);
  }
  return copy;
  /////////////////////////////////////////////////////////////////////////////
}
