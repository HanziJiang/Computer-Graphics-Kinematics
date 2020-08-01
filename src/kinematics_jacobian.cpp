#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>

void kinematics_jacobian(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  Eigen::MatrixXd & J)
{
  /////////////////////////////////////////////////////////////////////////////
  J = Eigen::MatrixXd::Zero(b.size()*3,skeleton.size()*3);
  Eigen::VectorXd tips = transformed_tips(skeleton, b);
  const double h = 1.0e-7;
  Skeleton skeleton_copy = skeleton;
  for (int i = 0; i < b.size(); i++) {
    for (int i3 = 0; i3 < 3; i3++) {
      for (int j = 0; j < skeleton.size(); j++){
		    for (int j3 = 0; j3 < 3; j3++) {
          skeleton_copy[j].xzx[j3] += h;
          Eigen::VectorXd new_tips = transformed_tips(skeleton_copy, b);
          skeleton_copy[j].xzx[j3] -= h;
			    J(3 * i + i3, 3 * j + j3) = (new_tips[3 * i + i3] - tips[3 * i + i3]) / h;
		    }  
      }
    }
  }
  
  /////////////////////////////////////////////////////////////////////////////
}
