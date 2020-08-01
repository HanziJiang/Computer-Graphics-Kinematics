#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>

void end_effectors_objective_and_gradient(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  const Eigen::VectorXd & xb0,
  std::function<double(const Eigen::VectorXd &)> & f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  std::function<void(Eigen::VectorXd &)> & proj_z)
{
  /////////////////////////////////////////////////////////////////////////////
  f = [&](const Eigen::VectorXd & A)->double
  {
    Skeleton new_skeleton = copy_skeleton_at(skeleton, A);
  	Eigen::Vector3d tips = transformed_tips(new_skeleton, b);
    return (tips - xb0).dot(tips - xb0);
  };

  grad_f = [&](const Eigen::VectorXd & A)->Eigen::VectorXd
  {
    Skeleton new_skeleton = copy_skeleton_at(skeleton, A);
  	Eigen::Vector3d tips = transformed_tips(new_skeleton, b);
    
    Eigen::MatrixXd J;
  	kinematics_jacobian(new_skeleton, b, J);

    return J.transpose() * 2 * (tips - xb0);
  };

  proj_z = [&](Eigen::VectorXd & A)
  {
    assert(skeleton.size()*3 == A.size());
    for (int i = 0; i < skeleton.size(); i++) {
			A[3 * i] = std::max(skeleton[i].xzx_min[0], std::min(skeleton[i].xzx_max[0], A[3 * i]));
			A[3 * i + 1] = std::max(skeleton[i].xzx_min[1], std::min(skeleton[i].xzx_max[1], A[3 * i + 1]));
			A[3 * i + 2] = std::max(skeleton[i].xzx_min[2], std::min(skeleton[i].xzx_max[2], A[3 * i + 2]));  	
  	}
  };
  /////////////////////////////////////////////////////////////////////////////
}
