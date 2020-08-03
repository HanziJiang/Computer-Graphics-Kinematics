#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>

Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{
  /////////////////////////////////////////////////////////////////////////////
  // Reference: http://graphics.cs.cmu.edu/nsp/course/15-462/Fall04/assts/catmullRom.pdf
  const int size = keyframes.size();

  if (size < 1) return Eigen::Vector3d(0.0, 0.0, 0.0);

  if (size == 1) return keyframes[0].second;

  const double tension = 0.5;


  t = fmod(t, keyframes[size - 1].first);

  // find where to insert the new frame
  int index = size;
  for (int i = 0; i < size; i++) {
    if (keyframes[i].first > t) {
      index = i;
      break;
    }
  }

  Eigen::Vector3d P0, P1, P2, P3;
  double t0, t1, t2, t3;

  if (index <= 1) {
    P0 = keyframes[0].second;
    P1 = keyframes[0].second;
    t0 = keyframes[0].first;
    t1 = keyframes[0].first;
  } else {
    P0 = keyframes[index - 2].second;
    P1 = keyframes[index - 1].second;
    t0 = keyframes[index - 2].first;
    t1 = keyframes[index - 1].first;
  }

  if (index >= size - 1) {
    P2 = keyframes[size - 1].second;
    P3 = keyframes[size - 1].second;
    t2 = keyframes[size - 1].first;
    t3 = keyframes[size - 1].first;
  } else {
    P2 = keyframes[index].second;
    P3 = keyframes[index + 1].second;
    t2 = keyframes[index].first;
    t3 = keyframes[index + 1].first;
  }

  const Eigen::Vector3d a = P1;
  const Eigen::Vector3d b = -tension * P0 + tension * P2;
  const Eigen::Vector3d c = 2 * tension * P0 + (tension - 3) * P1 + (3 - 2 * tension) * P2 - tension * P3;
  const Eigen::Vector3d d = -tension * P0 + (2 - tension) * P1 + (tension - 2) * P2 + tension * P3;

  const double u = (t2 - t1 == 0) ? 0.5 : abs((t - t1) / (t2 - t1));

  return d * pow(u, 3) + c * pow(u, 2) + b * u + a;
  /////////////////////////////////////////////////////////////////////////////
}
