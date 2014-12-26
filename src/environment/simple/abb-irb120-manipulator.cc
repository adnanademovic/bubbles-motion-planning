//
// Copyright (c) 2014, Adnan Ademovic
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include "abb-irb120-manipulator.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace environment {
namespace simple {
namespace {

double GetDistance(
    const std::vector<double>& p1, const std::vector<double>& p2) {
  double dx = p2[0] - p1[0];
  double dy = p2[1] - p1[1];
  double dz = p2[2] - p1[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

AbbIrb120Manipulator::AbbIrb120Manipulator()
    : coordinates_(6, 0.0), dh_transforms_(6),
      joint_coords_(7, std::vector<double>(3, 0.0)) {
  std::vector<double> a = std::vector<double>(6, 0.0);
  std::vector<double> d = std::vector<double>(6, 0.0);
  std::vector<double> alpha = std::vector<double>(6, 0.0);
  std::vector<double> theta = std::vector<double>(6, 0.0);

  a[1] = 270.0;
  a[2] = 70.0;

  alpha[0] = -1.570796327;
  alpha[2] = -1.570796327;
  alpha[3] = 1.570796327;
  alpha[4] = -1.570796327;

  d[0] = 290.0;
  d[3] = 302.0;
  d[5] = 72.0;

  theta[1] = -1.570796327;
  theta[5] = 1.570796327 * 2.0;

  for (int segment = 0; segment < 6; ++segment) {
    dh_transforms_[segment].Rotate(
        transforms::Transformation::Z, theta[segment]);
    dh_transforms_[segment].Translate(a[segment], 0.0, d[segment]);
    dh_transforms_[segment].Rotate(
        transforms::Transformation::X, alpha[segment]);
  }

  set_coordinates({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

void AbbIrb120Manipulator::set_coordinates(
    const std::vector<double>& coordinates) {
  std::lock_guard<std::mutex> guard(guard_mutex_);
  coordinates_ = coordinates;
  transforms::Transformation transformation;
  for (int segment = 0; segment < 6; ++segment) {
    transformation.Rotate(transforms::Transformation::Z, coordinates[segment]);
    transformation.Transform(dh_transforms_[segment]);
    joint_coords_[segment + 1][0] = transformation.coefficient(0, 3);
    joint_coords_[segment + 1][1] = transformation.coefficient(1, 3);
    joint_coords_[segment + 1][2] = transformation.coefficient(2, 3);
  }
}

std::vector<double> AbbIrb120Manipulator::coordinates() const {
  std::lock_guard<std::mutex> guard(guard_mutex_);
  return coordinates_;
}

double AbbIrb120Manipulator::DistanceToObstacle(
    const ObstacleInterface& obstacle, int part) const {
  std::lock_guard<std::mutex> guard(guard_mutex_);
  int segment = part / 5;
  int sub_part = part % 5;
  double start_x = joint_coords_[segment][0];
  double start_y = joint_coords_[segment][1];
  double start_z = joint_coords_[segment][2];
  double step_x = (joint_coords_[segment + 1][0] - start_x) / 5;
  double step_y = (joint_coords_[segment + 1][1] - start_y) / 5;
  double step_z = (joint_coords_[segment + 1][2] - start_z) / 5;
  return obstacle.DistanceToLine(
      start_x + step_x * sub_part,
      start_y + step_y * sub_part,
      start_z + step_z * sub_part,
      start_x + step_x * (sub_part + 1),
      start_y + step_y * (sub_part + 1),
      start_z + step_z * (sub_part + 1));
}

std::vector<double> AbbIrb120Manipulator::FurthestDistances(int part) const {
  std::lock_guard<std::mutex> guard(guard_mutex_);
  std::vector<double> joint_reaches(6, 0.0);
  int segment = part / 5;
  int sub_part = part % 5;
  for (int i = 0; i < segment; ++i)
    for (int j = 0; j <= i; ++j)
      joint_reaches[i] = std::max(joint_reaches[i],
          GetDistance(joint_coords_[j], joint_coords_[i + 1]));
  joint_reaches[segment] = (sub_part + 1) * 0.2 *
      GetDistance(joint_coords_[segment], joint_coords_[segment + 1]);
  return joint_reaches;
}

int AbbIrb120Manipulator::PartCount() const {
  return 30;
}

}  // namespace simple
}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
