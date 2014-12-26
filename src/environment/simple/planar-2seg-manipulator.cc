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

#include "planar-2seg-manipulator.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace environment {
namespace simple {

Planar2SegManipulator::Planar2SegManipulator(
    double segment_1_length, double segment_2_length, int parts_per_segment)
    : coordinates_(2, 0.0), mid_x_(0.0), mid_y_(0.0),
      segment_1_length_(segment_1_length), segment_2_length_(segment_2_length),
      parts_per_segment_(parts_per_segment) {
    set_coordinates({0.0, 0.0});
}

void Planar2SegManipulator::set_coordinates(
    const std::vector<double>& coordinates) {
  std::lock_guard<std::mutex> guard(guard_mutex_);
  coordinates_ = coordinates;
  double angle = coordinates[0];
  mid_x_ = segment_1_length_ * std::cos(angle);
  mid_y_ = segment_1_length_ * std::sin(angle);
  angle += coordinates[1];
  double tip_x = mid_x_ + segment_2_length_ * std::cos(angle);
  double tip_y = mid_y_ + segment_2_length_ * std::sin(angle);
  part_s1_x_ = mid_x_ / parts_per_segment_;
  part_s1_y_ = mid_y_ / parts_per_segment_;
  part_s2_x_ = (tip_x - mid_x_) / parts_per_segment_;
  part_s2_y_ = (tip_y - mid_x_) / parts_per_segment_;
}

std::vector<double> Planar2SegManipulator::coordinates() const {
  std::lock_guard<std::mutex> guard(guard_mutex_);
  return coordinates_;
}

double Planar2SegManipulator::DistanceToObstacle(
    const ObstacleInterface& obstacle, int part) const {
  std::lock_guard<std::mutex> guard(guard_mutex_);
  if (part < parts_per_segment_) {
    double start_x = part_s1_x_ * part;
    double start_y = part_s1_y_ * part;
    return obstacle.DistanceToLine(
        start_x, start_y, 0.0, start_x + part_s1_x_, start_y + part_s1_y_, 0.0);
  } else {
    part -= parts_per_segment_;
    double start_x = mid_x_ + part_s2_x_ * part;
    double start_y = mid_y_ + part_s2_y_ * part;
    return obstacle.DistanceToLine(
        start_x, start_y, 0.0, start_x + part_s2_x_, start_y + part_s2_y_, 0.0);
  }
}

std::vector<double> Planar2SegManipulator::FurthestDistances(int part) const {
  std::lock_guard<std::mutex> guard(guard_mutex_);
  std::vector<double> joint_reaches(2);
  if (part < parts_per_segment_) {
    joint_reaches[0] = segment_1_length_ * (part + 1) / parts_per_segment_;
    joint_reaches[1] = 0.0;
  } else {
    part -= parts_per_segment_;
    double tip_x = mid_x_ + part_s2_x_ * (part + 1);
    double tip_y = mid_y_ + part_s2_y_ * (part + 1);
    joint_reaches[0] =
        std::max(segment_1_length_, std::sqrt(tip_x * tip_x + tip_y * tip_y));
    joint_reaches[1] = segment_2_length_ * (part + 1) / parts_per_segment_;
  }
  return joint_reaches;
}

int Planar2SegManipulator::PartCount() const {
  return parts_per_segment_ * 2;
}

}  // namespace simple
}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
