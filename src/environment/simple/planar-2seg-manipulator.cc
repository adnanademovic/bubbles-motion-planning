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
    double segment_1_length, double segment_2_length)
    : coordinates_(2, 0.0), mid_x_(0.0), mid_y_(0.0), tip_x_(0.0), tip_y_(0.0),
      segment_1_length_(segment_1_length), segment_2_length_(segment_2_length) {
    set_coordinates({0.0, 0.0});
}

void Planar2SegManipulator::set_coordinates(
    const std::vector<double>& coordinates) {
  coordinates_ = coordinates;
  mid_x_ = segment_1_length_ * std::cos(coordinates[0]);
  mid_y_ = segment_1_length_ * std::sin(coordinates[0]);
  tip_x_ = mid_x_ + segment_2_length_ * std::cos(coordinates[1]);
  tip_y_ = mid_y_ + segment_2_length_ * std::sin(coordinates[1]);
}

std::vector<double> Planar2SegManipulator::coordinates() const {
  return coordinates_;
}

double Planar2SegManipulator::DistanceToObstacle(
    const ObstacleInterface& obstacle) const {
  return std::min(
      obstacle.DistanceToLine(0.0, 0.0, 0.0, mid_x_, mid_y_, 0.0),
      obstacle.DistanceToLine(mid_x_, mid_y_, 0.0, tip_x_, tip_y_, 0.0));
}

}  // namespace simple
}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
