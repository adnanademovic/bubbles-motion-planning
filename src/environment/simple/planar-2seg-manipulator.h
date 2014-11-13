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

#ifndef COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_SIMPLE_PLANAR_2SEG_MANIPULATOR_H_
#define COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_SIMPLE_PLANAR_2SEG_MANIPULATOR_H_

#include <memory>
#include <mutex>
#include <vector>

#include "robot-interface.h"

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace environment {
namespace simple {

class Planar2SegManipulator : public RobotInterface {
 public:
  Planar2SegManipulator(double segment_1_length, double segment_2_length);
  void set_coordinates(const std::vector<double>& coordinates);
  std::vector<double> coordinates() const;
  double DistanceToObstacle(const ObstacleInterface& obstacle) const;

 private:
  std::vector<double> coordinates_;
  double mid_x_, mid_y_, tip_x_, tip_y_;
  double segment_1_length_, segment_2_length_;
  
  mutable std::mutex guard_mutex_;
};

}  // namespace simple
}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com

#endif  // COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_SIMPLE_PLANAR_2SEG_MANIPULATOR_H_
