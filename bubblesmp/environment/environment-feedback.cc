//
// Copyright (c) 2015, Adnan Ademovic
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

#include "bubblesmp/environment/environment-feedback.h"

#include <cmath>
#include <limits>

#include "bubblesmp/bubble.h"
#include "bubblesmp/environment/environment-interface.h"

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace environment {
namespace {

constexpr double rad_to_deg() {
  return 45.0 / atan(1);
}

}  // namespace

EnvironmentFeedback::~EnvironmentFeedback() {}

EnvironmentFeedback::EnvironmentFeedback(EnvironmentInterface* environment)
    : environment_(environment) {}

Bubble* EnvironmentFeedback::NewBubble(
    const std::vector<double>& coordinates) const {

  std::vector<std::pair<double, std::vector<double> > > distance_profile(
      environment_->GetDistanceProfile(coordinates));

  std::vector<double> output(
      coordinates.size(), std::numeric_limits<double>::infinity());

  for (const auto& profile_part : distance_profile) {
    double distance = profile_part.first;
    int segment = 0;
    for (double subdistance : profile_part.second) {
      if (subdistance != 0.0)
        output[segment] = std::min(
            output[segment], rad_to_deg() * distance / subdistance);
      ++segment;
    }
  }

  return new Bubble(coordinates, output);
}

bool EnvironmentFeedback::IsCollision(
    const std::vector<double>& coordinates) const {
  return environment_->IsCollision(coordinates);
}

std::vector<std::pair<double, double> >
    EnvironmentFeedback::GetAngleRanges() const {
  return environment_->GetAngleRanges();
}

}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
