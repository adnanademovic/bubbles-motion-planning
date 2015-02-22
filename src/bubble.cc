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

#include "bubble.h"

#include <cmath>

namespace com {
namespace ademovic {
namespace bubblesmp {

Bubble::Bubble(const std::vector<double>& position,
               const std::vector<double>& size)
    : TreePoint(position), size_(size) {}

std::vector<double> Bubble::size() const {
  return size_;
}

bool Bubble::Contains(const std::vector<double>& q) const {
  unsigned int axis_count = position().size();
  double distance = 0.0;
  for (unsigned int i = 0; i < axis_count; ++i) {
    distance += fabs(q[i] - position()[i]) / size_[i];
  }
  return (distance < 1.0);
}

std::vector<double> Bubble::IntersectsHullAt(
    const std::vector<double>& q) const {
  unsigned int axis_count = position().size();
  double distance = 0.0;
  std::vector<double> new_q(axis_count, 0.0);
  for (unsigned int i = 0; i < axis_count; ++i) {
    new_q[i] = q[i] - position()[i];
    distance += fabs(new_q[i]) / size_[i];
  }
  for (unsigned int i = 0; i < axis_count; ++i)
    new_q[i] = position()[i] + new_q[i] / distance;
  return new_q;
}

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
