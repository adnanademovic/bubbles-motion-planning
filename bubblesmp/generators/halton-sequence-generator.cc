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

#include "bubblesmp/generators/halton-sequence-generator.h"

#include <chrono>

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace generators {

// TODO: add validity checks for keys.
HaltonSequenceGenerator::HaltonSequenceGenerator(
    const std::vector<std::pair<double, double> >& limits,
    const std::vector<unsigned>& keys, unsigned skips)
    : dimensions_(limits.size()),
      current_(dimensions_, std::vector<unsigned>(1, 1)), keys_(keys) {
  for (unsigned i = 0; i < dimensions_; ++i) {
    starts_.push_back(limits[i].first);
    steps_.emplace_back(1, (limits[i].second - limits[i].first) / keys[i]);
  }
  // TODO: generate the state instead of doing the incremental calculation.
  for (unsigned i = 0; i < skips; ++i)
    NextPoint();
}

HaltonSequenceGenerator::HaltonSequenceGenerator(
    const std::vector<std::pair<double, double> >& limits,
    const std::vector<unsigned>& keys)
    : HaltonSequenceGenerator(
        limits, keys,
        std::chrono::system_clock::now().time_since_epoch().count() & 0xFFFF) {}

std::vector<double> HaltonSequenceGenerator::NextPoint() {
  std::vector<double> point = starts_;
  for (unsigned i = 0; i < dimensions_; ++i) {
    unsigned depth = current_[i].size();
    for (unsigned j = 0; j < depth; ++j) {
      point[i] += steps_[i][j] * current_[i][j];
    }
    unsigned current_depth = 0;
    while (++current_[i][current_depth] >= keys_[i]) {
      current_[i][current_depth++] = 0;
      if (current_depth >= depth) {
        steps_[i].push_back(steps_[i].back() / keys_[i]);
        current_[i].push_back(0);
      }
    }
  }
  return point;
}

}  // namespace generators
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
