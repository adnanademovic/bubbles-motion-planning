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

#include "bubblesmp/generators/simple-generator.h"

#include <chrono>

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace generators {

SimpleGenerator::SimpleGenerator(
    const std::vector<std::pair<double, double> >& limits, unsigned seed)
    : generator_(seed) {
  for (const std::pair<double, double>& limit : limits)
    distributions_.emplace_back(limit.first, limit.second);
}

SimpleGenerator::SimpleGenerator(
    const std::vector<std::pair<double, double> >& limits)
    : SimpleGenerator(
        limits, std::chrono::system_clock::now().time_since_epoch().count()) {}

std::vector<double> SimpleGenerator::NextPoint() {
  std::vector<double> point;
  for (auto& distribution : distributions_)
    point.push_back(distribution(generator_));
  return point;
}

}  // namespace generators
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
