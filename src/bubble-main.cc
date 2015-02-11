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

#include <cmath>
#include <cstdio>

#include "rrt.h"
#include "bubble-tree.h"
#include "environment/environment-feedback-interface.h"
#include "environment/pqp-environment.h"
#include "environment/pqp-environment-feedback.h"
#include "generators/simple-generator.h"

using namespace com::ademovic::bubblesmp;
using namespace com::ademovic::bubblesmp::environment;
using namespace com::ademovic::bubblesmp::generators;

constexpr double pi() {
  return std::atan(1)*4;
}

void OutputPath(std::vector<std::shared_ptr<TreePoint> > bubbles) {
  std::vector<double> prev_pos(0);
  for (const auto& bubble : bubbles) {
    for (const auto& pos : bubble->position())
      printf("%lf ", pos * 180.0 / pi());
    printf("\n");
  }
}

int main() {
  std::vector<std::pair<double, double> > limits(6);
  limits[0].first = -2.87979;
  limits[0].second = 2.87979;
  limits[1].first = -1.91986;
  limits[1].second = 1.91986;
  limits[2].first = -1.57079;
  limits[2].second = 1.22173;
  limits[3].first = -2.79252;
  limits[3].second = 2.79252;
  limits[4].first = -2.09439;
  limits[4].second = 2.09439;
  limits[5].first = -6.98131;
  limits[5].second = 6.98131;
  
  std::string config("../models/abb.conf");
  std::string obstacle("../models/obs1.mdl");
  std::vector<std::string> segments;
  segments.emplace_back("../models/SEG_0.mdl");
  segments.emplace_back("../models/SEG_1.mdl");
  segments.emplace_back("../models/SEG_2.mdl");
  segments.emplace_back("../models/SEG_3.mdl");
  segments.emplace_back("../models/SEG_4.mdl");
  segments.emplace_back("../models/SEG_5.mdl");
  double threshold = 1.0;
  std::vector<int> parts_per_segment{1, 1, 1, 1, 1, 1};

  std::shared_ptr<EnvironmentFeedbackInterface> src_bubble_source(
      new PqpEnvironmentFeedback(new PqpEnvironment(
          config, segments, obstacle, threshold, parts_per_segment))); 
  std::shared_ptr<EnvironmentFeedbackInterface> dst_bubble_source(
      new PqpEnvironmentFeedback(new PqpEnvironment(
          config, segments, obstacle, threshold, parts_per_segment))); 

  int bubbles_per_branch = 50;
  RrtTree* src_tree = new BubbleTree(
      limits, bubbles_per_branch,
      {-pi() / 4.0, pi() / 6.0 + pi() / 20.0, pi() / 12.0 - pi() / 20.0,
       pi() / 20.0, pi() / 20.0, pi() / 20.0},
      src_bubble_source);
  RrtTree* dst_tree = new BubbleTree(
      limits, bubbles_per_branch,
      {pi() / 4.0, pi() / 6.0 - pi() / 20.0, pi() / 12.0 + pi() / 20.0,
       -pi() / 20.0, -pi() / 20.0, -pi() / 20.0},
      dst_bubble_source);

  Rrt bubble_rrt(src_tree, dst_tree, new SimpleGenerator(limits));
  int step = 0;
  while (!bubble_rrt.Step()) {
    fprintf(stderr, "Current step: %6d\n", ++step);
  }
  fprintf(stderr, "Final step: %6d\n", step);
  OutputPath(bubble_rrt.GetSolution());
  return 0;
}
