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
#include "classic-tree.h"
#include "environment/environment-feedback.h"
#include "environment/fcl-environment.h"
#include "generators/simple-generator.h"

using namespace com::ademovic::bubblesmp;
using namespace com::ademovic::bubblesmp::environment;
using namespace com::ademovic::bubblesmp::generators;

constexpr double pi() {
  return std::atan(1)*4;
}

// Does not draw start point
void DrawLine(
    const std::vector<double>& start, const std::vector<double>& goal) {
  double max_step = 0.05;
  std::vector<double> delta;
  for (size_t i = 0; i < start.size(); ++i)
    delta.push_back(goal[i] - start[i]);
  double distance = 0.0;
  for (double diff : delta)
    distance += fabs(diff);
  if (distance < max_step) {
    for (const auto& pos : goal)
      printf("%lf ", pos * 180.0 / pi());
    printf("\n");
  } else {
    int steps = static_cast<int>(distance / max_step + 1);
    for (size_t i = 0; i < delta.size(); ++i)
      delta[i] /= steps;
    for (int i = 1; i <= steps; ++i) {
      for (size_t j = 0; j < start.size(); ++j)
        printf("%lf ", (start[j] + i * delta[j]) * 180.0 / pi());
      printf("\n");
    }
  }
}

void OutputPath(std::vector<std::shared_ptr<TreePoint> > points) {
  if (!points.empty())
    DrawLine(points[0]->position(), points[0]->position());
  for (size_t i = 1; i < points.size(); ++i)
    DrawLine(points[i - 1]->position(), points[i]->position());
}

struct TestCase {
  std::string name;
  std::vector<double> start;
  std::vector<double> goal;
  std::string configuration;
};

void RunBubbleTree(unsigned seed, const TestCase& test_case) {
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

  std::shared_ptr<EnvironmentFeedback> src_bubble_source(
      new EnvironmentFeedback(new FclEnvironment(test_case.configuration)));
  std::shared_ptr<EnvironmentFeedback> dst_bubble_source(
      new EnvironmentFeedback(new FclEnvironment(test_case.configuration)));

  int bubbles_per_branch = 50;
  RrtTree* src_tree = new BubbleTree(
      limits, bubbles_per_branch, test_case.start, src_bubble_source, 0.3);
  RrtTree* dst_tree = new BubbleTree(
      limits, bubbles_per_branch, test_case.goal, dst_bubble_source, 0.3);

  Rrt bubble_rrt(src_tree, dst_tree, new SimpleGenerator(limits, seed));
  int step = 0;
  while (!bubble_rrt.Step())
    if (++step > 5000)
      break;
  fprintf(stderr, "Done Bubble seed: %u; case: %s\n",
          seed, test_case.name.c_str());
  printf("bubble_%s = [bubble_%s %d];\n",
         test_case.name.c_str(), test_case.name.c_str(), step);
}

void RunClassicTree(unsigned seed, const TestCase& test_case) {
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

  std::shared_ptr<EnvironmentFeedback> src_collision_source(
      new EnvironmentFeedback(new FclEnvironment(test_case.configuration)));
  std::shared_ptr<EnvironmentFeedback> dst_collision_source(
      new EnvironmentFeedback(new FclEnvironment(test_case.configuration)));

  double max_step = pi()/50.0;
  int ministeps_per_step = 10;
  RrtTree* src_tree = new ClassicTree(
      max_step, ministeps_per_step, test_case.start, src_collision_source);
  RrtTree* dst_tree = new ClassicTree(
      max_step, ministeps_per_step, test_case.goal, dst_collision_source);
  Rrt classic_rrt(src_tree, dst_tree, new SimpleGenerator(limits, seed));
  int step = 0;
  while (!classic_rrt.Step())
    if (++step > 5000)
      break;
  fprintf(stderr, "Done Classic seed: %u; case: %s\n",
          seed, test_case.name.c_str());
  printf("classic_%s = [classic_%s %d];\n",
         test_case.name.c_str(), test_case.name.c_str(), step);
}

int main() {
  std::vector<TestCase> test_cases;
  // Trivial test case
  test_cases.push_back({
      "trivial",
      {-pi() / 4.0, pi() / 6.0 + pi() / 20.0, pi() / 12.0 - pi() / 20.0,
       pi() / 20.0, pi() / 20.0, pi() / 20.0},
      {pi() / 4.0, pi() / 6.0 - pi() / 20.0, pi() / 12.0 + pi() / 20.0,
       -pi() / 20.0, -pi() / 20.0, -pi() / 20.0},
      "../models/abb1.conf"});
  // Easy test case
  test_cases.push_back({
      "easy",
      {-pi() / 3.0, pi() / 6.0 + pi() / 20.0, pi() / 12.0 - pi() / 20.0,
       pi() / 20.0, pi() / 20.0, pi() / 20.0},
      {pi() / 3.0, pi() / 6.0 - pi() / 20.0, pi() / 12.0 + pi() / 20.0,
       -pi() / 20.0, -pi() / 20.0, -pi() / 20.0},
      "../models/abb2.conf"});
  // Hard test case
  test_cases.push_back({
      "hard",
      {-pi() / 2.0, pi() / 3.6, -pi() / 3.6, 0.0, 0.0, 0.0},
      {0.0, pi() / 2.0, -pi() / 2.0, -pi() / 20.0, -pi() / 20.0, -pi() / 20.0},
      "../models/abb3.conf"});

  for (const TestCase& test_case : test_cases) {
    printf("bubble_%s = [];\n", test_case.name.c_str());
    printf("classic_%s = [];\n", test_case.name.c_str());
  }
  for (unsigned i = 0; i < 50; ++i) {
    fprintf(stderr, "Step: %u\n", i);
    for (const TestCase& test_case : test_cases) {
      RunBubbleTree(i * 100, test_case);
      RunClassicTree(i * 100, test_case);
    }
  }
  return 0;
}
