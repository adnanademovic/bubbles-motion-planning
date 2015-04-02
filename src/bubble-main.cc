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

#include <gflags/gflags.h>

#include "rrt.h"
#include "bubble-tree.h"
#include "environment/environment-feedback.h"
#include "environment/make-environment.h"
#include "generators/generator.pb.h"
#include "generators/make-generator.h"

DEFINE_int32(simulation_case, 2,
    "Chooses the case for the simulation (0 - trivial, 1 - easy, 2 - hard)");

namespace {
static bool ValidateSimulationCase(const char* flagname, int32_t value) {
  if (value < 0 || value > 2) {
    printf("Invalid value for --%s: %d\n", flagname, value);
    return false;
  }
  return true;
}

static const bool simulation_case_dummy = google::RegisterFlagValidator(
    &FLAGS_simulation_case, &ValidateSimulationCase);
}  // namespace

using namespace com::ademovic::bubblesmp;
using namespace com::ademovic::bubblesmp::environment;
using namespace com::ademovic::bubblesmp::generators;

// Does not draw start point
void DrawLine(
    const std::vector<double>& start, const std::vector<double>& goal) {
  double max_step = 3.6;
  std::vector<double> delta;
  for (size_t i = 0; i < start.size(); ++i)
    delta.push_back(goal[i] - start[i]);
  double distance = 0.0;
  for (double diff : delta)
    distance += fabs(diff);
  if (distance < max_step) {
    for (const auto& pos : goal)
      printf("%lf ", pos);
    printf("\n");
  } else {
    int steps = static_cast<int>(distance / max_step + 1);
    for (size_t i = 0; i < delta.size(); ++i)
      delta[i] /= steps;
    for (int i = 1; i <= steps; ++i) {
      for (size_t j = 0; j < start.size(); ++j)
        printf("%lf ", start[j] + i * delta[j]);
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
  std::vector<double> start;
  std::vector<double> goal;
  std::string configuration;
};

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::vector<TestCase> test_cases;
  // Trivial test case
  test_cases.push_back({
      {-45.0, 39.0, 6.0, 9.0, 9.0, 9.0},
      {45.0, 21.0, 21.0, -9.0, -9.0, -9.0},
      "motion-planning-data/abb-irb-120/case_trivial.conf"});
  // Easy test case
  test_cases.push_back({
      {-60.0, 39.0, 6.0, 9.0, 9.0, 9.0},
      {60.0, 21.0, 21.0, -9.0, -9.0, -9.0},
      "motion-planning-data/abb-irb-120/case_easy.conf"});
  // Hard test case
  test_cases.push_back({
      {-90.0, 50.0, -50.0, 0.0, 0.0, 0.0},
      {0.0, 90.0, -90.0, -9.0, -9.0, -9.0},
      "motion-planning-data/abb-irb-120/case_hard.conf"});
  int test = FLAGS_simulation_case;

  std::shared_ptr<EnvironmentFeedback> src_bubble_source(
      new EnvironmentFeedback(NewEnvironmentFromProtoBuffer(
          test_cases[test].configuration)));
  std::shared_ptr<EnvironmentFeedback> dst_bubble_source(
      new EnvironmentFeedback(NewEnvironmentFromProtoBuffer(
          test_cases[test].configuration)));

  IndexSettings index_settings;
  index_settings.mutable_index_params()->set_trees(8);
  index_settings.mutable_search_params()->set_checks(128);

  int bubbles_per_branch = 50;
  RrtTree* src_tree = new BubbleTree(
      bubbles_per_branch, test_cases[test].start, src_bubble_source, 1.8,
      index_settings);
  RrtTree* dst_tree = new BubbleTree(
      bubbles_per_branch, test_cases[test].goal, dst_bubble_source, 1.8,
      index_settings);

  generators::GeneratorSettings generator_settings;
  generator_settings.set_type(generators::GeneratorSettings::SIMPLE);
  Rrt bubble_rrt(src_tree, dst_tree, NewGeneratorFromProtoBuffer(
      src_bubble_source->GetAngleRanges(), generator_settings));
  int step = 0;
  while (!bubble_rrt.Step()) {
    fprintf(stderr, "Current step: %6d\n", ++step);
  }
  fprintf(stderr, "Final step: %6d\n", step);
  OutputPath(bubble_rrt.GetSolution());
  return 0;
}
