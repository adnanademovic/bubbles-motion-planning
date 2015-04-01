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
#include "classic-tree.h"
#include "environment/environment-feedback.h"
#include "environment/fcl-environment.h"
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
  std::vector<double> start;
  std::vector<double> goal;
  std::string configuration;
};

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::vector<TestCase> test_cases;
  // Trivial test case
  test_cases.push_back({
      {-pi() / 4.0, pi() / 6.0 + pi() / 20.0, pi() / 12.0 - pi() / 20.0,
       pi() / 20.0, pi() / 20.0, pi() / 20.0},
      {pi() / 4.0, pi() / 6.0 - pi() / 20.0, pi() / 12.0 + pi() / 20.0,
       -pi() / 20.0, -pi() / 20.0, -pi() / 20.0},
      "motion-planning-data/abb-irb-120/case_trivial.conf"});
  // Easy test case
  test_cases.push_back({
      {-pi() / 3.0, pi() / 6.0 + pi() / 20.0, pi() / 12.0 - pi() / 20.0,
       pi() / 20.0, pi() / 20.0, pi() / 20.0},
      {pi() / 3.0, pi() / 6.0 - pi() / 20.0, pi() / 12.0 + pi() / 20.0,
       -pi() / 20.0, -pi() / 20.0, -pi() / 20.0},
      "motion-planning-data/abb-irb-120/case_easy.conf"});
  // Hard test case
  test_cases.push_back({
      {-pi() / 2.0, pi() / 3.6, -pi() / 3.6, 0.0, 0.0, 0.0},
      {0.0, pi() / 2.0, -pi() / 2.0, -pi() / 20.0, -pi() / 20.0, -pi() / 20.0},
      "motion-planning-data/abb-irb-120/case_hard.conf"});
  int test = FLAGS_simulation_case;

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
      new EnvironmentFeedback(new FclEnvironment(
          test_cases[test].configuration)));
  std::shared_ptr<EnvironmentFeedback> dst_collision_source(
      new EnvironmentFeedback(new FclEnvironment(
          test_cases[test].configuration)));

  double max_step = pi()/50.0;
  int ministeps_per_step = 10;
  RrtTree* src_tree = new ClassicTree(
      max_step, ministeps_per_step, test_cases[test].start,
      src_collision_source);
  RrtTree* dst_tree = new ClassicTree(
      max_step, ministeps_per_step, test_cases[test].goal,
      dst_collision_source);

  generators::GeneratorSettings generator_settings;
  generator_settings.set_type(generators::GeneratorSettings::SIMPLE);
  Rrt classic_rrt(src_tree, dst_tree, NewGeneratorFromProtoBuffer(
      limits, generator_settings));
  int step = 0;
  while (!classic_rrt.Step()) {
    fprintf(stderr, "Current step: %6d\n", ++step);
  }
  fprintf(stderr, "Final step: %6d\n", step);
  OutputPath(classic_rrt.GetSolution());
  return 0;
}
