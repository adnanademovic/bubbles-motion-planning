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
#include "classic-tree.h"
#include "environment/environment-feedback.h"
#include "environment/make-environment.h"
#include "generators/generator.pb.h"
#include "generators/make-generator.h"

using namespace com::ademovic::bubblesmp;
using namespace com::ademovic::bubblesmp::environment;
using namespace com::ademovic::bubblesmp::generators;

struct TestCase {
  std::string name;
  std::vector<double> start;
  std::vector<double> goal;
  std::string configuration;
};

void RunBubbleTree(unsigned seed, const TestCase& test_case) {
  std::shared_ptr<EnvironmentFeedback> src_bubble_source(
      new EnvironmentFeedback(NewEnvironmentFromProtoBuffer(test_case.configuration)));
  std::shared_ptr<EnvironmentFeedback> dst_bubble_source(
      new EnvironmentFeedback(NewEnvironmentFromProtoBuffer(test_case.configuration)));

  IndexSettings index_settings;
  index_settings.mutable_index_params()->set_trees(8);
  index_settings.mutable_search_params()->set_checks(128);

  int bubbles_per_branch = 50;
  unsigned bin_search_depth = 4;
  RrtTree* src_tree = new BubbleTree(
      bubbles_per_branch, bin_search_depth, test_case.start, src_bubble_source,
      1.8, index_settings);
  RrtTree* dst_tree = new BubbleTree(
      bubbles_per_branch, bin_search_depth, test_case.goal, dst_bubble_source,
      1.8, index_settings);

  generators::GeneratorSettings generator_settings;
  generator_settings.set_type(generators::GeneratorSettings::SIMPLE);
  Rrt bubble_rrt(src_tree, dst_tree, NewGeneratorFromProtoBuffer(
      src_bubble_source->GetAngleRanges(), generator_settings));
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
  std::shared_ptr<EnvironmentFeedback> src_collision_source(
      new EnvironmentFeedback(NewEnvironmentFromProtoBuffer(
          test_case.configuration)));
  std::shared_ptr<EnvironmentFeedback> dst_collision_source(
      new EnvironmentFeedback(NewEnvironmentFromProtoBuffer(
          test_case.configuration)));

  IndexSettings index_settings;
  index_settings.mutable_index_params()->set_trees(8);
  index_settings.mutable_search_params()->set_checks(128);

  double max_step = 3.6;
  int ministeps_per_step = 10;
  RrtTree* src_tree = new ClassicTree(
      max_step, ministeps_per_step, test_case.start, src_collision_source,
      index_settings);
  RrtTree* dst_tree = new ClassicTree(
      max_step, ministeps_per_step, test_case.goal, dst_collision_source,
      index_settings);

  generators::GeneratorSettings generator_settings;
  generator_settings.set_type(generators::GeneratorSettings::SIMPLE);
  Rrt classic_rrt(src_tree, dst_tree, NewGeneratorFromProtoBuffer(
      src_collision_source->GetAngleRanges(), generator_settings));
  int step = 0;
  while (!classic_rrt.Step())
    if (++step > 5000)
      break;
  fprintf(stderr, "Done Classic seed: %u; case: %s\n",
          seed, test_case.name.c_str());
  printf("classic_%s = [classic_%s %d];\n",
         test_case.name.c_str(), test_case.name.c_str(), step);
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::vector<TestCase> test_cases;
  // Trivial test case
  test_cases.push_back({
      "trivial",
      {-45.0, 39.0, 6.0, 9.0, 9.0, 9.0},
      {45.0, 21.0, 21.0, -9.0, -9.0, -9.0},
      "motion-planning-data/abb-irb-120/case_trivial.conf"});
  // Easy test case
  test_cases.push_back({
      "easy",
      {-60.0, 39.0, 6.0, 9.0, 9.0, 9.0},
      {60.0, 21.0, 21.0, -9.0, -9.0, -9.0},
      "motion-planning-data/abb-irb-120/case_easy.conf"});
  // Hard test case
  test_cases.push_back({
      "hard",
      {-90.0, 50.0, -50.0, 0.0, 0.0, 0.0},
      {0.0, 90.0, -90.0, -9.0, -9.0, -9.0},
      "motion-planning-data/abb-irb-120/case_hard.conf"});

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
