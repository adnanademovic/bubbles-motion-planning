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
#include <utility>
#include <vector>

#include <gflags/gflags.h>

#include "rrt.h"
#include "bubble-tree.h"
#include "classic-tree.h"
#include "environment/environment-feedback.h"
#include "environment/fcl-environment.h"
#include "generators/generator.pb.h"
#include "generators/make-generator.h"

using namespace com::ademovic::bubblesmp;
using namespace com::ademovic::bubblesmp::environment;
using namespace com::ademovic::bubblesmp::generators;

void OutputPath(std::vector<std::shared_ptr<TreePoint> > bubbles) {
  printf("configs=[];\n");
  for (const auto& bubble : bubbles) {
    printf("configs = [configs;");
    for (const auto& pos : bubble->position())
      printf(" %lf", pos);
    printf("];\n");
  }
}

void MakeFile(const char filename[], const char input[]) {
  FILE* f = fopen(filename, "w");
  fprintf(f, "%s", input);
  fclose(f);
}

void MakeModelFile(const char filename[], const std::vector<float>& coords) {
  FILE* f = fopen(filename, "wb");
  uint8_t header[80];
  float normal[] = {0.0f, 0.0f, 0.0f};
  uint16_t attribute = 0;
  uint32_t triangle_count = static_cast<uint32_t>(coords.size() / 9);
  fwrite(header, sizeof(header[0]), 80, f);
  fwrite(&triangle_count, sizeof(triangle_count), 1, f);
  for (size_t i = 0; i < coords.size(); i += 9) {
    fwrite(normal, sizeof(normal[0]), 3, f);
    for (size_t j = 0; j < 9; ++j)
      fwrite(&coords[i + j], sizeof(coords[0]), 1, f);
    fwrite(&attribute, sizeof(attribute), 1, f);
  }
  fclose(f);
}

void Make2DPolyFile(const char filename[],
    const std::vector<std::pair<double, double> >& points) {
  std::vector<float> coords;
  for (size_t i = 1; i < points.size(); ++i) {
    coords.push_back(points[i - 1].first);
    coords.push_back(points[i - 1].second);
    coords.push_back(0.0f);
    coords.push_back(points[i].first);
    coords.push_back(points[i].second);
    coords.push_back(-1.0f);
    coords.push_back(points[i].first);
    coords.push_back(points[i].second);
    coords.push_back(1.0f);
  }
  MakeModelFile(filename, coords);
}

void Make2DLineFile(const char filename[], const std::vector<double>& points) {
  std::vector<float> coords;
  for (size_t i = 1; i < points.size(); ++i) {
    coords.push_back(points[i - 1]);
    coords.push_back(0.0f);
    coords.push_back(0.0f);
    coords.push_back(points[i]);
    coords.push_back(0.0f);
    coords.push_back(-1.0f);
    coords.push_back(points[i]);
    coords.push_back(0.0f);
    coords.push_back(1.0f);
  }
  MakeModelFile(filename, coords);
}

struct TestCase {
  std::string name;
  std::vector<double> start;
  std::vector<double> goal;
  std::string configuration;
};

void RunBubbleTree(unsigned seed, const TestCase& test_case) {
  std::vector<std::pair<double, double> > limits(2);
  limits[0].first = -180.0;
  limits[0].second = 180.0;
  limits[1].first = -180.0;
  limits[1].second = 180.0;

  std::shared_ptr<EnvironmentFeedback> src_bubble_source(
      new EnvironmentFeedback(new FclEnvironment(test_case.configuration)));
  std::shared_ptr<EnvironmentFeedback> dst_bubble_source(
      new EnvironmentFeedback(new FclEnvironment(test_case.configuration)));

  IndexSettings index_settings;
  index_settings.mutable_index_params()->set_trees(8);
  index_settings.mutable_search_params()->set_checks(128);

  int bubbles_per_branch = 50;
  RrtTree* src_tree = new BubbleTree(
      limits, bubbles_per_branch, test_case.start, src_bubble_source, 0.3,
      index_settings);
  RrtTree* dst_tree = new BubbleTree(
      limits, bubbles_per_branch, test_case.goal, dst_bubble_source, 0.3,
      index_settings);

  generators::GeneratorSettings generator_settings;
  generator_settings.set_type(generators::GeneratorSettings::SIMPLE);
  Rrt bubble_rrt(src_tree, dst_tree, NewGeneratorFromProtoBuffer(
      limits, generator_settings));
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
  std::vector<std::pair<double, double> > limits(2);
  limits[0].first = -180.0;
  limits[0].second = 180.0;
  limits[1].first = -180.0;
  limits[1].second = 180.0;

  std::shared_ptr<EnvironmentFeedback> src_collision_source(
      new EnvironmentFeedback(new FclEnvironment(test_case.configuration)));
  std::shared_ptr<EnvironmentFeedback> dst_collision_source(
      new EnvironmentFeedback(new FclEnvironment(test_case.configuration)));

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
      limits, generator_settings));
  int step = 0;
  while (!classic_rrt.Step())
    if (++step > 5000)
      break;
  fprintf(stderr, "Done Classic seed: %u; case: %s\n",
          seed, test_case.name.c_str());
  printf("classic_%s = [classic_%s %d];\n",
         test_case.name.c_str(), test_case.name.c_str(), step);
}

void GenerateFiles() {
  Make2DLineFile("sub11.stl", {0.0, 1.0});
  Make2DLineFile("sub21.stl", {1.0, 2.0});
  Make2DLineFile("sub31.stl", {2.0, 3.0});
  Make2DLineFile("sub41.stl", {3.0, 4.0});
  Make2DLineFile("sub51.stl", {4.0, 5.0});
  Make2DLineFile("sub12.stl", {5.0, 6.0});
  Make2DLineFile("sub22.stl", {6.0, 7.0});
  Make2DLineFile("sub32.stl", {7.0, 8.0});
  Make2DLineFile("sub42.stl", {8.0, 9.0});
  Make2DLineFile("sub52.stl", {9.0, 10.0});

  MakeFile("config1.conf",
      "robot_filename: \"robot.conf\","
      "environment_filename: \"obs.stl\","
      "max_underestimate: 0.1"
      );
  MakeFile("config2.conf",
      "robot_filename: \"robot.conf\","
      "environment_filename: \"obs.stl\","
      "max_underestimate: 0.1"
      );
  MakeFile("config3.conf",
      "robot_filename: \"robot.conf\","
      "environment_filename: \"obs.stl\","
      "max_underestimate: 0.1"
      );
  MakeFile("robot.conf",
      "segments {"
      "  a: 5"
      "  parts: \"sub11.stl\""
      "  parts: \"sub21.stl\""
      "  parts: \"sub31.stl\""
      "  parts: \"sub41.stl\""
      "  parts: \"sub51.stl\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      "segments {"
      "  a: 5"
      "  parts: \"sub12.stl\""
      "  parts: \"sub22.stl\""
      "  parts: \"sub32.stl\""
      "  parts: \"sub42.stl\""
      "  parts: \"sub52.stl\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      );

  std::vector<std::pair<double, double> > obstacles;
  obstacles.push_back({3.0, -6.0});
  obstacles.push_back({6.0, -3.0});
  obstacles.push_back({6.0, 3.0});
  obstacles.push_back({3.0, 6.0});
  obstacles.push_back({10.0, 6.0});
  obstacles.push_back({10.0, 15.0});
  obstacles.push_back({0.0, 15.0});
  obstacles.push_back({0.0, 7.0});
  obstacles.push_back({-2.0, 5.0});
  obstacles.push_back({-2.0, 0.0});
  obstacles.push_back({-5.0, -3.0});
  Make2DPolyFile("obs3.stl", obstacles);
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  GenerateFiles();

  std::vector<TestCase> test_cases;
  // Hard test case
  test_cases.push_back({
      "hard",
      {-90.0, 45.0},
      {90.0, -45.0},
      "config3.conf"});

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

