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

#include "rrt.h"
#include "bubble-tree.h"
#include "environment/environment-feedback.h"
#include "environment/pqp-environment.h"
#include "generators/simple-generator.h"

using namespace com::ademovic::bubblesmp;
using namespace com::ademovic::bubblesmp::environment;
using namespace com::ademovic::bubblesmp::generators;

constexpr double pi() {
  return std::atan(1)*4;
}

void OutputPath(std::vector<std::shared_ptr<TreePoint> > bubbles) {
  std::vector<double> prev_pos(0);
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

int main() {
  std::vector<std::pair<double, double> > limits(2);
  limits[0].first = -3.14;
  limits[0].second = 3.14;
  limits[1].first = -3.14;
  limits[1].second = 3.14;

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

  MakeFile("config.conf", "{"
      "\"robot\": \"robot.conf\","
      "\"environment\": \"obs.stl\","
      "\"max_underestimate\": 0.1"
      "}");
  MakeFile("robot.conf", "{"
      "\"parts\":[\"sub11.stl\", \"sub21.stl\", \"sub31.stl\","
                 "\"sub41.stl\", \"sub51.stl\","
                 "\"sub12.stl\", \"sub22.stl\", \"sub32.stl\","
                 "\"sub42.stl\", \"sub52.stl\"],"
      "\"parts_per_joint\":[5, 5],"
      "\"dh\": [[5, 0, 0, 0], [5, 0, 0, 0]]"
      "}");

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
  Make2DPolyFile("obs.stl", obstacles);

  std::shared_ptr<EnvironmentFeedback> src_bubble_source(
      new EnvironmentFeedback(new PqpEnvironment("config.conf")));
  std::shared_ptr<EnvironmentFeedback> dst_bubble_source(
      new EnvironmentFeedback(new PqpEnvironment("config.conf")));
  int bubbles_per_branch = 50;

  RrtTree* src_tree = new BubbleTree(
      limits, bubbles_per_branch, {-3.1415/2.0, 3.1415/4.0},
      src_bubble_source, 0.3);
  RrtTree* dst_tree = new BubbleTree(
      limits, bubbles_per_branch, {3.1415/2.0, -3.1415/4.0},
      dst_bubble_source, 0.3);
  Rrt bubble_rrt(src_tree, dst_tree, new SimpleGenerator(limits));
  int step = 0;
  while (!bubble_rrt.Step()) {
    fprintf(stderr, "Current step: %6d\n", ++step);
  }
  fprintf(stderr, "Final step: %6d\n", step);

  PqpEnvironment printing_environment("config.conf");
  printf("obs=[];\n");
  for (int i = -100; i < 101; ++i)
    for (int j = -100; j < 101; ++j)
      if (printing_environment.IsCollision({i * 0.031415, j * 0.031415})) {
        printf("obs=[obs; %lf %lf];\n", i * 0.031415, j * 0.031415);
      }
  OutputPath(bubble_rrt.GetSolution());
  printf("figure;\n"
         "plot(obs(:,1), obs(:,2), 'r.');\n"
         "hold on;\n"
         "plot(configs(:,1), configs(:,2), 'b');\n"
         "axis([-pi pi -pi pi]);"
         "hold off;\n"
         "pause;\n");
  return 0;
}
