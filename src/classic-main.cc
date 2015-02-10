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
#include "classic-tree.h"
#include "environment/simple/abb-irb120-manipulator.h"
#include "environment/simple/bubble-source.h"
#include "environment/simple/obstacle-rectangle.h"
#include "environment/simple/obstacle-sphere.h"
#include "generators/simple-generator.h"

using namespace com::ademovic::bubblesmp;
using namespace com::ademovic::bubblesmp::environment;
using namespace com::ademovic::bubblesmp::environment::simple;
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
  std::vector<BubbleSource*> bubble_sources;
  for (int sources = 0; sources < 2; ++sources) {
    BubbleSource* bs = new BubbleSource(new AbbIrb120Manipulator);
    double position = 250;
    double bottom = 400.0;
    double movement = 100.0;
    for (int j = -1; j < 2; j += 2)
      for (int i = 0; i < 5; i++)
      {
        bs->AddObstacle(new ObstacleSphere(
              position + movement,
              j * (-position + movement),
              bottom + i * movement/2,
              50));
        bs->AddObstacle(new ObstacleSphere(
              position - movement,
              j * (-position - movement),
              bottom + i * movement/2,
              50));
        bs->AddObstacle(new ObstacleSphere(
              position - movement + ( i + .5) * movement / 2.5,
              j * (-position - movement + (i + .5) * movement / 2.5),
              bottom - 50,
              50));
        bs->AddObstacle(new ObstacleSphere(
              position - movement + (i + .5) * movement / 2.5,
              j * (-position - movement + (i + .5) * movement / 2.5),
              bottom - 50 + movement * 3,
              50));
      }
    bubble_sources.push_back(bs);
  }
  std::shared_ptr<EnvironmentFeedbackInterface> src_bubble_source(
      bubble_sources[0]);
  std::shared_ptr<EnvironmentFeedbackInterface> dst_bubble_source(
      bubble_sources[1]);
  double max_step = pi()/50.0;
  int ministeps_per_step = 10;
  RrtTree* src_tree = new ClassicTree(
      max_step, ministeps_per_step,
      {-3.1415/4.0, 3.1415/3.0, -3.1415/3.0,
       3.1415/2.0, 3.1415/4.0, 3.1415/2.0},
      src_bubble_source);
  RrtTree* dst_tree = new ClassicTree(
      max_step, ministeps_per_step,
      {3.1415/4.0, 3.1415/3.0, -3.1415/3.0,
       -3.1415/2.0, -3.1415/4.0, -3.1415/2.0},
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