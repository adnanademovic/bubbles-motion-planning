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

#include <cstdio>

#include "bubble-rrt.h"
#include "bubble-tree.h"
#include "environment/simple/bubble-source.h"
#include "environment/simple/obstacle-rectangle.h"
#include "environment/simple/obstacle-sphere.h"
#include "environment/simple/planar-2seg-manipulator.h"
#include "generators/simple-generator.h"

using namespace com::ademovic::bubblesmp;
using namespace com::ademovic::bubblesmp::environment;
using namespace com::ademovic::bubblesmp::environment::simple;
using namespace com::ademovic::bubblesmp::generators;

void doTree(std::vector<std::shared_ptr<Bubble> > bubbles) {
  std::vector<double> prev_pos(0);
  for (const auto& bubble : bubbles) {
    auto pos = bubble->position();
    auto siz = bubble->size();
    printf("plot(%lf, %lf, 'gx');\n", pos[0], pos[1]);
    printf("plot([%lf %lf %lf %lf %lf], [%lf %lf %lf %lf %lf], 'b');\n",
        pos[0], pos[0] + siz[0], pos[0], pos[0] - siz[0], pos[0],
        pos[1] + siz[1], pos[1], pos[1] - siz[1], pos[1], pos[1] + siz[1]);
    if (!prev_pos.empty()) {
      printf("plot([%lf %lf], [%lf %lf], 'k');\n",
          pos[0], prev_pos[0], pos[1], prev_pos[1]);
    }
    prev_pos = pos;
  }
}

int main() {
  BubbleSource* bs = new BubbleSource(new Planar2SegManipulator(3.0, 2.0));
  bs->AddObstacle(new ObstacleRectangle(-3.0, -3.0, -2.0, -2.0));
  bs->AddObstacle(new ObstacleSphere(3.0, 3.0, 0.0, 1.0));
  std::shared_ptr<BubbleSourceInterface> bubble_source(bs);
  double step = 3.1415 / 50.0;
  printf("environment = [];\n");
  bool has_environment = false;
  for (int i = -50; i < 50; ++i)
    for (int j = -50; j < 50; ++j) {
      double x = i * step;
      double y = j * step;
      double bubble_sum = 0.0;
      std::unique_ptr<Bubble> bubble(bubble_source->NewBubble({x, y}));
      for (double dimension : bubble->size())
        bubble_sum += dimension;
      if (bubble_sum == 0.0) {
        printf("environment = [environment; %lf %lf];\n", x, y);
        has_environment = true;
      }
    }
  double pi = 3.1415;
  BubbleRrt bubble_rrt({0, 0}, {2, -2}, 3, bubble_source,
                       new SimpleGenerator({{-pi, pi}, {-pi, pi}}));
  printf("figure;\nhold on;\n");
  if (has_environment)
    printf("plot(environment(:,1),environment(:,2),'ro');\n");
  printf("axis([-3.5 3.5 -3.5 3.5]);\n");
  bubble_rrt.Run(100);
  doTree(bubble_rrt.GetSolution());
  return 0;
}
