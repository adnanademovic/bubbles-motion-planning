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

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE BubbleSource
#include "bubble-source.h"
#include <boost/test/unit_test.hpp>
#include <boost/test/test_tools.hpp>
#include "obstacle-interface.h"
#include "obstacle-rectangle.h"
#include "obstacle-sphere.h"
#include "planar-2seg-manipulator.h"

using ::com::ademovic::bubblesmp::environment::simple::BubbleSource;
using ::com::ademovic::bubblesmp::environment::simple::ObstacleInterface;
using ::com::ademovic::bubblesmp::environment::simple::ObstacleRectangle;
using ::com::ademovic::bubblesmp::environment::simple::ObstacleSphere;
using ::com::ademovic::bubblesmp::environment::simple::Planar2SegManipulator;

void CheckCloseCollections(std::vector<double> a, std::vector<double> b) {
  BOOST_CHECK_EQUAL(a.size(), b.size());
  for (unsigned int i = 0; i != a.size(); ++i)
    BOOST_CHECK_CLOSE(a[i], b[i], 0.001);
}

BOOST_AUTO_TEST_CASE(bubble_test_one_obstacle) {
  BubbleSource source(new Planar2SegManipulator(2.0, 1.0));
  source.AddObstacle(new ObstacleRectangle(4.0, -3.0, 5.0, 3.0));
  CheckCloseCollections(
      source.GetBubbleDimensions({0.0, 0.0}), {1.0 / 3.0, 1.0});
}

BOOST_AUTO_TEST_CASE(bubble_test_two_obstacles) {
  BubbleSource source(new Planar2SegManipulator(1.5, 1.0));
  source.AddObstacle(new ObstacleRectangle(4.0, -3.0, 5.0, 3.0));
  source.AddObstacle(new ObstacleSphere(0.0, 6.0, 0.0, 1.0));
  CheckCloseCollections(
      source.GetBubbleDimensions({M_PI / 2.0, 0.0}), {1.0, 2.5});
}
