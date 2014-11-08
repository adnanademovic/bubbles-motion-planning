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
#define BOOST_TEST_MODULE ObstacleSphere
#include "obstacle-sphere.h"
#include <boost/test/unit_test.hpp>

using ::com::ademovic::bubblesmp::environment::simple::ObstacleSphere;

BOOST_AUTO_TEST_CASE(zero_distance) {
  ObstacleSphere obstacle(1.0, 1.0, 1.0, 0.5);

  BOOST_CHECK_EQUAL(obstacle.DistanceToLine(1.0, 0.0, 1.0, 1.0, 2.0, 1.0), 0.0);
}

BOOST_AUTO_TEST_CASE(nonzero_distance_inside_line) {
  ObstacleSphere obstacle(1.0, 1.0, 1.0, 0.5);

  BOOST_CHECK_CLOSE(obstacle.DistanceToLine(0.0, 0.0, 1.0, 0.0, 2.0, 1.0),
                    0.5, 0.001);
}

BOOST_AUTO_TEST_CASE(nonzero_distance_at_first_vertex) {
  ObstacleSphere obstacle(1.0, 1.0, 1.0, 0.6);

  BOOST_CHECK_CLOSE(obstacle.DistanceToLine(-0.6, -0.6, 0.2, -1.0, -2.0, 0.0),
                    1.8, 0.001);
}

BOOST_AUTO_TEST_CASE(nonzero_distance_at_second_vertex) {
  ObstacleSphere obstacle(1.0, 1.0, 1.0, 0.7);

  BOOST_CHECK_CLOSE(obstacle.DistanceToLine(-2.0, -2.0, 0.0, 0.5, 0.0, 0.0),
                    0.8, 0.001);
}
