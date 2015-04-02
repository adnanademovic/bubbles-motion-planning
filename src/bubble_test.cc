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
#define BOOST_TEST_MODULE Bubble
#include "bubble.h"

#include <cmath>

#include <boost/test/unit_test.hpp>

using ::com::ademovic::bubblesmp::Bubble;

void CheckCloseCollections(const std::vector<double>& a,
                           const std::vector<double>& b,
                           double error = 0.001) {
  BOOST_CHECK_EQUAL(a.size(), b.size());
  for (unsigned int i = 0; i != a.size(); ++i)
    BOOST_CHECK_CLOSE(a[i], b[i], error);
}

BOOST_AUTO_TEST_CASE(constructor) {
  Bubble bubble({1.0, 2.0, 3.0}, {0.3, 0.7, 0.6});
  CheckCloseCollections(bubble.position(), {1.0, 2.0, 3.0});
  CheckCloseCollections(bubble.size(), {0.3, 0.7, 0.6});
}

BOOST_AUTO_TEST_CASE(contains_insides) {
  Bubble bubble({1.0, 2.0, 3.0}, {0.3, 0.7, 0.6});

  BOOST_CHECK(bubble.Contains({1.2, 2.0, 3.0}));
  BOOST_CHECK(bubble.Contains({1.0, 2.5, 3.0}));
  BOOST_CHECK(bubble.Contains({1.0, 2.0, 3.4}));
  BOOST_CHECK(bubble.Contains({0.8, 2.0, 3.0}));
  BOOST_CHECK(bubble.Contains({1.0, 1.4, 3.0}));
  BOOST_CHECK(bubble.Contains({1.0, 2.0, 2.5}));

  BOOST_CHECK(bubble.Contains({1.1, 2.2, 3.1}));
  BOOST_CHECK(bubble.Contains({1.2, 2.2, 3.0}));
  BOOST_CHECK(bubble.Contains({0.9, 2.2, 2.9}));
}

BOOST_AUTO_TEST_CASE(does_not_contain_exterior) {
  Bubble bubble({1.0, 2.0, 3.0}, {0.3, 0.7, 0.6});

  BOOST_CHECK(!bubble.Contains({1.4, 2.0, 3.0}));
  BOOST_CHECK(!bubble.Contains({1.0, 2.8, 3.0}));
  BOOST_CHECK(!bubble.Contains({1.0, 2.0, 3.7}));
  BOOST_CHECK(!bubble.Contains({0.6, 2.0, 3.0}));
  BOOST_CHECK(!bubble.Contains({1.0, 1.2, 3.0}));
  BOOST_CHECK(!bubble.Contains({1.0, 2.0, 2.3}));

  BOOST_CHECK(!bubble.Contains({1.2, 2.2, 3.1}));
  BOOST_CHECK(!bubble.Contains({1.2, 2.4, 3.0}));
  BOOST_CHECK(!bubble.Contains({0.8, 2.4, 2.9}));
}

BOOST_AUTO_TEST_CASE(hull_intersection_2d) {
  Bubble bubble({15.0, 20.0}, {10.0, 5.0});

  CheckCloseCollections(bubble.IntersectsHullAt({34.0, 20.0}), {25.0, 20.0});
  CheckCloseCollections(bubble.IntersectsHullAt({2.0, 20.0}), {5.0, 20.0});
  CheckCloseCollections(bubble.IntersectsHullAt({15.0, 47.0}), {15.0, 25.0});
  CheckCloseCollections(bubble.IntersectsHullAt({15.0, 12.0}), {15.0, 15.0});

  CheckCloseCollections(bubble.IntersectsHullAt({25.0, 25.0}), {20.0, 22.5});
  CheckCloseCollections(bubble.IntersectsHullAt({5.0, 15.0}), {10.0, 17.5});
  CheckCloseCollections(bubble.IntersectsHullAt({25.0, 15.0}), {20.0, 17.5});
  CheckCloseCollections(bubble.IntersectsHullAt({5.0, 25.0}), {10.0, 22.5});

  double angle_1 = atan2(10.0, 15.0);
  double angle_2 = atan2(5.0, 10.0);
  double sol = std::sin(angle_2)/std::sin(angle_1);
  double sol_x = 10 * sol / (1 + sol);
  double sol_y = sol_x / 1.5;
  sol_x += 15.0;
  sol_y += 20.0;

  CheckCloseCollections(
      bubble.IntersectsHullAt({30.0, 30.0}), {sol_x, sol_y}, 2.0);
}
