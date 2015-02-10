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
#define BOOST_TEST_MODULE PqpEnvironment
#include "pqp-environment.h"
#include <boost/test/unit_test.hpp>

using ::com::ademovic::bubblesmp::environment::PqpEnvironment;

void MakeFile(const char filename[], const char input[]) {
  FILE* f = fopen(filename, "w");
  fprintf(f, "%s", input);
  fclose(f);
}

double AbsToRelTolerance(double value, double tolerance) {
  return 100.0 * tolerance / value;
}

BOOST_AUTO_TEST_CASE(trivial_collision) {
  MakeFile("conf.testfile", "10 0 0 0\n");
  MakeFile("seg1.testfile", "10 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("env.testfile", "5 -10 -10\n5 -10 10\n5 10 0\n");
  
  PqpEnvironment environment(
      "conf.testfile", {"seg1.testfile"}, "env.testfile", 0.1, {1});

  BOOST_CHECK_EQUAL(environment.IsCollision({0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({1.57}), false);
}

BOOST_AUTO_TEST_CASE(two_segment_collision) {
  MakeFile("conf.testfile", "10 0 0 0\n10 0 0 0\n");
  MakeFile("seg1.testfile", "10 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("seg2.testfile", "10 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("env.testfile", "8 -10 -10\n8 -10 10\n8 10 0\n");
  
  PqpEnvironment environment(
      "conf.testfile", {"seg1.testfile", "seg2.testfile"}, "env.testfile",
      0.1, {1, 1});

  BOOST_CHECK_EQUAL(environment.IsCollision({0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.78}), false);
}

BOOST_AUTO_TEST_CASE(three_segment_collision) {
  MakeFile("conf.testfile", "10 0 0 0\n10 0 0 0\n5 0 0 0\n");
  MakeFile("seg1.testfile", "10 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("seg2.testfile", "10 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("seg3.testfile", "5 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("env.testfile", "8 -20 -10\n8 -20 10\n8 20 0\n");
  
  PqpEnvironment environment(
      "conf.testfile", {"seg1.testfile", "seg2.testfile", "seg3.testfile"},
      "env.testfile", 0.1, {1, 1, 1});

  BOOST_CHECK_EQUAL(environment.IsCollision({0.0, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.78, 0.0}), false);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.78, -1.57}), true);
}

BOOST_AUTO_TEST_CASE(three_segment_multiple_part_collision) {
  MakeFile("conf.testfile", "10 0 0 0\n10 0 0 0\n5 0 0 0\n");
  MakeFile("seg11.testfile", "5 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("seg12.testfile", "10 0 0\n5 0 1\n5 0 -1\n");
  MakeFile("seg21.testfile", "5 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("seg22.testfile", "10 0 0\n5 0 1\n5 0 -1\n");
  MakeFile("seg31.testfile", "2.5 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("seg32.testfile", "5 0 0\n2.5 0 1\n2.5 0 -1\n");
  MakeFile("env.testfile", "8 -20 -10\n8 -20 10\n8 20 0\n");
  
  PqpEnvironment environment(
      "conf.testfile", {
        "seg11.testfile", "seg12.testfile",
        "seg21.testfile", "seg22.testfile",
        "seg31.testfile", "seg32.testfile"
      }, "env.testfile", 0.1, {2, 2, 2});

  BOOST_CHECK_EQUAL(environment.IsCollision({0.0, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.78, 0.0}), false);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.78, -1.57}), true);
}

BOOST_AUTO_TEST_CASE(trivial_distance) {
  MakeFile("conf.testfile", "10 0 0 0\n");
  MakeFile("seg1.testfile", "10 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("env.testfile", "15 -10 -10\n15 -10 10\n15 10 0\n");
  
  PqpEnvironment environment(
      "conf.testfile", {"seg1.testfile"}, "env.testfile", 0.1, {1});

  PqpEnvironment::DistanceProfile d;

  d = environment.GetDistanceProfile({0.0});
  BOOST_CHECK_EQUAL(d.size(), 1);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 5.0, AbsToRelTolerance(5.0, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 11.0, 1.0);

  d = environment.GetDistanceProfile({0.78});
  BOOST_CHECK_EQUAL(d.size(), 1);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 7.89, AbsToRelTolerance(7.89, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 11.0, 1.0);
}

BOOST_AUTO_TEST_CASE(trivial_distance_two_part) {
  MakeFile("conf.testfile", "10 0 0 0\n");
  MakeFile("seg11.testfile", "5 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("seg12.testfile", "10 0 0\n5 0 1\n5 0 -1\n");
  MakeFile("env.testfile", "15 -10 -10\n15 -10 10\n15 10 0\n");
  
  PqpEnvironment environment(
      "conf.testfile", {"seg11.testfile", "seg12.testfile"}, "env.testfile",
      0.1, {2});

  PqpEnvironment::DistanceProfile d;

  d = environment.GetDistanceProfile({0.0});
  BOOST_CHECK_EQUAL(d.size(), 2);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_EQUAL(d[1].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 10.0, AbsToRelTolerance(10.0, 0.11));
  BOOST_CHECK_CLOSE(d[1].first, 5.0, AbsToRelTolerance(5.0, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 6.0, 1.0);
  BOOST_CHECK_CLOSE(d[1].second[0], 11.0, 1.0);

  d = environment.GetDistanceProfile({0.78});
  BOOST_CHECK_EQUAL(d.size(), 2);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_EQUAL(d[1].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 11.4454, AbsToRelTolerance(11.4454, 0.11));
  BOOST_CHECK_CLOSE(d[1].first, 7.89, AbsToRelTolerance(7.89, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 6.0, 1.0);
  BOOST_CHECK_CLOSE(d[1].second[0], 11.0, 1.0);
}

BOOST_AUTO_TEST_CASE(two_segment_distance) {
  MakeFile("conf.testfile", "10 0 0 0\n10 0 0 0\n");
  MakeFile("seg1.testfile", "10 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("seg2.testfile", "10 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("env.testfile", "25 -20 -10\n25 -20 10\n25 20 0\n");
  
  PqpEnvironment environment(
      "conf.testfile", {"seg1.testfile", "seg2.testfile"}, "env.testfile",
      0.1, {1, 1});

  PqpEnvironment::DistanceProfile d;

  d = environment.GetDistanceProfile({0.0, 0.0});
  BOOST_CHECK_EQUAL(d.size(), 2);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 15.0, AbsToRelTolerance(15.0, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 11.0, 1.0);
  BOOST_CHECK_EQUAL(d[1].second.size(), 2);
  BOOST_CHECK_CLOSE(d[1].first, 5.0, AbsToRelTolerance(5.0, 0.11));
  BOOST_CHECK_CLOSE(d[1].second[0], 21.0, 1.0);
  BOOST_CHECK_CLOSE(d[1].second[1], 11.0, 1.0);

  d = environment.GetDistanceProfile({0.78, 0.0});
  BOOST_CHECK_EQUAL(d.size(), 2);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 17.89, AbsToRelTolerance(17.89, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 11.0, 1.0);
  BOOST_CHECK_EQUAL(d[1].second.size(), 2);
  BOOST_CHECK_CLOSE(d[1].first, 10.78, AbsToRelTolerance(10.78, 0.11));
  BOOST_CHECK_CLOSE(d[1].second[0], 21.0, 1.0);
  BOOST_CHECK_CLOSE(d[1].second[1], 11.0, 1.0);

  d = environment.GetDistanceProfile({0.78, 0.78});
  BOOST_CHECK_EQUAL(d.size(), 2);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 17.89, AbsToRelTolerance(17.89, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 11.0, 1.0);
  BOOST_CHECK_EQUAL(d[1].second.size(), 2);
  BOOST_CHECK_CLOSE(d[1].first, 17.78, AbsToRelTolerance(17.78, 0.11));
  BOOST_CHECK_CLOSE(d[1].second[0], 19.498, 1.0);
  BOOST_CHECK_CLOSE(d[1].second[1], 11.0, 1.0);

  d = environment.GetDistanceProfile({0.0, 2.5});
  BOOST_CHECK_EQUAL(d.size(), 2);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 15.0, AbsToRelTolerance(15.0, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 11.0, 1.0);
  BOOST_CHECK_EQUAL(d[1].second.size(), 2);
  BOOST_CHECK_CLOSE(d[1].first, 15.0, AbsToRelTolerance(5.0, 0.11));
  BOOST_CHECK_CLOSE(d[1].second[0], 11.0, 1.0);
  BOOST_CHECK_CLOSE(d[1].second[1], 11.0, 1.0);
}
