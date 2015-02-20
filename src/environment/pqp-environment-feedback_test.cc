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
#define BOOST_TEST_MODULE PqpEnvironmentFeedback
#include "pqp-environment-feedback.h"
#include <boost/test/unit_test.hpp>

#include <memory>

using ::com::ademovic::bubblesmp::Bubble;
using ::com::ademovic::bubblesmp::environment::PqpEnvironment;
using ::com::ademovic::bubblesmp::environment::PqpEnvironmentFeedback;

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
  
  PqpEnvironmentFeedback environment(new PqpEnvironment(
      "conf.testfile", {"seg1.testfile"}, "env.testfile", 0.1, {1}));

  BOOST_CHECK_EQUAL(environment.IsCollision({0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({1.57}), false);
}

BOOST_AUTO_TEST_CASE(two_segment_collision) {
  MakeFile("conf.testfile", "10 0 0 0\n10 0 0 0\n");
  MakeFile("seg1.testfile", "10 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("seg2.testfile", "20 0 0\n10 0 1\n10 0 -1\n");
  MakeFile("env.testfile", "8 -10 -10\n8 -10 10\n8 10 0\n");
  
  PqpEnvironmentFeedback environment(new PqpEnvironment(
      "conf.testfile", {"seg1.testfile", "seg2.testfile"}, "env.testfile",
      0.1, {1, 1}));

  BOOST_CHECK_EQUAL(environment.IsCollision({0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.78}), false);
}

BOOST_AUTO_TEST_CASE(three_segment_collision) {
  MakeFile("conf.testfile", "10 0 0 0\n10 0 0 0\n5 0 0 0\n");
  MakeFile("seg1.testfile", "10 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("seg2.testfile", "20 0 0\n10 0 1\n10 0 -1\n");
  MakeFile("seg3.testfile", "25 0 0\n20 0 1\n20 0 -1\n");
  MakeFile("env.testfile", "8 -20 -10\n8 -20 10\n8 20 0\n");
  
  PqpEnvironmentFeedback environment(new PqpEnvironment(
      "conf.testfile", {"seg1.testfile", "seg2.testfile", "seg3.testfile"},
      "env.testfile", 0.1, {1, 1, 1}));

  BOOST_CHECK_EQUAL(environment.IsCollision({0.0, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.78, 0.0}), false);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.78, -1.57}), true);
}

BOOST_AUTO_TEST_CASE(three_segment_multiple_part_collision) {
  MakeFile("conf.testfile", "10 0 0 0\n10 0 0 0\n5 0 0 0\n");
  MakeFile("seg11.testfile", "5 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("seg12.testfile", "10 0 0\n5 0 1\n5 0 -1\n");
  MakeFile("seg21.testfile", "15 0 0\n10 0 1\n10 0 -1\n");
  MakeFile("seg22.testfile", "20 0 0\n15 0 1\n15 0 -1\n");
  MakeFile("seg31.testfile", "22.5 0 0\n20 0 1\n20 0 -1\n");
  MakeFile("seg32.testfile", "25 0 0\n22.5 0 1\n22.5 0 -1\n");
  MakeFile("env.testfile", "8 -20 -10\n8 -20 10\n8 20 0\n");
  
  PqpEnvironmentFeedback environment(new PqpEnvironment(
      "conf.testfile", {
        "seg11.testfile", "seg12.testfile",
        "seg21.testfile", "seg22.testfile",
        "seg31.testfile", "seg32.testfile"
      }, "env.testfile", 0.1, {2, 2, 2}));

  BOOST_CHECK_EQUAL(environment.IsCollision({0.0, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.78, 0.0}), false);
  BOOST_CHECK_EQUAL(environment.IsCollision({0.78, 0.78, -1.57}), true);
}

BOOST_AUTO_TEST_CASE(trivial_bubble) {
  MakeFile("conf.testfile", "10 0 0 0\n");
  MakeFile("seg1.testfile", "10 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("env.testfile", "15 -10 -10\n15 -10 10\n15 10 0\n");
  
  PqpEnvironmentFeedback environment(new PqpEnvironment(
      "conf.testfile", {"seg1.testfile"}, "env.testfile", 0.1, {1}));

  std::unique_ptr<Bubble> bub;
  std::vector<double> dims;

  bub.reset(environment.NewBubble({0.0}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 1);
  BOOST_CHECK_CLOSE(dims[0], 5.0 / 11.0, AbsToRelTolerance(5.0 / 11.0, 0.11));

  bub.reset(environment.NewBubble({0.78}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 1);
  BOOST_CHECK_CLOSE(dims[0], 7.89 / 11.0, AbsToRelTolerance(7.89 / 11.0, 0.11));
}

BOOST_AUTO_TEST_CASE(trivial_bubble_two_part) {
  MakeFile("conf.testfile", "10 0 0 0\n");
  MakeFile("seg11.testfile", "5 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("seg12.testfile", "10 0 0\n5 0 1\n5 0 -1\n");
  MakeFile("env.testfile", "15 -10 -10\n15 -10 10\n15 10 0\n");
  
  PqpEnvironmentFeedback environment(new PqpEnvironment(
      "conf.testfile", {"seg11.testfile", "seg12.testfile"}, "env.testfile",
      0.1, {2}));

  std::unique_ptr<Bubble> bub;
  std::vector<double> dims;

  bub.reset(environment.NewBubble({0.0}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 1);
  BOOST_CHECK_CLOSE(dims[0], 5.0 / 11.0, AbsToRelTolerance(5.0 / 11.0, 0.11));

  bub.reset(environment.NewBubble({0.78}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 1);
  BOOST_CHECK_CLOSE(dims[0], 7.89 / 11.0, AbsToRelTolerance(7.89 / 11.0, 0.11));
}

BOOST_AUTO_TEST_CASE(two_segment_bubble) {
  MakeFile("conf.testfile", "10 0 0 0\n10 0 0 0\n");
  MakeFile("seg1.testfile", "10 0 0\n0 0 1\n0 0 -1\n");
  MakeFile("seg2.testfile", "20 0 0\n10 0 1\n10 0 -1\n");
  MakeFile("env.testfile", "25 -20 -10\n25 -20 10\n25 20 0\n");
  
  PqpEnvironmentFeedback environment(new PqpEnvironment(
      "conf.testfile", {"seg1.testfile", "seg2.testfile"}, "env.testfile",
      0.1, {1, 1}));

  std::unique_ptr<Bubble> bub;
  std::vector<double> dims;

  bub.reset(environment.NewBubble({0.0, 0.0}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(dims[0], 5.0 / 21.0, AbsToRelTolerance(5.0 / 21.0, 0.11));
  BOOST_CHECK_CLOSE(dims[1], 5.0 / 11.0, AbsToRelTolerance(5.0 / 11.0, 0.11));

  bub.reset(environment.NewBubble({0.78, 0.0}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(
      dims[0], 10.78 / 21.0, AbsToRelTolerance(10.78 / 21.0, 0.11));
  BOOST_CHECK_CLOSE(
      dims[1], 10.78 / 11.0, AbsToRelTolerance(10.78 / 11.0, 0.11));

  bub.reset(environment.NewBubble({0.78, 0.78}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(
      dims[0], 17.78 / 19.498, AbsToRelTolerance(17.78 / 19.498, 0.11));
  BOOST_CHECK_CLOSE(
      dims[1], 17.78 / 11.0, AbsToRelTolerance(17.78 / 11.0, 0.11));

  bub.reset(environment.NewBubble({0.0, 2.5}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(
      dims[0], 15.0 / 11.0, AbsToRelTolerance(15.0 / 15.0, 0.11));
  BOOST_CHECK_CLOSE(
      dims[1], 15.0 / 11.0, AbsToRelTolerance(15.0 / 15.0, 0.11));
}
