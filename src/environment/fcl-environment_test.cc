//
// Copyright (c) 2015, Adnan Ademovic
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
#define BOOST_TEST_MODULE FclEnvironment
#include "fcl-environment.h"
#include <boost/test/unit_test.hpp>
#include <cstdio>

using ::com::ademovic::bubblesmp::environment::EnvironmentInterface;
using ::com::ademovic::bubblesmp::environment::FclEnvironment;

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
      fwrite(&coords[j], sizeof(coords[0]), 1, f);
    fwrite(&attribute, sizeof(attribute), 1, f);
  }
  fclose(f);
}

double AbsToRelTolerance(double value, double tolerance) {
  return 100.0 * tolerance / value;
}

BOOST_AUTO_TEST_CASE(trivial_collision) {
  MakeFile("conf.testfile",
      "robot_filename: \"robot.testfile\","
      "environment_filename: \"env.testfile\","
      );
  MakeFile("robot.testfile",
      "segments {"
      "  a: 10"
      "  parts: \"seg1.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      );
  MakeModelFile("seg1.testfile", {10, 0, 0, 0, 0, 1, 0, 0, -1});
  MakeModelFile("env.testfile", {5, -10, -10, 5, -10, 10, 5, 10, 0});

  FclEnvironment environment("conf.testfile");

  BOOST_CHECK_EQUAL(environment.IsCollision({0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({90.0}), false);
}

BOOST_AUTO_TEST_CASE(two_segment_collision) {
  MakeFile("conf.testfile",
      "robot_filename: \"robot.testfile\","
      "environment_filename: \"env.testfile\","
      );
  MakeFile("robot.testfile",
      "segments {"
      "  a: 10"
      "  parts: \"seg1.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      "segments {"
      "  a: 10"
      "  parts: \"seg2.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      );
  MakeModelFile("seg1.testfile", {10, 0, 0, 0, 0, 1, 0, 0, -1});
  MakeModelFile("seg2.testfile", {20, 0, 0, 10, 0, 1, 10, 0, -1});
  MakeModelFile("env.testfile", {8, -10, -10, 8, -10, 10, 8, 10, 0});

  FclEnvironment environment("conf.testfile");

  BOOST_CHECK_EQUAL(environment.IsCollision({0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({45.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({45.0, 45.0}), false);
}

BOOST_AUTO_TEST_CASE(three_segment_collision) {
  MakeFile("conf.testfile",
      "robot_filename: \"robot.testfile\","
      "environment_filename: \"env.testfile\","
      );
  MakeFile("robot.testfile",
      "segments {"
      "  a: 10"
      "  parts: \"seg1.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      "segments {"
      "  a: 10"
      "  parts: \"seg2.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      "segments {"
      "  a: 5"
      "  parts: \"seg3.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      );
  MakeModelFile("seg1.testfile", {10, 0, 0, 0, 0, 1, 0, 0, -1});
  MakeModelFile("seg2.testfile", {20, 0, 0, 10, 0, 1, 10, 0, -1});
  MakeModelFile("seg3.testfile", {25, 0, 0, 20, 0, 1, 20, 0, -1});
  MakeModelFile("env.testfile", {8, -20, -10, 8, -20, 10, 8, 20, 0});

  FclEnvironment environment("conf.testfile");

  BOOST_CHECK_EQUAL(environment.IsCollision({0.0, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({45.0, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({45.0, 45.0, 0.0}), false);
  BOOST_CHECK_EQUAL(environment.IsCollision({45.0, 45.0, -90.0}), true);
}

BOOST_AUTO_TEST_CASE(three_segment_transformation_stacking) {
  MakeFile("conf.testfile",
      "robot_filename: \"robot.testfile\","
      "environment_filename: \"env.testfile\","
      );
  MakeFile("robot.testfile",
      "segments {"
      "  a: 10"
      "  theta: 90.0"
      "  parts: \"seg1.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      "segments {"
      "  a: 10"
      "  parts: \"seg2.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      "segments {"
      "  a: 5"
      "  parts: \"seg3.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      );
  MakeModelFile("seg1.testfile", {0, 10, 0, 0, 0, 1, 0, 0, -1});
  MakeModelFile("seg2.testfile", {0, 20, 0, 0, 10, 1, 0, 10, -1});
  MakeModelFile("seg3.testfile", {0, 25, 0, 0, 20, 1, 0, 20, -1});
  MakeModelFile("env.testfile", {8, -20, -10, 8, -20, 10, 8, 20, 0});

  FclEnvironment environment("conf.testfile");

  BOOST_CHECK_EQUAL(environment.IsCollision({-90.0, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({-45.0, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({-45.0, 45.0, 0.0}), false);
  BOOST_CHECK_EQUAL(environment.IsCollision({-45.0, 45.0, -90.0}), true);
}

BOOST_AUTO_TEST_CASE(three_segment_multiple_part_collision) {
  MakeFile("conf.testfile",
      "robot_filename: \"robot.testfile\","
      "environment_filename: \"env.testfile\","
      );
  MakeFile("robot.testfile",
      "segments {"
      "  a: 10"
      "  parts: \"seg11.testfile\""
      "  parts: \"seg12.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      "segments {"
      "  a: 10"
      "  parts: \"seg21.testfile\""
      "  parts: \"seg22.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      "segments {"
      "  a: 5"
      "  parts: \"seg31.testfile\""
      "  parts: \"seg32.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      );
  MakeModelFile("seg11.testfile", {5, 0, 0, 0, 0, 1, 0, 0, -1});
  MakeModelFile("seg12.testfile", {10, 0, 0, 5, 0, 1, 5, 0, -1});
  MakeModelFile("seg21.testfile", {15, 0, 0, 10, 0, 1, 10, 0, -1});
  MakeModelFile("seg22.testfile", {20, 0, 0, 15, 0, 1, 15, 0, -1});
  MakeModelFile("seg31.testfile", {22.5, 0, 0, 20, 0, 1, 20, 0, -1});
  MakeModelFile("seg32.testfile", {25, 0, 0, 22.5, 0, 1, 22.5, 0, -1});
  MakeModelFile("env.testfile", {8, -20, -10, 8, -20, 10, 8, 20, 0});

  FclEnvironment environment("conf.testfile");

  BOOST_CHECK_EQUAL(environment.IsCollision({0.0, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({45.0, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({45.0, 45.0, 0.0}), false);
  BOOST_CHECK_EQUAL(environment.IsCollision({45.0, 45.0, -90.0}), true);
}

BOOST_AUTO_TEST_CASE(trivial_distance) {
  MakeFile("conf.testfile",
      "robot_filename: \"robot.testfile\","
      "environment_filename: \"env.testfile\","
      "max_underestimate: 0.1"
      );
  MakeFile("robot.testfile",
      "segments {"
      "  a: 10"
      "  parts: \"seg1.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      );
  MakeModelFile("seg1.testfile", {10, 0, 0, 0, 0, 1, 0, 0, -1});
  MakeModelFile("env.testfile", {15, -10, -10, 15, -10, 10, 15, 10, 0});

  FclEnvironment environment("conf.testfile");

  EnvironmentInterface::DistanceProfile d;

  d = environment.GetDistanceProfile({0.0});
  BOOST_CHECK_EQUAL(d.size(), 1);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 5.0, AbsToRelTolerance(5.0, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 11.0, 1.0);

  d = environment.GetDistanceProfile({45.0});
  BOOST_CHECK_EQUAL(d.size(), 1);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 7.929, AbsToRelTolerance(7.929, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 11.0, 1.0);
}

BOOST_AUTO_TEST_CASE(trivial_distance_two_part) {
  MakeFile("conf.testfile",
      "robot_filename: \"robot.testfile\","
      "environment_filename: \"env.testfile\","
      "max_underestimate: 0.1"
      );
  MakeFile("robot.testfile",
      "segments {"
      "  a: 10"
      "  parts: \"seg11.testfile\""
      "  parts: \"seg12.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      );
  MakeModelFile("seg11.testfile", {5, 0, 0, 0, 0, 1, 0, 0, -1});
  MakeModelFile("seg12.testfile", {10, 0, 0, 5, 0, 1, 5, 0, -1});
  MakeModelFile("env.testfile", {15, -10, -10, 15, -10, 10, 15, 10, 0});

  FclEnvironment environment("conf.testfile");

  EnvironmentInterface::DistanceProfile d;

  d = environment.GetDistanceProfile({0.0});
  BOOST_CHECK_EQUAL(d.size(), 2);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_EQUAL(d[1].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 10.0, AbsToRelTolerance(10.0, 0.11));
  BOOST_CHECK_CLOSE(d[1].first, 5.0, AbsToRelTolerance(5.0, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 6.0, 1.0);
  BOOST_CHECK_CLOSE(d[1].second[0], 11.0, 1.0);

  d = environment.GetDistanceProfile({45.0});
  BOOST_CHECK_EQUAL(d.size(), 2);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_EQUAL(d[1].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 11.4645, AbsToRelTolerance(11.4645, 0.11));
  BOOST_CHECK_CLOSE(d[1].first, 7.929, AbsToRelTolerance(7.929, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 6.0, 1.0);
  BOOST_CHECK_CLOSE(d[1].second[0], 11.0, 1.0);
}

BOOST_AUTO_TEST_CASE(two_segment_distance) {
  MakeFile("conf.testfile",
      "robot_filename: \"robot.testfile\","
      "environment_filename: \"env.testfile\","
      "max_underestimate: 0.1"
      );
  MakeFile("robot.testfile",
      "segments {"
      "  a: 10"
      "  parts: \"seg1.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      "segments {"
      "  a: 10"
      "  parts: \"seg2.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      );
  MakeModelFile("seg1.testfile", {10, 0, 0, 0, 0, 1, 0, 0, -1});
  MakeModelFile("seg2.testfile", {20, 0, 0, 10, 0, 1, 10, 0, -1});
  MakeModelFile("env.testfile", {25, -20, -10, 25, -20, 10, 25, 20, 0});

  FclEnvironment environment("conf.testfile");

  EnvironmentInterface::DistanceProfile d;

  d = environment.GetDistanceProfile({0.0, 0.0});
  BOOST_CHECK_EQUAL(d.size(), 2);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 15.0, AbsToRelTolerance(15.0, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 11.0, 1.0);
  BOOST_CHECK_EQUAL(d[1].second.size(), 2);
  BOOST_CHECK_CLOSE(d[1].first, 5.0, AbsToRelTolerance(5.0, 0.11));
  BOOST_CHECK_CLOSE(d[1].second[0], 21.0, 1.0);
  BOOST_CHECK_CLOSE(d[1].second[1], 11.0, 1.0);

  d = environment.GetDistanceProfile({45.0, 0.0});
  BOOST_CHECK_EQUAL(d.size(), 2);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 17.929, AbsToRelTolerance(17.929, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 11.0, 1.0);
  BOOST_CHECK_EQUAL(d[1].second.size(), 2);
  BOOST_CHECK_CLOSE(d[1].first, 10.858, AbsToRelTolerance(10.858, 0.11));
  BOOST_CHECK_CLOSE(d[1].second[0], 21.0, 1.0);
  BOOST_CHECK_CLOSE(d[1].second[1], 11.0, 1.0);

  d = environment.GetDistanceProfile({45.0, 45.0});
  BOOST_CHECK_EQUAL(d.size(), 2);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 17.929, AbsToRelTolerance(17.929, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 11.0, 1.0);
  BOOST_CHECK_EQUAL(d[1].second.size(), 2);
  BOOST_CHECK_CLOSE(d[1].first, 17.929, AbsToRelTolerance(17.929, 0.11));
  BOOST_CHECK_CLOSE(d[1].second[0], 19.4776, 1.0);
  BOOST_CHECK_CLOSE(d[1].second[1], 11.0, 1.0);

  d = environment.GetDistanceProfile({0.0, 135.0});
  BOOST_CHECK_EQUAL(d.size(), 2);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 15.0, AbsToRelTolerance(15.0, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 11.0, 1.0);
  BOOST_CHECK_EQUAL(d[1].second.size(), 2);
  BOOST_CHECK_CLOSE(d[1].first, 15.0, AbsToRelTolerance(5.0, 0.11));
  BOOST_CHECK_CLOSE(d[1].second[0], 11.0, 1.0);
  BOOST_CHECK_CLOSE(d[1].second[1], 11.0, 1.0);
}

BOOST_AUTO_TEST_CASE(three_segment_3d_distance) {
  MakeFile("conf.testfile",
      "robot_filename: \"robot.testfile\","
      "environment_filename: \"env.testfile\","
      "max_underestimate: 0.1"
      );
  MakeFile("robot.testfile",
      "segments {"
      "  a: 10"
      "  parts: \"seg1.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      "segments {"
      "  a: 10"
      "  alpha: 90.0"
      "  parts: \"seg2.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      "segments {"
      "  a: 10"
      "  theta: 90.0"
      "  parts: \"seg3.testfile\""
      "  range {"
      "    min: -180"
      "    max: 180"
      "  }"
      "}"
      );
  MakeModelFile("seg1.testfile", {10, 0, 0, 0, 0, 1, 0, 0, -1});
  MakeModelFile("seg2.testfile", {20, 0, 0, 10, 0, 1, 10, 0, -1});
  MakeModelFile("seg3.testfile", {20, 0, 10, 20, 1, 0, 20, -1, 0});
  MakeModelFile("env.testfile", {35, -20, -10, 35, -20, 10, 35, 20, 0});

  FclEnvironment environment("conf.testfile");

  EnvironmentInterface::DistanceProfile d;

  d = environment.GetDistanceProfile({0.0, 0.0, 0.0});
  BOOST_CHECK_EQUAL(d.size(), 3);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 25.0, AbsToRelTolerance(25.0, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 11.0, 1.0);
  BOOST_CHECK_EQUAL(d[1].second.size(), 2);
  BOOST_CHECK_CLOSE(d[1].first, 15.0, AbsToRelTolerance(15.0, 0.11));
  BOOST_CHECK_CLOSE(d[1].second[0], 21.0, 1.0);
  BOOST_CHECK_CLOSE(d[1].second[1], 11.0, 1.0);
  BOOST_CHECK_EQUAL(d[2].second.size(), 3);
  BOOST_CHECK_CLOSE(d[2].first, 15.0, AbsToRelTolerance(15.0, 0.11));
  BOOST_CHECK_CLOSE(d[2].second[0], 21.0, 1.0);
  BOOST_CHECK_CLOSE(d[2].second[1], 11.0, 1.0);
  BOOST_CHECK_CLOSE(d[2].second[2], 11.0, 1.0);

  d = environment.GetDistanceProfile({0.0, 0.0, -90.0});
  BOOST_CHECK_EQUAL(d.size(), 3);
  BOOST_CHECK_EQUAL(d[0].second.size(), 1);
  BOOST_CHECK_CLOSE(d[0].first, 25.0, AbsToRelTolerance(25.0, 0.11));
  BOOST_CHECK_CLOSE(d[0].second[0], 11.0, 1.0);
  BOOST_CHECK_EQUAL(d[1].second.size(), 2);
  BOOST_CHECK_CLOSE(d[1].first, 15.0, AbsToRelTolerance(15.0, 0.11));
  BOOST_CHECK_CLOSE(d[1].second[0], 21.0, 1.0);
  BOOST_CHECK_CLOSE(d[1].second[1], 11.0, 1.0);
  BOOST_CHECK_EQUAL(d[2].second.size(), 3);
  BOOST_CHECK_CLOSE(d[2].first, 5.0, AbsToRelTolerance(5.0, 0.11));
  BOOST_CHECK_CLOSE(d[2].second[0], 31.0, 1.0);
  BOOST_CHECK_CLOSE(d[2].second[1], 21.0, 1.0);
  BOOST_CHECK_CLOSE(d[2].second[2], 11.0, 1.0);
}
