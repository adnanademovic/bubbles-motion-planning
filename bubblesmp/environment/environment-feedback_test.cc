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
#define BOOST_TEST_MODULE EnvironmentFeedback
#include "bubblesmp/bubble.h"
#include "bubblesmp/environment/environment-feedback.h"
#include "bubblesmp/environment/make-environment.h"
#include <boost/test/unit_test.hpp>

#include <cmath>
#include <memory>
#include <vector>

constexpr double rad_to_deg() {
  return 45.0 / atan(1);
}

using ::com::ademovic::bubblesmp::Bubble;
using ::com::ademovic::bubblesmp::environment::NewEnvironmentFromProtoBuffer;
using ::com::ademovic::bubblesmp::environment::EnvironmentFeedback;

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

  EnvironmentFeedback environment(
      NewEnvironmentFromProtoBuffer("conf.testfile"));

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

  EnvironmentFeedback environment(
      NewEnvironmentFromProtoBuffer("conf.testfile"));

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

  EnvironmentFeedback environment(
      NewEnvironmentFromProtoBuffer("conf.testfile"));

  BOOST_CHECK_EQUAL(environment.IsCollision({0.0, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({45.0, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({45.0, 45.0, 0.0}), false);
  BOOST_CHECK_EQUAL(environment.IsCollision({45.0, 45.0, -90.0}), true);
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

  EnvironmentFeedback environment(
      NewEnvironmentFromProtoBuffer("conf.testfile"));

  BOOST_CHECK_EQUAL(environment.IsCollision({0.0, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({45.0, 0.0, 0.0}), true);
  BOOST_CHECK_EQUAL(environment.IsCollision({45.0, 45.0, 0.0}), false);
  BOOST_CHECK_EQUAL(environment.IsCollision({45.0, 45.0, -90.0}), true);
}

BOOST_AUTO_TEST_CASE(trivial_bubble) {
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

  EnvironmentFeedback environment(
      NewEnvironmentFromProtoBuffer("conf.testfile"));

  std::unique_ptr<Bubble> bub;
  std::vector<double> dims;

  bub.reset(environment.NewBubble({0.0}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 1);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 5.0 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 5.0 / 11.0, 0.11));

  bub.reset(environment.NewBubble({45.0}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 1);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 7.929 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 7.929 / 11.0, 0.11));
}

BOOST_AUTO_TEST_CASE(trivial_bubble_two_part) {
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

  EnvironmentFeedback environment(
      NewEnvironmentFromProtoBuffer("conf.testfile"));

  std::unique_ptr<Bubble> bub;
  std::vector<double> dims;

  bub.reset(environment.NewBubble({0.0}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 1);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 5.0 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 5.0 / 11.0, 0.11));

  bub.reset(environment.NewBubble({45.0}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 1);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 7.929 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 7.929 / 11.0, 0.11));
}

BOOST_AUTO_TEST_CASE(two_segment_bubble) {
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

  EnvironmentFeedback environment(
      NewEnvironmentFromProtoBuffer("conf.testfile"));

  std::unique_ptr<Bubble> bub;
  std::vector<double> dims;

  bub.reset(environment.NewBubble({0.0, 0.0}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 5.0 / 21.0,
                    AbsToRelTolerance(rad_to_deg() * 5.0 / 21.0, 0.11));
  BOOST_CHECK_CLOSE(dims[1], rad_to_deg() * 5.0 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 5.0 / 11.0, 0.11));

  bub.reset(environment.NewBubble({45.0, 0.0}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 10.858 / 21.0,
                    AbsToRelTolerance(rad_to_deg() * 10.858 / 21.0, 0.11));
  BOOST_CHECK_CLOSE(dims[1], rad_to_deg() * 10.858 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 10.858 / 11.0, 0.11));

  bub.reset(environment.NewBubble({45.0, 45.0}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 17.929 / 19.4776,
                    AbsToRelTolerance(rad_to_deg() * 17.929 / 19.4776, 0.11));
  BOOST_CHECK_CLOSE(dims[1], rad_to_deg() * 17.929 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 17.929 / 11.0, 0.11));

  bub.reset(environment.NewBubble({0.0, 135.0}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 15.0 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 15.0 / 11.0, 0.11));
  BOOST_CHECK_CLOSE(dims[1], rad_to_deg() * 15.0 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 15.0 / 11.0, 0.11));

  bub.reset(environment.NewBubble({180.0, 0.0}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 35.0 / 21.0,
                    AbsToRelTolerance(rad_to_deg() * 35.0 / 21.0, 0.11));
  BOOST_CHECK_CLOSE(dims[1], rad_to_deg() * 35.0 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 35.0 / 11.0, 0.11));
}

BOOST_AUTO_TEST_CASE(trivial_bubble_unextended) {
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

  EnvironmentFeedback environment(
      NewEnvironmentFromProtoBuffer("conf.testfile"));

  std::unique_ptr<Bubble> bub;
  std::vector<double> dims;

  bub.reset(environment.NewBubble({0.0}, false));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 1);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 5.0 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 5.0 / 11.0, 0.11));

  bub.reset(environment.NewBubble({45.0}, false));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 1);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 7.929 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 7.929 / 11.0, 0.11));
}

BOOST_AUTO_TEST_CASE(two_segment_bubble_unextended) {
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

  EnvironmentFeedback environment(
      NewEnvironmentFromProtoBuffer("conf.testfile"));

  std::unique_ptr<Bubble> bub;
  std::vector<double> dims;

  bub.reset(environment.NewBubble({0.0, 0.0}, false));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 5.0 / 21.0,
                    AbsToRelTolerance(rad_to_deg() * 5.0 / 21.0, 0.11));
  BOOST_CHECK_CLOSE(dims[1], rad_to_deg() * 5.0 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 5.0 / 11.0, 0.11));

  bub.reset(environment.NewBubble({45.0, 0.0}, false));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 10.858 / 21.0,
                    AbsToRelTolerance(rad_to_deg() * 10.858 / 21.0, 0.11));
  BOOST_CHECK_CLOSE(dims[1], rad_to_deg() * 10.858 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 10.858 / 11.0, 0.11));

  bub.reset(environment.NewBubble({45.0, 45.0}, false));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 17.929 / 19.4776,
                    AbsToRelTolerance(rad_to_deg() * 17.929 / 19.4776, 0.11));
  BOOST_CHECK_CLOSE(dims[1], rad_to_deg() * 17.929 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 17.929 / 11.0, 0.11));

  bub.reset(environment.NewBubble({0.0, 135.0}, false));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 15.0 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 15.0 / 11.0, 0.11));
  BOOST_CHECK_CLOSE(dims[1], rad_to_deg() * 15.0 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 15.0 / 11.0, 0.11));

  bub.reset(environment.NewBubble({180.0, 0.0}, false));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 25.0 / 21.0,
                    AbsToRelTolerance(rad_to_deg() * 25.0 / 21.0, 0.11));
  BOOST_CHECK_CLOSE(dims[1], rad_to_deg() * 25.0 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 25.0 / 11.0, 0.11));
}

BOOST_AUTO_TEST_CASE(two_segment_bubble_extension_difference) {
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
  MakeModelFile("env.testfile", {15, -20, -10, 15, -20, 10, 15, 20, 0});

  EnvironmentFeedback environment(
      NewEnvironmentFromProtoBuffer("conf.testfile"));

  std::unique_ptr<Bubble> bub;
  std::vector<double> dims;

  bub.reset(environment.NewBubble({180.0, 0.0}));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 25.0 / 21.0,
                    AbsToRelTolerance(rad_to_deg() * 25.0 / 21.0, 0.11));
  BOOST_CHECK_CLOSE(dims[1], rad_to_deg() * 25.0 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 25.0 / 11.0, 0.11));

  bub.reset(environment.NewBubble({180.0, 0.0}, false));
  dims = bub->size();
  BOOST_CHECK_EQUAL(dims.size(), 2);
  BOOST_CHECK_CLOSE(dims[0], rad_to_deg() * 15.0 / 21.0,
                    AbsToRelTolerance(rad_to_deg() * 15.0 / 21.0, 0.11));
  BOOST_CHECK_CLOSE(dims[1], rad_to_deg() * 15.0 / 11.0,
                    AbsToRelTolerance(rad_to_deg() * 15.0 / 11.0, 0.11));
}
