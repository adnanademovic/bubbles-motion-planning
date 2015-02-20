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
#define BOOST_TEST_MODULE Transformation
#include "transformation.h"

#include <cmath>

#include <boost/test/unit_test.hpp>

using ::com::ademovic::bubblesmp::transforms::Transformation;

void CheckEquality(const Transformation& transform, const double t[4][4]) {
  for (int i = 0; i != 4; ++i)
    for (int j = 0; j != 4; ++j)
      BOOST_CHECK_EQUAL(transform.coefficient(i, j), t[i][j]);
}

void CheckClose(
    const Transformation& transform, const double t[4][4], double error) {
  for (int i = 0; i != 4; ++i)
    for (int j = 0; j != 4; ++j)
      BOOST_CHECK_CLOSE(transform.coefficient(i, j), t[i][j], error);
}

BOOST_AUTO_TEST_CASE(constructor_is_identity) {
  Transformation transform;
  double matrix[4][4] = {{1.0, 0.0, 0.0, 0.0},
                         {0.0, 1.0, 0.0, 0.0},
                         {0.0, 0.0, 1.0, 0.0},
                         {0.0, 0.0, 0.0, 1.0}};
  CheckEquality(transform, matrix);
}

BOOST_AUTO_TEST_CASE(translation) {
  Transformation transform;
  transform.Translate(2.0, 2.5, 3.0);
  double matrix[4][4] = {{1.0, 0.0, 0.0, 2.0},
                         {0.0, 1.0, 0.0, 2.5},
                         {0.0, 0.0, 1.0, 3.0},
                         {0.0, 0.0, 0.0, 1.0}};
  CheckEquality(transform, matrix);
}

BOOST_AUTO_TEST_CASE(rotation_x) {
  double angle = M_PI/6.0;
  Transformation transform;
  transform.Rotate(Transformation::X, angle);
  double matrix[4][4] = {{1.0,             0.0,             0.0, 0.0},
                         {0.0, std::sqrt(0.75),            -0.5, 0.0},
                         {0.0,             0.5, std::sqrt(0.75), 0.0},
                         {0.0,             0.0,             0.0, 1.0}};
  CheckClose(transform, matrix, 0.001);
}

BOOST_AUTO_TEST_CASE(rotation_y) {
  double angle = M_PI/3.0;
  Transformation transform;
  transform.Rotate(Transformation::Y, angle);
  double matrix[4][4] = {{             0.5, 0.0, std::sqrt(0.75), 0.0},
                         {             0.0, 1.0,             0.0, 0.0},
                         {-std::sqrt(0.75), 0.0,             0.5, 0.0},
                         {             0.0, 0.0,             0.0, 1.0}};
  CheckClose(transform, matrix, 0.001);
}

BOOST_AUTO_TEST_CASE(rotation_z) {
  double angle = M_PI/4.0;
  Transformation transform;
  transform.Rotate(Transformation::Z, angle);
  double matrix[4][4] = {{std::sqrt(0.5), -std::sqrt(0.5), 0.0, 0.0},
                         {std::sqrt(0.5),  std::sqrt(0.5), 0.0, 0.0},
                         {           0.0,             0.0, 1.0, 0.0},
                         {           0.0,             0.0, 0.0, 1.0}};
  CheckClose(transform, matrix, 0.001);
}

BOOST_AUTO_TEST_CASE(composition) {
  double angle = M_PI/4.0;
  Transformation transform1;
  transform1.Translate(4.0, 0.0, 0.0);
  double matrix1[4][4] = {{1.0, 0.0, 0.0, 4.0},
                          {0.0, 1.0, 0.0, 0.0},
                          {0.0, 0.0, 1.0, 0.0},
                          {0.0, 0.0, 0.0, 1.0}};
  CheckEquality(transform1, matrix1);
  Transformation transform2;
  transform2.Rotate(Transformation::Z, angle);
  transform2.Translate(0.0, std::sqrt(2.0), 3.0);
  transform1.Transform(transform2);
  double matrix2[4][4] = {{std::sqrt(0.5), -std::sqrt(0.5), 0.0, -1.0},
                          {std::sqrt(0.5),  std::sqrt(0.5), 0.0,  1.0},
                          {           0.0,             0.0, 1.0,  3.0},
                          {           0.0,             0.0, 0.0,  1.0}};
  double matrix3[4][4] = {{std::sqrt(0.5), -std::sqrt(0.5), 0.0, 3.0},
                          {std::sqrt(0.5),  std::sqrt(0.5), 0.0, 1.0},
                          {           0.0,             0.0, 1.0, 3.0},
                          {           0.0,             0.0, 0.0, 1.0}};
  CheckClose(transform2, matrix2, 0.001);
  CheckClose(transform1, matrix3, 0.001);
}

BOOST_AUTO_TEST_CASE(point_moving) {
  double angle = M_PI/4.0;
  Transformation transform1;
  transform1.Translate(4.0, 0.0, 0.0);
  double matrix1[4][4] = {{1.0, 0.0, 0.0, 4.0},
                          {0.0, 1.0, 0.0, 0.0},
                          {0.0, 0.0, 1.0, 0.0},
                          {0.0, 0.0, 0.0, 1.0}};
  CheckEquality(transform1, matrix1);
  Transformation transform2;
  transform2.Rotate(Transformation::Z, angle);
  transform2.Translate(0.0, std::sqrt(2.0), 3.0);
  transform1.Transform(transform2);
  double matrix2[4][4] = {{std::sqrt(0.5), -std::sqrt(0.5), 0.0, -1.0},
                          {std::sqrt(0.5),  std::sqrt(0.5), 0.0,  1.0},
                          {           0.0,             0.0, 1.0,  3.0},
                          {           0.0,             0.0, 0.0,  1.0}};
  double matrix3[4][4] = {{std::sqrt(0.5), -std::sqrt(0.5), 0.0, 3.0},
                          {std::sqrt(0.5),  std::sqrt(0.5), 0.0, 1.0},
                          {           0.0,             0.0, 1.0, 3.0},
                          {           0.0,             0.0, 0.0, 1.0}};
  CheckClose(transform2, matrix2, 0.001);
  CheckClose(transform1, matrix3, 0.001);

  double p[] = {2.0, 4.0, 6.0};
  transform2.MovePoint(p);
  BOOST_CHECK_CLOSE(p[0], -std::sqrt(2.0) - 1.0, 0.001);
  BOOST_CHECK_CLOSE(p[1], 3.0 * std::sqrt(2.0) + 1.0, 0.001);
  BOOST_CHECK_CLOSE(p[2], 9.0, 0.001);
}
