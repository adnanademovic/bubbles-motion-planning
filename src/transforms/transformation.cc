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

#include "transformation.h"

#include <cmath>
#include <cstring>

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace transforms {

Transformation::Transformation() {
  memset(rotation_, 0, sizeof(rotation_));
  memset(translation_, 0, sizeof(translation_));
  rotation_[0][0] = rotation_[1][1] = rotation_[2][2] = 1.0;
}

void Transformation::Rotate(Axis axis, double angle) {
  Transformation transform;
  int n1 = (axis + 1) % 3;
  int n2 = (axis + 2) % 3;
  transform.rotation_[n1][n1] = cos(angle);
  transform.rotation_[n2][n2] = cos(angle);
  transform.rotation_[n1][n2] = -sin(angle);
  transform.rotation_[n2][n1] = sin(angle);
  Transform(transform);
}

void Transformation::Translate(double x, double y, double z) {
  Transformation transform;
  transform.translation_[0] = x;
  transform.translation_[1] = y;
  transform.translation_[2] = z;
  Transform(transform);
}

void Transformation::Transform(const Transformation& transform) {
  double rotation[3][3];
  double translation[3];
  for (int i = 0; i != 3; ++i)
    for (int j = 0; j != 3; ++j)
      rotation[i][j] = rotation_[i][j];
  for (int i = 0; i != 3; ++i)
    translation[i] = translation_[i];

  transform.ApplyTo(rotation, translation);

  std::lock_guard<std::mutex> matrices_guard(matrices_mutex_);
  for (int i = 0; i != 3; ++i)
    for (int j = 0; j != 3; ++j)
      rotation_[i][j] = rotation[i][j];
  for (int i = 0; i != 3; ++i)
    translation_[i] = translation[i];
}

double Transformation::coefficient(int i, int j) const {
  std::lock_guard<std::mutex> matrices_guard(matrices_mutex_);
  return (i==3)
      ? ((j==3) ? 1.0 : 0.0)
      : ((j==3) ? translation_[i] : rotation_[i][j]);
}

void Transformation::ApplyTo(
    double rotation[3][3], double translation[3]) const {
  std::lock_guard<std::mutex> matrices_guard(matrices_mutex_);

  for (int i = 0; i != 3; ++i)
    for (int j = 0; j != 3; ++j)
      translation[i] += rotation[i][j] * translation_[j];

  double result[3][3];
  memset(result, 0, sizeof(result));
  for (int i = 0; i != 3; ++i)
    for (int j = 0; j != 3; ++j)
      for (int k = 0; k != 3; ++k)
        result[i][j] += rotation[i][k] * rotation_[k][j];

  for (int i = 0; i != 3; ++i)
    for (int j = 0; j != 3; ++j)
      rotation[i][j] = result[i][j];
}

}  // namespace transforms
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
