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

#include "obstacle-sphere.h"

#include <cmath>

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace environment {
namespace simple {
namespace {

inline double Dot(double x1, double y1, double z1,
    double x2, double y2, double z2) {
  return (x1 * x2 + y1 * y2 + z1 * z2);
}

inline double NormSquared(double x, double y, double z) {
  return Dot(x, y, z, x, y, z);
}

double CenterDistanceToLine(double pos_x, double pos_y, double pos_z,
                            double x1, double y1, double z1,
                            double x2, double y2, double z2) {
  double point_a_distance(NormSquared(pos_x - x1, pos_y - y1, pos_z - z1));
  double point_b_distance(NormSquared(pos_x - x2, pos_y - y2, pos_z - z2));
  double ux = x1 - x2;
  double uy = y1 - y2;
  double uz = z1 - z2;
  double normalizer = sqrt(NormSquared(ux, uy, uz));
  if (normalizer != 0.0 && normalizer != -0.0) {
    ux /= normalizer;
    uy /= normalizer;
    uz /= normalizer;
  }
  if (point_a_distance < point_b_distance) {
    double dx = pos_x - x1;
    double dy = pos_y - y1;
    double dz = pos_z - z1;
    normalizer = sqrt(NormSquared(dx, dy, dz));
    if (normalizer != 0.0 && normalizer != -0.0) {
      dx /= normalizer;
      dy /= normalizer;
      dz /= normalizer;
    }
    normalizer = Dot(dx, dy, dz, -ux, -uy, -uz);
    return (normalizer > 0.0)
        ? sqrt(point_a_distance * (1.0 - normalizer * normalizer))
        : sqrt(point_a_distance);
  } else {
    double dx = pos_x - x2;
    double dy = pos_y - y2;
    double dz = pos_z - z2;
    normalizer = sqrt(NormSquared(dx, dy, dz));
    if (normalizer != 0.0 && normalizer != -0.0) {
      dx /= normalizer;
      dy /= normalizer;
      dz /= normalizer;
    }
    normalizer = Dot(dx, dy, dz, ux, uy, uz);
    return (normalizer > 0.0)
        ? sqrt(point_b_distance * (1.0 - normalizer * normalizer))
        : sqrt(point_b_distance);
  }
}

}  // namespace

ObstacleSphere::ObstacleSphere(
    double pos_x, double pos_y, double pos_z, double radius)
  : pos_x_(pos_x), pos_y_(pos_y), pos_z_(pos_z), radius_(radius) {}

double ObstacleSphere::DistanceToLine(
    double x1, double y1, double z1, double x2, double y2, double z2) const {
  double distance = CenterDistanceToLine(
      pos_x_, pos_y_, pos_z_, x1, y1, z1, x2, y2, z2) - radius_;
  return (distance > 0.0)
      ? distance
      : 0.0;
}

}  // namespace simple
}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
