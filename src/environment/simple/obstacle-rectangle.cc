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

#include "obstacle-rectangle.h"

#include <cmath>

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace environment {
namespace simple {
namespace {

bool LinesCollide(double l1_x1, double l1_y1, double l1_x2, double l1_y2,
                  double l2_x1, double l2_y1, double l2_x2, double l2_y2) {
  double denominator =
      (l1_x1 - l1_x2) * (l2_y1 - l2_y2) - (l1_y1 - l1_y2) * (l2_x1 - l2_x2);
  if ((denominator == 0.0) || (denominator == -0.0))
    return false;
  double x = ((l1_x1 * l1_y2 - l1_y1 * l1_x2) * (l2_x1 - l2_x2) -
      (l1_x1 - l1_x2) * (l2_x1 * l2_y2 - l2_y1 * l2_x2)) / denominator;
  double y = ((l1_x1 * l1_y2 - l1_y1 * l1_x2) * (l2_y1 - l2_y2) -
      (l1_y1 - l1_y2) * (l2_x1 * l2_y2 - l2_y1 * l2_x2)) / denominator;
  if (l1_x1 >= l1_x2) {
    if (!(l1_x2 <= x && x <= l1_x1))
      return false;
  } else {
    if (!(l1_x1 <= x && x <= l1_x2))
      return false;
  }
  if (l1_y1 >= l1_y2) {
    if (!(l1_y2 <= y && y <= l1_y1))
      return false;
  } else {
    if (!(l1_y1 <= y && y <= l1_y2))
      return false;
  }
  if (l2_x1 >= l2_x2) {
    if (!(l2_x2 <= x && x <= l2_x1))
      return false;
  } else {
    if (!(l2_x1 <= x && x <= l2_x2))
      return false;
  }
  if (l2_y1 >= l2_y2) {
    if (!(l2_y2 <= y && y <= l2_y1))
      return false;
  } else {
    if (!(l2_y1 <= y && y <= l2_y2))
      return false;
  }
  return true;
}

inline double Dot(double x1, double y1, double x2, double y2) {
  return (x1 * x2 + y1 * y2);
}

inline double NormSquared(double x, double y) {
  return Dot(x, y, x, y);
}

double DistanceToLineHelper(double point_x, double point_y,
                            double line_x1, double line_y1,
                            double line_x2, double line_y2) {
  double point_a_distance(NormSquared(point_x - line_x1, point_y - line_y1));
  double point_b_distance(NormSquared(point_x - line_x2, point_y - line_y2));
  double ux = line_x1 - line_x2;
  double uy = line_y1 - line_y2;
  double normalizer = sqrt(NormSquared(ux, uy));
  if (normalizer != 0.0 && normalizer != -0.0) {
    ux /= normalizer;
    uy /= normalizer;
  }
  if (point_a_distance < point_b_distance) {
    double dx = point_x - line_x1;
    double dy = point_y - line_y1;
    normalizer = sqrt(NormSquared(dx, dy));
    if (normalizer != 0.0 && normalizer != -0.0) {
      dx /= normalizer;
      dy /= normalizer;
    }
    normalizer = Dot(dx, dy, -ux, -uy);
    return (normalizer > 0.0)
        ? sqrt(point_a_distance * (1.0 - normalizer * normalizer))
        : sqrt(point_a_distance);
  } else {
    double dx = point_x - line_x2;
    double dy = point_y - line_y2;
    normalizer = sqrt(NormSquared(dx, dy));
    if (normalizer != 0.0 && normalizer != -0.0) {
      dx /= normalizer;
      dy /= normalizer;
    }
    normalizer = Dot(dx, dy, ux, uy);
    return (normalizer > 0.0)
        ? sqrt(point_b_distance * (1.0 - normalizer * normalizer))
        : sqrt(point_b_distance);
  }
}

double DistanceToLineReducer(
    double current_distance, double point_x, double point_y,
    double line_x1, double line_y1, double line_x2, double line_y2) {
  double new_distance = DistanceToLineHelper(
      point_x, point_y, line_x1, line_y1, line_x2, line_y2);
  return (new_distance < current_distance)
      ? new_distance
      : current_distance;
}

}  // namespace

ObstacleRectangle::ObstacleRectangle(double x_low, double y_low,
                                     double x_high, double y_high)
    : x_low_(x_low), y_low_(y_low), x_high_(x_high), y_high_(y_high) {}

double ObstacleRectangle::DistanceToLine(
    double x1, double y1, double z1, double x2, double y2, double z2) const {
  if (LinesCollide(x_high_, y_high_, x_low_, y_high_, x1, y1, x2, y2))
    return 0.0;
  if (LinesCollide(x_high_, y_low_, x_low_, y_low_, x1, y1, x2, y2))
    return 0.0;
  if (LinesCollide(x_high_, y_high_, x_high_, y_low_, x1, y1, x2, y2))
    return 0.0;
  if (LinesCollide(x_low_, y_high_, x_low_, y_low_, x1, y1, x2, y2))
    return 0.0;

  double dist = DistanceToLineHelper(x_high_, y_high_, x1, y1, x2, y2);
  dist = DistanceToLineReducer(dist, x_high_, y_low_, x1, y1, x2, y2);
  dist = DistanceToLineReducer(dist, x_low_, y_high_, x1, y1, x2, y2);
  dist = DistanceToLineReducer(dist, x_low_, y_low_, x1, y1, x2, y2);

  dist = DistanceToLineReducer(dist, x1, y1, x_high_, y_high_, x_high_, y_low_);
  dist = DistanceToLineReducer(dist, x1, y1, x_low_, y_high_, x_low_, y_low_);
  dist = DistanceToLineReducer(dist, x1, y1, x_high_, y_high_, x_low_, y_high_);
  dist = DistanceToLineReducer(dist, x1, y1, x_high_, y_low_, x_low_, y_low_);

  dist = DistanceToLineReducer(dist, x2, y2, x_high_, y_high_, x_high_, y_low_);
  dist = DistanceToLineReducer(dist, x2, y2, x_low_, y_high_, x_low_, y_low_);
  dist = DistanceToLineReducer(dist, x2, y2, x_high_, y_high_, x_low_, y_high_);
  dist = DistanceToLineReducer(dist, x2, y2, x_high_, y_low_, x_low_, y_low_);

  return dist;
}

}  // namespace simple
}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
