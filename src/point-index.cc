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

#include "point-index.h"

#include <gflags/gflags.h>

DEFINE_bool(point_index_force_flann, false,
    "Sets if point indices should have to use flann");
DEFINE_bool(point_index_force_simple, false,
    "Sets if point indices should have to use a simple index "
    "(faster for small amounts of points)");
DEFINE_int32(point_index_tree_count, 8,
    "Number of trees used for the point indices");
DEFINE_int32(point_index_checks, 128,
    "Number of trees used for the point indices");

namespace {
static bool ValidatePointIndexParams(const char* flagname, int32_t value) {
  if (value < 1) {
    printf("Invalid value for --%s: %d\n", flagname, value);
    return false;
  }
  return true;
}

static const bool tree_count_dummy = google::RegisterFlagValidator(
    &FLAGS_point_index_tree_count, &ValidatePointIndexParams);
static const bool checks_dummy = google::RegisterFlagValidator(
    &FLAGS_point_index_checks, &ValidatePointIndexParams);
}  // namespace

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace {
double DistanceSquared(
    const std::vector<double>& first, const std::vector<double>& second) {
  double d = 0.0;
  double c = 0.0;
  size_t l = first.size();
  for (size_t i = 0; i < l; ++i) {
    c = second[i] - first[i];
    d += c * c;
  }
  return d;
}
}  // namespace

PointIndex::PointIndex(const std::vector<double>& q_root, TreeNode* root_node,
                       bool using_flann_index)
    : using_flann_index_(FLAGS_point_index_force_flann ||
          (using_flann_index && !FLAGS_point_index_force_simple)),
      attachment_points_(1, AttachmentPoint{q_root, root_node}),
      root_node_(root_node), index_(flann::Matrix<double>(
                 std::vector<double>(q_root).data(), 1, q_root.size()),
             flann::KDTreeIndexParams(FLAGS_point_index_tree_count)) {
  if ((using_flann_index_ && !FLAGS_point_index_force_simple) ||
      FLAGS_point_index_force_flann)
    index_.buildIndex();
  search_parameters_.checks = FLAGS_point_index_checks;
  search_parameters_.use_heap = flann::FLANN_False;
}

void PointIndex::AddPoint(const std::vector<double>& q, TreeNode* parent) {
  attachment_points_.push_back(AttachmentPoint{q, parent});
  if (using_flann_index_)
    index_.addPoints(
        flann::Matrix<double>(std::vector<double>(q).data(), 1, q.size()));
}

AttachmentPoint PointIndex::GetNearestPoint(
    const std::vector<double>& q) const {
  if (using_flann_index_) {
    std::vector<std::vector<int> > indices(1, std::vector<int>(1, 0));
    std::vector<std::vector<double> > distances(1, std::vector<double>(1, 0.0));
    index_.knnSearch(
        flann::Matrix<double>(std::vector<double>(q).data(), 1, q.size()),
        indices, distances, 1, search_parameters_);
    return attachment_points_[indices[0][0]];
  } else {
    double closest_distance = DistanceSquared(
        attachment_points_[0].position, q);
    int closest_point = 0;
    double dist_now;
    for (size_t i = attachment_points_.size() - 1; i > 0; --i) {
      dist_now = DistanceSquared(attachment_points_[i].position, q);
      if (dist_now < closest_distance) {
        closest_point = i;
        closest_distance = dist_now;
      }
    }
    return attachment_points_[closest_point];
  }
}

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
