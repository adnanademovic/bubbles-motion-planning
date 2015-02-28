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
    : using_flann_index_(using_flann_index),
      attachment_points_(1, AttachmentPoint{q_root, root_node}),
      root_node_(root_node), index_(flann::Matrix<double>(
                 std::vector<double>(q_root).data(), 1, q_root.size()),
             flann::KDTreeIndexParams(8)) {
  if (using_flann_index_)
    index_.buildIndex();
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
        indices, distances, 1, flann::SearchParams(128));
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
