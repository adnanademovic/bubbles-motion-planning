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

#include "bubble-tree.h"

#include <cmath>

namespace com {
namespace ademovic {
namespace bubblesmp {

double BubbleTree::AttachmentPoint::Distance(
    const std::vector<double>& q) const {
  unsigned int axis_count = position.size();
  double distance_squared = 0.0;
  for (unsigned int i = 0; i < axis_count; ++i) {
    double d = q[i] - position[i];
    distance_squared += d * d;
  }
  return std::sqrt(distance_squared);
}

BubbleTree::BubbleTree(
    int max_bubbles_per_branch, const std::vector<double>& root,
    std::shared_ptr<environment::BubbleSourceInterface> bubble_source)
    : max_bubbles_per_branch_(max_bubbles_per_branch),
      bubble_source_(bubble_source), start_point_{root, nullptr},
      attachment_points_(1, AttachmentPoint{root, nullptr}),
      index_(flann::Matrix<double>(
        std::vector<double>(root).data(), 1, root.size()),
             flann::KDTreeIndexParams(8)) {
  index_.buildIndex();
}

bool BubbleTree::Connect(const std::vector<double>& q_target) {
  std::vector<std::vector<int> > indices(1, std::vector<int>(1, 0));
  std::vector<std::vector<double> > distances(1, std::vector<double>(1, 0.0));
  index_.knnSearch(
      flann::Matrix<double>(
          std::vector<double>(q_target).data(), 1, q_target.size()),
      indices, distances, 1, flann::SearchParams(128));
  AttachmentPoint& connection_point = attachment_points_[indices[0][0]];
  return Connect(
      AddNode(connection_point.position, connection_point.parent), q_target);
}

TreeNode* BubbleTree::AddNode(
    const std::vector<double>& q, TreeNode* parent) {
  TreeNode* current_node = new TreeNode(bubble_source_->NewBubble(q), parent);
  Bubble* current_bubble = static_cast<Bubble*>(current_node->point.get());
  nodes_.emplace_back(current_node);
  std::vector<double> position = current_bubble->position();
  std::vector<double> size = current_bubble->size();
  std::vector<double> point = position;
  unsigned int axis_count = position.size();
  for (unsigned int i = 0; i < axis_count; ++i) {
    point[i] = position[i] + size[i];
    attachment_points_.push_back(AttachmentPoint{point, current_node});
    index_.addPoints(flann::Matrix<double>(point.data(), 1, point.size()));
    point[i] = position[i] - size[i];
    attachment_points_.push_back(AttachmentPoint{point, current_node});
    index_.addPoints(flann::Matrix<double>(point.data(), 1, point.size()));
    point[i] = position[i];
  }
  return current_node;
}

bool BubbleTree::Connect(TreeNode* node, const std::vector<double>& q_target) {
  TreeNode* current_node = node;
  Bubble* current_bubble;
  for (int i = 0; i < max_bubbles_per_branch_; ++i) {
    current_bubble = static_cast<Bubble*>(current_node->point.get());
    if (current_bubble->Contains(q_target)) {
      AddNode(q_target, current_node);
      return true;
    }
    current_node = AddNode(
        current_bubble->IntersectsHullAt(q_target), current_node);
  }
  return false;
}

TreeNode* BubbleTree::GetNewestNode() const {
  return nodes_.back().get();
}

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
