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

#include "bubblesmp/greedy-bubble-tree.h"

#include <algorithm>
#include <cstdio>

#include <glog/logging.h>

#include "bubblesmp/environment/environment-feedback.h"
#include "bubblesmp/bubble.h"

namespace com {
namespace ademovic {
namespace bubblesmp {

GreedyBubbleTree::~GreedyBubbleTree() {}

GreedyBubbleTree::GreedyBubbleTree(
    unsigned max_bubbles_per_branch, unsigned max_binary_search_depth,
    const std::vector<double>& root,
    std::shared_ptr<environment::EnvironmentFeedback> bubble_source,
    double min_move_size, const IndexSettings& index_settings,
    bool use_extended)
    : RrtTree(root, index_settings),
      max_bubbles_per_branch_(max_bubbles_per_branch),
      max_binary_search_depth_(max_binary_search_depth),
      bubble_source_(bubble_source), limits_(bubble_source->GetAngleRanges()),
      min_move_size_(min_move_size), use_extended_(use_extended) {
  CHECK(!bubble_source_->IsCollision(root)) << "Collision at root point";
}

bool GreedyBubbleTree::Connect(
      TreeNode* node, const std::vector<double>& q_target) {
  Bubble* current_bubble = static_cast<Bubble*>(node->point.get());

  if (current_bubble->Contains(q_target))
    return true;
  return CanReachBetween(current_bubble->IntersectsHullAt(q_target), q_target,
                         max_binary_search_depth_);
}

bool GreedyBubbleTree::CanReachBetween(
    const std::vector<double>& q_1, const std::vector<double>& q_2,
    unsigned iterations_left) const {
  if (iterations_left == 0)
    return false;

  iterations_left--;

  unsigned dims = q_1.size();
  std::vector<double> q_mid(dims, 0.0);
  for (unsigned i = 0; i < dims; ++i)
    q_mid[i] = (q_1[i] + q_2[i]) / 2.0;
  std::unique_ptr<Bubble> bubble(
      bubble_source_->NewBubble(q_mid, use_extended_));

  if (bubble->IsCollision())
    return false;

  if (!bubble->Contains(q_1) && !CanReachBetween(
        q_1, bubble->IntersectsHullAt(q_1), iterations_left))
    return false;
  if (!bubble->Contains(q_2) && !CanReachBetween(
        q_2, bubble->IntersectsHullAt(q_2), iterations_left))
    return false;

  return true;
}

TreeNode* GreedyBubbleTree::AddNode(
    const std::vector<double>& q, TreeNode* parent) {
  TreeNode* current_node = new TreeNode(
      bubble_source_->NewBubble(q, use_extended_), parent);
  Bubble* current_bubble = static_cast<Bubble*>(current_node->point.get());
  nodes_.emplace_back(current_node);
  std::vector<double> position(current_bubble->position());
  std::vector<double> size(current_bubble->size());
  std::vector<double> point(position);
  unsigned int axis_count = position.size();
  for (size_t i = 0; i < axis_count; ++i) {
    point[i] = std::min(limits_[i].second, position[i] + size[i]);
    point_index_->AddPoint(point, current_node);
    point[i] = std::max(limits_[i].first, position[i] - size[i]);
    point_index_->AddPoint(point, current_node);
    point[i] = position[i];
  }
  return current_node;
}

ExtensionResult GreedyBubbleTree::ExtendFrom(
    const AttachmentPoint& point, const std::vector<double>& q_target) {
  TreeNode* current_node = AddNode(point.position, point.parent);
  Bubble* current_bubble = static_cast<Bubble*>(current_node->point.get());

  double move_size_limit = min_move_size_;

  for (int i = 0; i < max_bubbles_per_branch_; ++i) {
    current_bubble = static_cast<Bubble*>(current_node->point.get());
    if (current_bubble->Contains(q_target)) {
      AddNode(q_target, current_node);
      return ExtensionResult::REACHED;
    }

    std::vector<double> q_next(current_bubble->IntersectsHullAt(q_target));
    std::vector<double> q_prev(current_bubble->position());

    // Just like Classic RRT has a big step size that would represent the
    // resolution of the movement, Bubble RRT has its own as a lower limit,
    // whereby the limit is either the big step or embracing the target.
    // Embracing the target is checked in the early return above.
    // TODO: experiment with more heuristics.
    double current_move_size = 0.0;
    for (size_t j = 0; j < q_next.size(); ++j)
      current_move_size += fabs(q_next[j] - q_prev[j]);
    if (move_size_limit < 0.05 * current_move_size)
      move_size_limit = 0.05 * current_move_size;
    else if (current_move_size < move_size_limit)
      return ExtensionResult::TRAPPED;

    current_node = AddNode(q_next, current_node);
  }
  return ExtensionResult::TRAPPED;
}

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
