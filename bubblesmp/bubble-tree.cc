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

#include "bubble-tree.h"

#include <algorithm>
#include <cstdio>

#include <glog/logging.h>

namespace com {
namespace ademovic {
namespace bubblesmp {

BubbleTree::BubbleTree(
    double eps, double min_bubble_reach,
    double max_bubble_gap, const std::vector<double>& root,
    std::shared_ptr<environment::EnvironmentFeedback> bubble_source,
    const IndexSettings& index_settings)
    : RrtTree(root, index_settings), eps_(eps),
      min_bubble_reach_(min_bubble_reach), max_bubble_gap_(max_bubble_gap),
      bubble_source_(bubble_source), limits_(bubble_source->GetAngleRanges()) {
  CHECK(!bubble_source_->IsCollision(root)) << "Collision at root point";
}

bool BubbleTree::Connect(TreeNode* node, const std::vector<double>& q_target) {
  Bubble* current_bubble = static_cast<Bubble*>(node->point.get());

  if (current_bubble->Contains(q_target))
    return true;
  TreeNode tree_node(nullptr, nullptr);
  TreeNode* tn_ptr = &tree_node;
  return CanReachBetween(current_bubble->IntersectsHullAt(q_target), q_target,
                         node, true, &tn_ptr);
}

bool BubbleTree::CanReachBetween(
    const std::vector<double>& q_1, const std::vector<double>& q_2,
    TreeNode* parent, bool use_bubbles, TreeNode** ret_final_node) {
  size_t axis_count = q_1.size();
  *ret_final_node = parent;

  if (!use_bubbles) {
    double length = 0.0;
    for (size_t i = 0; i < axis_count; ++i)
      length += fabs(q_2[i] - q_1[i]);
    if (length < max_bubble_gap_)
      return true;
  }

  std::vector<double> q_mid(axis_count, 0.0);
  for (size_t i = 0; i < axis_count; ++i)
    q_mid[i] = (q_1[i] + q_2[i]) / 2.0;
  if (use_bubbles) {
    std::unique_ptr<Bubble> bubble(bubble_source_->NewBubble(q_mid));
    Bubble* bubble_ptr = bubble.get();
    std::vector<double> q_1_hull(bubble_ptr->IntersectsHullAt(q_1));
    std::vector<double> q_2_hull(bubble_ptr->IntersectsHullAt(q_2));
    double length = 0.0;
    for (size_t i = 0; i < axis_count; ++i)
      length += fabs(q_1_hull[i] - q_mid[i]);
    bool children_use_bubbles = length > min_bubble_reach_;

    if (!bubble_ptr->Contains(q_1) && !CanReachBetween(
        q_1, q_1_hull, parent, children_use_bubbles, ret_final_node))
      return false;

    if (bubble_ptr->IsCollision())
      return false;

    TreeNode* mid_bubble = AddNodeFromBubble(*ret_final_node, bubble.release());
    *ret_final_node = mid_bubble;

    if (!bubble_ptr->Contains(q_2) && !CanReachBetween(
          q_2_hull, q_2, mid_bubble, children_use_bubbles, ret_final_node))
      return false;
  } else {
    if (bubble_source_->IsCollision(q_mid))
        return false;
    if (!CanReachBetween(q_1, q_mid, parent, false, ret_final_node))
        return false;
    if (!CanReachBetween(q_mid, q_2, parent, false, ret_final_node))
        return false;
  }
  return true;
}

TreeNode* BubbleTree::AddNode(const std::vector<double>& q, TreeNode* parent) {
  return AddNodeFromBubble(parent, bubble_source_->NewBubble(q));
}

TreeNode* BubbleTree::AddNodeFromBubble(TreeNode* parent, Bubble* bubble) {
  TreeNode* current_node = new TreeNode(bubble, parent);
  Bubble* current_bubble = static_cast<Bubble*>(current_node->point.get());
  nodes_.emplace_back(current_node);
  std::vector<double> position(current_bubble->position());
  std::vector<double> size(current_bubble->size());
  std::vector<double> point(position);
  size_t axis_count = position.size();
  for (size_t i = 0; i < axis_count; ++i) {
    point[i] = std::min(limits_[i].second, position[i] + size[i]);
    point_index_->AddPoint(point, current_node);
    point[i] = std::max(limits_[i].first, position[i] - size[i]);
    point_index_->AddPoint(point, current_node);
    point[i] = position[i];
  }
  return current_node;
}

ExtensionResult BubbleTree::ExtendFrom(
    const AttachmentPoint& point, const std::vector<double>& q_target) {
  TreeNode* node = AddNode(point.position, point.parent);
  Bubble* current_bubble = static_cast<Bubble*>(node->point.get());

  std::vector<double> current(current_bubble->IntersectsHullAt(q_target));
  std::vector<double> step(q_target);
  size_t axis_count = q_target.size();

  double length = 0.0;
  for (size_t i = 0; i < axis_count; ++i) {
    step[i] -= current[i];
    length += fabs(step[i]);
  }

  bool not_reached = length > eps_;
  if (not_reached)
    length = eps_ / length;
  else
    length = 1.0;

  for (size_t i = 0; i < axis_count; ++i)
    current[i] += step[i] * length;

  ExtensionResult retval = not_reached
      ? ExtensionResult::ADVANCED
      : ExtensionResult::REACHED;

  if (current_bubble->Contains(current))
    return retval;

  TreeNode tree_node(nullptr, nullptr);
  TreeNode* tn_ptr = &tree_node;
  if (!CanReachBetween(current_bubble->IntersectsHullAt(current), current, node,
      true, &tn_ptr))
    retval = ExtensionResult::ADVANCED;

  return retval;
}

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
