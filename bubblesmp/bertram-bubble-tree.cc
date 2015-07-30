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

#include "bubblesmp/bertram-bubble-tree.h"

#include <cmath>
#include <cstdio>

#include <glog/logging.h>

#include "bubblesmp/environment/environment-feedback.h"
#include "bubblesmp/bubble.h"

namespace com {
namespace ademovic {
namespace bubblesmp {

BertramBubbleTree::~BertramBubbleTree() {}

BertramBubbleTree::BertramBubbleTree(
    double s_min, unsigned substeps, const std::vector<double>& root,
    std::shared_ptr<environment::EnvironmentFeedback> bubble_source,
    const IndexSettings& index_settings, bool use_extended)
    : RrtTree(root, index_settings), eps_(s_min / substeps),
      substeps_(substeps), bubble_source_(bubble_source),
      use_extended_(use_extended) {
  CHECK(!bubble_source_->IsCollision(root)) << "Collision at root point";
}

bool BertramBubbleTree::Connect(TreeNode* node, const std::vector<double>& q_target) {
  TreeNode* current = node;
  ExtensionResult result;

  do {
    result = ExtendFromNode(current->point->position(), current, q_target);
    current = GetNewestNode();
  } while (result == ExtensionResult::ADVANCED);

  return result == ExtensionResult::REACHED;
}

TreeNode* BertramBubbleTree::AddNode(
    const std::vector<double>& q, TreeNode* parent) {
  TreeNode* current_node = new TreeNode(new TreePoint(q), parent);
  nodes_.emplace_back(current_node);
  point_index_->AddPoint(q, current_node);
  return current_node;
}

ExtensionResult BertramBubbleTree::ExtendFrom(
    const AttachmentPoint& point, const std::vector<double>& q_target) {
  return ExtendFromNode(point.position, point.parent, q_target);
}

ExtensionResult BertramBubbleTree::ExtendFromNode(
    const std::vector<double>& q, TreeNode* node,
    const std::vector<double>& q_target) {
  ExtensionResult retval = ExtendBubbleFromNode(q, node, q_target);
  if (retval != ExtensionResult::TRAPPED)
    return retval;
  retval = ExtendCollisionFromNode(q, node, q_target);
  return retval;
}

ExtensionResult BertramBubbleTree::ExtendBubbleFromNode(
    const std::vector<double>& q, TreeNode* node,
    const std::vector<double>& q_target) {
  std::unique_ptr<Bubble> bubble(bubble_source_->NewBubble(q, use_extended_));

  if (bubble->Contains(q_target)) {
    AddNode(q_target, node);
    return ExtensionResult::REACHED;
  }

  std::vector<double> step(bubble->IntersectsHullAt(q_target));
  size_t axis_count = q_target.size();

  double length = 0.0;
  for (size_t i = 0; i < axis_count; ++i)
    length += fabs(step[i] - q[i]);

  if (length < eps_ * substeps_)
    return ExtensionResult::TRAPPED;

  AddNode(step, node);
  return ExtensionResult::ADVANCED;
}

ExtensionResult BertramBubbleTree::ExtendCollisionFromNode(
    const std::vector<double>& q, TreeNode* node,
    const std::vector<double>& q_target) {
  std::vector<double> current(q);
  std::vector<double> step(q_target);
  size_t axis_count = q_target.size();

  double length = 0.0;
  for (size_t i = 0; i < axis_count; ++i) {
    step[i] -= current[i];
    length += fabs(step[i]);
  }

  bool not_reached = length > eps_ * substeps_;
  if (not_reached)
    length = eps_ / length;
  else
    length = 1.0 / substeps_;

  for (size_t i = 0; i < axis_count; ++i)
    step[i] *= length;

  for (int s = 0; s < substeps_; ++s) {
    for (size_t i = 0; i < axis_count; ++i)
      current[i] += step[i];
    if (bubble_source_->IsCollision(current))
      return ExtensionResult::TRAPPED;
  }
  AddNode(not_reached ? current : q_target, node);
  return not_reached
      ? ExtensionResult::ADVANCED
      : ExtensionResult::REACHED;
}

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
