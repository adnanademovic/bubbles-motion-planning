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

namespace com {
namespace ademovic {
namespace bubblesmp {

BubbleTree::BubbleTree(
    int max_bubbles_per_branch, const std::vector<double>& root,
    std::shared_ptr<environment::BubbleSourceInterface> bubble_source)
    : RrtTree(root), max_bubbles_per_branch_(max_bubbles_per_branch),
      bubble_source_(bubble_source) {}

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
    point_index_.AddPoint(point, parent);
    point[i] = position[i] - size[i];
    point_index_.AddPoint(point, parent);
    point[i] = position[i];
  }
  return current_node;
}

bool BubbleTree::ConnectLine(
    const AttachmentPoint& point, const std::vector<double>& q_target) {
  TreeNode* current_node = AddNode(point.position, point.parent);
  Bubble* current_bubble = static_cast<Bubble*>(current_node->point.get());

  double previous_bubble_size = 0.0;
  for (double element : current_bubble->size()) {
    previous_bubble_size += element;
  }

  for (int i = 0; i < max_bubbles_per_branch_; ++i) {
    current_bubble = static_cast<Bubble*>(current_node->point.get());
    if (current_bubble->Contains(q_target)) {
      AddNode(q_target, current_node);
      return true;
    }

    double current_bubble_size = 0.0;
    for (double element : current_bubble->size()) {
      current_bubble_size += element;
    }
    if (current_bubble_size < 0.1 * previous_bubble_size) {
      return false;
    }

    current_node = AddNode(
        current_bubble->IntersectsHullAt(q_target), current_node);
  }
  return false;
}

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
