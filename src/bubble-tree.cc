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
      bubble_source_(bubble_source), start_point_{root, nullptr} {
  AddNode(root, nullptr);
}

bool BubbleTree::Connect(const std::vector<double>& q_target) {
  for (const std::unique_ptr<Node>& node : nodes_) {
    if (node->bubble->Contains(q_target)) {
      return Connect(node.get(), q_target);
    }
  }
  double distance = start_point_.Distance(q_target);
  AttachmentPoint& connection_point = start_point_;
  for (AttachmentPoint& point : attachment_points_) {
    double new_distance = point.Distance(q_target);
    if (new_distance < distance) {
      distance = new_distance;
      connection_point = point;
    }
  }
  return Connect(
      AddNode(connection_point.position, connection_point.parent), q_target);
}

BubbleTree::Node* BubbleTree::AddNode(
    const std::vector<double>& q, Node* parent) {
  Node* current_node = new Node(bubble_source_->NewBubble(q), parent);
  nodes_.emplace_back(current_node);
  std::vector<double> position = current_node->bubble->position();
  std::vector<double> size = current_node->bubble->size();
  std::vector<double> point = position;
  unsigned int axis_count = position.size();
  for (unsigned int i = 0; i < axis_count; ++i) {
    point[i] = position[i] + size[i];
    attachment_points_.push_back(AttachmentPoint{point, current_node});
    point[i] = position[i] - size[i];
    attachment_points_.push_back(AttachmentPoint{point, current_node});
    point[i] = position[i];
  }
  return current_node;
}

bool BubbleTree::Connect(Node* node, const std::vector<double>& q_target) {
  Node* current_node = node;
  for (int i = 0; i < max_bubbles_per_branch_; ++i) {
    if (current_node->bubble->Contains(q_target)) {
      AddNode(q_target, current_node);
      return true;
    }
    current_node = AddNode(
        current_node->bubble->IntersectsHullAt(q_target), current_node);
  }
  return false;
}

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
