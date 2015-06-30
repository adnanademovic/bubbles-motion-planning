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

#include "bubblesmp/greedy-classic-tree.h"

#include <cmath>
#include <cstdio>

#include <glog/logging.h>

#include "bubblesmp/environment/environment-feedback.h"
#include "bubblesmp/tree-node.h"

namespace com {
namespace ademovic {
namespace bubblesmp {

GreedyClassicTree::~GreedyClassicTree() {}

GreedyClassicTree::GreedyClassicTree(
    double max_step, unsigned substeps, const std::vector<double>& root,
    std::shared_ptr<environment::EnvironmentFeedback> collision_source,
    const IndexSettings& index_settings)
    : RrtTree(root, index_settings), eps_(max_step / substeps),
      substeps_(substeps), collision_source_(collision_source) {
  CHECK(!collision_source_->IsCollision(root)) << "Collision at root point";
}

bool GreedyClassicTree::Connect(
      TreeNode* node, const std::vector<double>& q_target) {
  std::vector<double> current(node->point->position());
  std::vector<double> step(q_target);
  unsigned int axis_count = q_target.size();

  double length = 0.0;
  for (unsigned int i = 0; i < axis_count; ++i) {
    step[i] -= current[i];
    length += step[i] * step[i];
  }

  length = sqrt(length);
  int max_steps = int(length / eps_) + 1;
  length = eps_ / length;

  for (unsigned int i = 0; i < axis_count; ++i)
    step[i] *= length;
  for (int s = 0; s < max_steps; ++s) {
    for (unsigned int i = 0; i < axis_count; ++i)
      current[i] += step[i];

    if (s == max_steps - 1)
      current = q_target;

    if (collision_source_->IsCollision(current)) {
      int steps_shortened = s - s % substeps_;
      if (!steps_shortened)
        return false;
      current = node->point->position();
      for (unsigned int i = 0; i < axis_count; ++i)
        current[i] += step[i] * steps_shortened;
      return false;
    }
  }
  return true;
}

TreeNode* GreedyClassicTree::AddNode(
    const std::vector<double>& q, TreeNode* parent) {
  TreeNode* current_node = new TreeNode(new TreePoint(q), parent);
  nodes_.emplace_back(current_node);
  point_index_->AddPoint(q, current_node);
  return current_node;
}

ExtensionResult GreedyClassicTree::ExtendFrom(
    const AttachmentPoint& point, const std::vector<double>& q_target) {
  std::vector<double> current(point.position);
  std::vector<double> step(q_target);
  unsigned int axis_count = q_target.size();

  double length = 0.0;
  for (unsigned int i = 0; i < axis_count; ++i) {
    step[i] -= current[i];
    length += step[i] * step[i];
  }

  length = sqrt(length);
  int max_steps = int(length / eps_) + 1;
  length = eps_ / length;

  for (unsigned int i = 0; i < axis_count; ++i)
    step[i] *= length;
  for (int s = 0; s < max_steps; ++s) {
    for (unsigned int i = 0; i < axis_count; ++i)
      current[i] += step[i];

    if (s == max_steps - 1)
      current = q_target;

    if (collision_source_->IsCollision(current)) {
      int steps_shortened = s - s % substeps_;
      if (!steps_shortened)
        return ExtensionResult::TRAPPED;
      current = point.position;
      for (unsigned int i = 0; i < axis_count; ++i)
        current[i] += step[i] * steps_shortened;
      AddNode(current, point.parent);
      return ExtensionResult::TRAPPED;
    }
  }
  AddNode(q_target, point.parent);
  return ExtensionResult::REACHED;
}

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
