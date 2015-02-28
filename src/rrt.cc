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

#include "rrt.h"

#include <thread>
#include <queue>

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace {

void step_thread(RrtTree* rrt_tree, const std::vector<double>& q,
                 bool* return_value) {
  *return_value = rrt_tree->Connect(q);
}

}  // namespace

Rrt::Rrt(RrtTree* src_tree, RrtTree* dst_tree,
         RandomPointGeneratorInterface* random_point_generator)
    : random_point_generator_(random_point_generator),
      src_tree_(src_tree), dst_tree_(dst_tree), done_(false) {}

bool Rrt::Run(int max_steps) {
  for (int i = 0; i < max_steps; ++i)
    if (Step())
      return true;
  return false;
}

bool Rrt::Step(bool connect) {
  return Step(random_point_generator_->NextPoint(), connect);
}

bool Rrt::Step(const std::vector<double>& q, bool connect) {
  if (done_)
    return true;
  bool src_connected = false;
  bool dst_connected = false;
  std::vector<std::thread> threads;

  threads.emplace_back(step_thread, src_tree_.get(), q, &src_connected);
  threads.emplace_back(step_thread, dst_tree_.get(), q, &dst_connected);

  for (std::thread& thread : threads) {
    thread.join();
  }

  src_connect_node_ = src_tree_->GetNewestNode();
  dst_connect_node_ = dst_tree_->GetNewestNode();

  if (src_connected && dst_connected) {
    done_ = true;
    return true;
  }

  if (connect) {
    src_connected = dst_connected = false;

    threads.clear();
    threads.emplace_back(step_thread, src_tree_.get(),
                         dst_connect_node_->point->position(), &src_connected);
    threads.emplace_back(step_thread, dst_tree_.get(),
                         src_connect_node_->point->position(), &dst_connected);

    for (std::thread& thread : threads) {
      thread.join();
    }

    if (src_connected) {
      src_connect_node_ = src_tree_->GetNewestNode();
      done_ = true;
      return true;
    }

    if (dst_connected) {
      dst_connect_node_ = dst_tree_->GetNewestNode();
      done_ = true;
      return true;
    }
  }

  return false;
}

std::vector<std::shared_ptr<TreePoint> > Rrt::GetSolution() const {
  if (!done_)
    return std::vector<std::shared_ptr<TreePoint> >(0);
  std::deque<TreeNode*> nodes;
  nodes.push_back(src_connect_node_);
  nodes.push_back(dst_connect_node_);
  while (nodes.front()->parent != nullptr)
    nodes.push_front(nodes.front()->parent);
  while (nodes.back()->parent != nullptr)
    nodes.push_back(nodes.back()->parent);
  std::vector<std::shared_ptr<TreePoint> > solution;
  for (TreeNode* node : nodes)
    solution.emplace_back(node->point);
  return solution;
}

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
