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

#include "bubble-rrt.h"

#include <thread>

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace {

void step_thread(BubbleTree* bubble_tree, const std::vector<double>& q,
                 bool* return_value) {
  *return_value = bubble_tree->Connect(q);
}

}  // namespace

BubbleRrt::BubbleRrt(
    const std::vector<double>& q_src, const std::vector<double>& q_dst,
    int max_bubbles_per_branch,
    std::shared_ptr<environment::BubbleSourceInterface> bubble_source,
    RandomPointGeneratorInterface* random_point_generator)
    : random_point_generator_(random_point_generator),
      src_trees_(0), dst_trees_(0) {
  src_trees_.emplace_back(
      new BubbleTree(max_bubbles_per_branch, q_src, bubble_source));
  dst_trees_.emplace_back(
      new BubbleTree(max_bubbles_per_branch, q_dst, bubble_source));
}

bool BubbleRrt::Run(int max_steps) {
  for (int i = 0; i < max_steps; ++i)
    if (Step())
      return true;
  return false;
}

bool BubbleRrt::Step() {
  return Step(random_point_generator_->NextPoint());
}

bool BubbleRrt::Step(const std::vector<double>& q) {
  std::vector<std::unique_ptr<bool> > srcs_connected, dsts_connected;
  std::vector<std::thread> threads;
  for (std::unique_ptr<BubbleTree>& tree : src_trees_) {
    srcs_connected.emplace_back(new bool(false));
    threads.emplace_back(
        step_thread, tree.get(), q, srcs_connected.back().get());
  }
  for (std::unique_ptr<BubbleTree>& tree : dst_trees_) {
    dsts_connected.emplace_back(new bool(false));
    threads.emplace_back(
        step_thread, tree.get(), q, dsts_connected.back().get());
  }
  for (std::thread& thread : threads) {
    thread.join();
  }
  int src_connect = -1;
  int dst_connect = -1;
  for (unsigned int i = 0; i < srcs_connected.size(); ++i)
    if (*(srcs_connected[i])) {
      src_connect = static_cast<int>(i);
      break;
    }
  for (unsigned int i = 0; i < dsts_connected.size(); ++i)
    if (*(dsts_connected[i])) {
      dst_connect = static_cast<int>(i);
      break;
    }
  if (src_connect > -1 && dst_connect > -1) {
    return true;
  }
}

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
