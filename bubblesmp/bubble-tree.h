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

#ifndef COM_ADEMOVIC_BUBBLESMP_BUBBLE_TREE_H_
#define COM_ADEMOVIC_BUBBLESMP_BUBBLE_TREE_H_

#include <memory>
#include <utility>
#include <vector>

#include "bubble.h"
#include "rrt-tree.h"
#include "environment/environment-feedback.h"

namespace com {
namespace ademovic {
namespace bubblesmp {

// Should run in one thread, due to sequential nature of "Connect". Thus it is
// not made threadsafe, but supports a threadsafe BubbleSource to be used in
// multiple threads.
class BubbleTree : public RrtTree {
 public:
  BubbleTree(double eps, unsigned halving_depth, double min_bubble_reach,
             double max_bubble_gap, const std::vector<double>& root,
             std::shared_ptr<environment::EnvironmentFeedback> bubble_source,
             const IndexSettings& index_settings);
  virtual ~BubbleTree() {}

  virtual bool Connect(TreeNode* node, const std::vector<double>& q_target);

 private:
  // Does not take ownership of parent.
  // Has ownership of returned pointer.
  virtual TreeNode* AddNode(const std::vector<double>& q, TreeNode* parent);
  TreeNode* AddNodeFromBubble(TreeNode* parent, Bubble* bubble);
  virtual ExtensionResult ExtendFrom(
      const AttachmentPoint& point, const std::vector<double>& q_target);

  bool CanReachBetween(
      const std::vector<double>& q_1, const std::vector<double>& q_2,
      TreeNode* parent, bool use_bubbles, TreeNode** ret_final_node,
      int halving_depth);

  double eps_;
  int halvings_;
  unsigned bubbles_per_extend_;
  double min_bubble_reach_;
  double max_bubble_gap_;
  std::shared_ptr<environment::EnvironmentFeedback> bubble_source_;
  std::vector<std::pair<double, double> > limits_;
};

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com

#endif  // COM_ADEMOVIC_BUBBLESMP_BUBBLE_TREE_H_
