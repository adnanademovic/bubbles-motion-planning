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

#ifndef COM_ADEMOVIC_BUBBLESMP_RRT_TREE_H_
#define COM_ADEMOVIC_BUBBLESMP_RRT_TREE_H_

#include <memory>
#include <vector>

#include "point-index.h"

namespace com {
namespace ademovic {
namespace bubblesmp {

// Should run in one thread, due to sequential nature of "Connect". Thus it is
// not made threadsafe, but supports a threadsafe BubbleSource to be used in
// multiple threads.
class RrtTree {
 public:
  RrtTree(const std::vector<double>& root, const IndexSettings& index_settings);
  virtual ~RrtTree() {}

  bool Extend(const std::vector<double>& q_target);
  TreeNode* GetNewestNode() const;
  virtual bool Connect(
      const TreeNode& node, const std::vector<double>& q_target) const = 0;

 protected:
  virtual TreeNode* AddNode(const std::vector<double>& q, TreeNode* parent) = 0;
  virtual bool ExtendFrom(
      const AttachmentPoint& point, const std::vector<double>& q_target) = 0;

  std::unique_ptr<PointIndex> point_index_;
  std::vector<std::unique_ptr<TreeNode> > nodes_;
};

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com

#endif  // COM_ADEMOVIC_BUBBLESMP_RRT_TREE_H_
