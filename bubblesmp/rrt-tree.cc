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

#include "rrt-tree.h"
#include <glog/logging.h>

namespace com {
namespace ademovic {
namespace bubblesmp {

RrtTree::RrtTree(
    const std::vector<double>& root, const IndexSettings& index_settings) {
  nodes_.emplace_back(new TreeNode(new TreePoint(root), nullptr));
  point_index_.reset(new PointIndex(root, GetNewestNode(), index_settings));
}

ExtensionResult RrtTree::Extend(const std::vector<double>& q_target) {
  AttachmentPoint extension_point = point_index_->GetNearestPoint(q_target);
  return ExtendFrom(extension_point, q_target);
}

TreeNode* RrtTree::GetNewestNode() const {
  return nodes_.back().get();
}

AttachmentPoint RrtTree::ClosestPointTo(
    const std::vector<double>& q) const {
  return point_index_->GetNearestPoint(q);
}

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
