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

#ifndef COM_ADEMOVIC_BUBBLESMP_BUBBLE_TREE_H_
#define COM_ADEMOVIC_BUBBLESMP_BUBBLE_TREE_H_

#include <memory>
#include <vector>

#include <flann/flann.hpp>

#include "bubble.h"
#include "environment/bubble-source-interface.h"

namespace com {
namespace ademovic {
namespace bubblesmp {

// Should run in one thread, due to sequential nature of "Connect". Thus it is
// not made threadsafe, but supports a threadsafe BubbleSource to be used in
// multiple threads.
class BubbleTree {
 public:
  struct Node {
    // Takes ownership of bubble.
    // Does not take ownership of parent.
    Node(Bubble* bubble, Node* parent)
        : bubble(bubble), parent(parent) {}
    std::shared_ptr<Bubble> bubble;
    Node* parent;
  };

  BubbleTree(int max_bubbles_per_branch, const std::vector<double>& root,
             std::shared_ptr<environment::BubbleSourceInterface> bubble_source);

  bool Connect(const std::vector<double>& q_target);
  Node* GetNewestNode() const;

 private:
  struct AttachmentPoint {
    double Distance(const std::vector<double>& q) const;
    std::vector<double> position;
    Node* parent;
  };

  // Does not take ownership of parent.
  // Has ownership of returned pointer.
  Node* AddNode(const std::vector<double>& q, Node* parent);
  bool Connect(Node* node, const std::vector<double>& q_target);

  int max_bubbles_per_branch_;
  std::shared_ptr<environment::BubbleSourceInterface> bubble_source_;
  AttachmentPoint start_point_;
  std::vector<AttachmentPoint> attachment_points_;
  std::vector<std::unique_ptr<Node> > nodes_;

  flann::Index<flann::L2<double> > index_;
};

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com

#endif  // COM_ADEMOVIC_BUBBLESMP_BUBBLE_TREE_H_
