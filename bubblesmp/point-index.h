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

#ifndef COM_ADEMOVIC_BUBBLESMP_POINT_INDEX_H_
#define COM_ADEMOVIC_BUBBLESMP_POINT_INDEX_H_

#include <memory>
#include <vector>

#include <flann/flann.hpp>

namespace com {
namespace ademovic {
namespace bubblesmp {

class TreeNode;
class IndexSettings;

struct AttachmentPoint {
  std::vector<double> position;
  TreeNode* parent;
};

// Not threadsafe, use in single thread.
class PointIndex {
 public:
  PointIndex(const std::vector<double>& q_root, TreeNode* root_node,
             const IndexSettings& flann_settings);
  virtual ~PointIndex() {}

  // PointIndex doesn't take ownership of parent
  void AddPoint(const std::vector<double>& q, TreeNode* parent);
  AttachmentPoint GetNearestPoint(const std::vector<double>& q) const;

 private:
  std::vector<AttachmentPoint> attachment_points_;
  std::unique_ptr<flann::Index<flann::L2<double> > > index_;
  flann::SearchParams search_parameters_;
};

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com

#endif  // COM_ADEMOVIC_BUBBLESMP_POINT_INDEX_H_
