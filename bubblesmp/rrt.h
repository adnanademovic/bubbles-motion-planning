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

#ifndef COM_ADEMOVIC_BUBBLESMP_BUBBLE_RRT_H_
#define COM_ADEMOVIC_BUBBLESMP_BUBBLE_RRT_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#define BOOST_SPIRIT_THREADSAFE
#include <boost/filesystem.hpp>

namespace com {
namespace ademovic {
namespace bubblesmp {

namespace generators {
class RandomPointGeneratorInterface;
}  // namespace generators

class RrtTree;
class TaskConfig;
class TreeNode;
class TreePoint;

class Rrt {
 public:
  explicit Rrt(const std::string& configuration);
  Rrt(const TaskConfig& configuration,
      const boost::filesystem::path& config_file_path);
  // Takes ownership of everything passed to it.
  Rrt(RrtTree* src_tree, RrtTree* dst_tree,
      generators::RandomPointGeneratorInterface* random_point_generator);
  virtual ~Rrt();

  bool Run(int max_steps);
  bool Step();
  bool Step(const std::vector<double>& q);
  std::vector<std::shared_ptr<TreePoint> > GetSolution() const;

 private:
  void Configure(const TaskConfig& config,
                 const boost::filesystem::path& config_file_path);

  std::unique_ptr<generators::RandomPointGeneratorInterface>
      random_point_generator_;
  std::unique_ptr<RrtTree> src_tree_;
  std::unique_ptr<RrtTree> dst_tree_;
  TreeNode* src_connect_node_;
  TreeNode* dst_connect_node_;
  bool done_;
};

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com

#endif  // COM_ADEMOVIC_BUBBLESMP_BUBBLE_RRT_H_
