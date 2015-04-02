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

#include <iostream>
#include <fstream>
#include <thread>
#include <queue>

#include <google/protobuf/text_format.h>

#include "environment/environment-feedback.h"
#include "environment/make-environment.h"
#include "generators/make-generator.h"
#include "bubble-tree.h"
#include "classic-tree.h"

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace {

void step_thread(RrtTree* rrt_tree, const std::vector<double>& q,
                 bool* return_value) {
  *return_value = rrt_tree->Connect(q);
}

}  // namespace


Rrt::Rrt(const std::string& configuration) {
  boost::filesystem::path config_file_path(configuration);
  config_file_path.remove_filename();
  std::ifstream fin(configuration);
  TaskConfig config_pb;
  std::string input_string((std::istreambuf_iterator<char>(fin)),
                           std::istreambuf_iterator<char>());
  bool success = google::protobuf::TextFormat::ParseFromString(
      input_string, &config_pb);
  fin.close();
  if (!success) {
    std::cerr << "Failed parsing file: " << configuration << std::endl;
    throw;
  }
  Configure(config_pb, config_file_path);
}

void Rrt::Configure(const TaskConfig& config,
                    const boost::filesystem::path& config_file_path) {
  done_ = false;
  std::vector<double> src, dst;
  for (double q : config.source().q())
    src.push_back(q);
  for (double q : config.destination().q())
    dst.push_back(q);
  std::shared_ptr<environment::EnvironmentFeedback> src_bubble_source(
      new environment::EnvironmentFeedback(
          environment::NewEnvironmentFromProtoBuffer(
              config.environment(), config_file_path)));
  std::shared_ptr<environment::EnvironmentFeedback> dst_bubble_source(
      new environment::EnvironmentFeedback(
          environment::NewEnvironmentFromProtoBuffer(
              config.environment(), config_file_path)));
  switch (config.tree().type()) {
    case (TreeConfig::BUBBLE):
      src_tree_.reset(new BubbleTree(
          static_cast<int>(config.tree().max_bubbles_per_branch()), src,
          src_bubble_source, config.tree().min_move_length(), config.index()));
      dst_tree_.reset(new BubbleTree(
          static_cast<int>(config.tree().max_bubbles_per_branch()), dst,
          dst_bubble_source, config.tree().min_move_length(), config.index()));
      break;
    case (TreeConfig::CLASSIC):
      src_tree_.reset(new ClassicTree(
          config.tree().step_length(),
          static_cast<int>(config.tree().checks_per_step()), src,
          src_bubble_source, config.index()));
      dst_tree_.reset(new ClassicTree(
          config.tree().step_length(),
          static_cast<int>(config.tree().checks_per_step()), dst,
          dst_bubble_source, config.index()));
      break;
    default:
      std::cerr << stderr, "Invalid TreeConfig type given\n";
      throw;
      break;
  }
  random_point_generator_.reset(NewGeneratorFromProtoBuffer(
      src_bubble_source->GetAngleRanges(), config.generator()));
}

Rrt::Rrt(RrtTree* src_tree, RrtTree* dst_tree,
         generators::RandomPointGeneratorInterface* random_point_generator)
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
