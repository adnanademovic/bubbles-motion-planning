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


#include <chrono>
#include <cstdio>
#include <string>
#include <queue>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "rrt.h"
#include "environment/environment-feedback.h"
#include "environment/make-environment.h"

DEFINE_double(step_low, 3.6, "Minimum length of the RRT step");
DEFINE_double(step_high, 3600.0, "Maximum length of the RRT step");
DEFINE_double(step_threshold, 0.2, "Biggest error for best RRT step length");
DEFINE_double(check_threshold, 0.2, "Collision check substep");
DEFINE_int32(test_count, 30, "Number of tests done one one step length");

using namespace com::ademovic::bubblesmp;

std::string MakeUsage(const char* argv0) {
  std::string usage;
  usage += "determines a motion plan for the given task.\n"
           "Usage: ";
  usage += argv0;
  usage += " [OPTION]... [DATA PATH]\n"
           "Try \'";
  usage += argv0;
  usage += " --help' for more information.";
  return usage;
}

double GetStepLength(
    TaskConfig config, boost::filesystem::path config_path,
    double min, double max) {
  if (max - min < FLAGS_step_threshold)
    return min;

  double current = (max + min) / 2.0;
  //current = 3.6;

  config.mutable_tree()->set_step_length(current);
  LOG(INFO) << "Running case for step length: " << current;

  bool has_collision(false);

  for (int tests = 0; !has_collision && tests < FLAGS_test_count; ++tests) {
    Rrt rrt(config, config_path);

    bool done = false;
    while (!done) {
      done = rrt.Step();
      LOG_EVERY_N(INFO, 1000) << google::COUNTER << " steps done";
    }

    std::vector<std::shared_ptr<TreePoint> > solution(rrt.GetSolution());
    std::unique_ptr<environment::EnvironmentFeedback> collision_checker(
        new environment::EnvironmentFeedback(
            environment::NewEnvironmentFromProtoBuffer(
                config.environment(), config_path)));
    for (size_t i = 1; !has_collision && i < solution.size(); ++i) {
      std::vector<double> p_begin(solution[i - 1]->position());
      std::vector<double> p_step(solution[i]->position());
      double length(0.0);
      for (size_t j = 0; j < p_step.size(); ++j) {
        p_step[j] -= p_begin[j];
        length += fabs(p_step[j]);
      }
      int substeps = (int)(length / FLAGS_check_threshold) + 1;
      for (size_t j = 0; j < p_step.size(); ++j)
        p_step[j] /= (double) substeps;
      substeps++;
      for (int step = 0; step < substeps; ++step) {
        if (collision_checker->IsCollision(p_begin))
          has_collision = true;
        for (size_t j = 0; j < p_step.size(); ++j)
          p_begin[j] += p_step[j];
      }
    }
  }
  if (has_collision) {
    LOG(INFO) << "Length " << current << " doesn't meet requirements";
    return GetStepLength(config, config_path, min, current);
  } else {
    LOG(INFO) << "Length " << current << " meets requirements";
    return GetStepLength(config, config_path, current, max);
  }
}

int main(int argc, char** argv) {
  google::SetUsageMessage(MakeUsage("rrtoptimize"));
  google::SetVersionString("");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(google::GetArgv0());
  CHECK_LT(FLAGS_step_low, FLAGS_step_high)
    << "The lowest step length must be smaller than the highest step length";
  if (argc != 2) {
    fprintf(stdout, "%s: %s\n",
        google::ProgramInvocationShortName(), google::ProgramUsage());
    return 2;
  }

  TaskConfig task_config;
  task_config.mutable_environment()->set_robot_filename("robot/setup.robot");
  task_config.mutable_environment()->set_environment_filename(
      "environment/obstacles_hard.stl");
  task_config.mutable_environment()->set_max_underestimate(20.0);
  task_config.mutable_index()->set_type(IndexSettings::KD_TREE);
  task_config.mutable_index()->mutable_index_params()->set_trees(8);
  task_config.mutable_index()->mutable_search_params()->set_checks(8);
  task_config.mutable_index()->mutable_search_params()->set_use_heap(false);
  task_config.mutable_tree()->set_type(TreeConfig::CLASSIC);
  task_config.mutable_tree()->set_attempt_connect(true);
  task_config.mutable_source()->add_q(-90.0);
  task_config.mutable_source()->add_q( 50.0);
  task_config.mutable_source()->add_q(-50.0);
  task_config.mutable_source()->add_q(  0.0);
  task_config.mutable_source()->add_q(  0.0);
  task_config.mutable_source()->add_q(  0.0);
  task_config.mutable_destination()->add_q(  0.0);
  task_config.mutable_destination()->add_q( 90.0);
  task_config.mutable_destination()->add_q(-90.0);
  task_config.mutable_destination()->add_q( -9.0);
  task_config.mutable_destination()->add_q( -9.0);
  task_config.mutable_destination()->add_q( -9.0);

  boost::filesystem::path config_file_path(argv[1]);

  double step_length = GetStepLength(
      task_config, config_file_path, FLAGS_step_low, FLAGS_step_high);
  printf("%lf\n", step_length);

  google::protobuf::ShutdownProtobufLibrary();
  google::ShutdownGoogleLogging();
  google::ShutDownCommandLineFlags();
  return 0;
}
