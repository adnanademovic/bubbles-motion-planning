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
#include <google/protobuf/stubs/common.h>

#include "bubblesmp/rrt.h"
#include "bubblesmp/tree-node.h"

DEFINE_bool(verbose, false, "Print verification progress information");
DEFINE_string(output_type, "",
    "Set the information that should be returned in the output ("
    "path - array of configurations, "
    "times - duration of each step in microseconds, "
    "progress - progress information for the current case to stderr, "
    "an empty string results in no output"
    ")");

namespace {
static bool ValidateOutputType(const char* flagname, const std::string& value) {
  if (value != "path" && value != "times" && value != "progress"
      && value != "") {
    printf("Invalid value for --%s: %s\nOptions are: \"\", path, times\n",
           flagname, value.c_str());
    return false;
  }
  return true;
}

static const bool simulation_case_dummy = google::RegisterFlagValidator(
    &FLAGS_output_type, &ValidateOutputType);
}  // namespace

using namespace com::ademovic::bubblesmp;

void OutputPath(std::vector<std::shared_ptr<TreePoint> > points) {
  for (const std::shared_ptr<TreePoint> q : points) {
    for (double qi : q->position())
      printf(" %lf", qi);
    printf("\n");
  }
}

void OutputTimes(const std::vector<long int>& times) {
  for (long int t : times)
    printf("%ld\n", t);
}

template<typename T = std::chrono::microseconds>
struct TimeMeasure
{
  template<typename F, typename ...Args>
  static typename T::rep Run(F func, Args&&... args)
  {
    auto start = std::chrono::system_clock::now();
    func(std::forward<Args>(args)...);
    auto duration = std::chrono::duration_cast<T>
      (std::chrono::system_clock::now() - start);
    return duration.count();
  }
};

void DoStep(Rrt* rrt, bool* done) {
  *done = rrt->Step();
}

std::string MakeUsage(const char* argv0) {
  std::string usage;
  usage += "determines a motion plan for the given task.\n"
           "Usage: ";
  usage += argv0;
  usage += " [OPTION]... [FILE]...\n"
           "Try \'";
  usage += argv0;
  usage += " --help' for more information.";
  return usage;
}

int main(int argc, char** argv) {
  google::SetUsageMessage(MakeUsage("rrtplan"));
  google::SetVersionString("");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(google::GetArgv0());
  if (argc < 2) {
    fprintf(stdout, "%s: %s\n",
        google::ProgramInvocationShortName(), google::ProgramUsage());
    return 2;
  }
  LOG(INFO) << "Number of configurations to open: " << argc - 1;
  LOG(INFO) << "Verifying configurations...";
  for (int task = 1; task < argc; ++task) {
    if (FLAGS_verbose)
      LOG(INFO) << "Verifying " << argv[task];
    Rrt rrt_verification(argv[task]);
  }
  LOG(INFO) << "All configurations are valid";

  for (int task = 1; task < argc; ++task) {
    Rrt rrt(argv[task]);
    LOG(INFO) << "Running case: " << argv[task];
    if (FLAGS_output_type == "progress")
      fprintf(stderr, "Begin: %s\n", argv[task]);
    int step = 0;
    bool done = false;
    std::vector<long int> durations;
    while (!done) {
      durations.push_back(static_cast<long int>(
          TimeMeasure<std::chrono::microseconds>::Run(
            DoStep, &rrt, &done)));
    }
    step = 0;
    for (long int t : durations)
      LOG(INFO) << "Step " << ++step << " took " << t << " us";
    if (FLAGS_output_type == "progress")
      fprintf(stderr, "End: %s\n", argv[task]);
    if (FLAGS_output_type == "path")
      OutputPath(rrt.GetSolution());
    else if (FLAGS_output_type == "times")
      OutputTimes(durations);
  }
  google::protobuf::ShutdownProtobufLibrary();
  google::ShutdownGoogleLogging();
  google::ShutDownCommandLineFlags();
  return 0;
}
