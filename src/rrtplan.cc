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

#include <gflags/gflags.h>

#include "rrt.h"

DEFINE_bool(verbose, false, "Print algorithm progress information");
DEFINE_string(output_type, "path",
    "Set the information that should be returned in the output ("
    "path - array of configurations, "
    "times - duration of each step in microseconds"
    ")");

namespace {
static bool ValidateOutputType(const char* flagname, const std::string& value) {
  if (value != "path" && value != "times") {
    printf("Invalid value for --%s: %s\nOptions are: path, times\n",
           flagname, value.c_str());
    return false;
  }
  return true;
}

static const bool simulation_case_dummy = google::RegisterFlagValidator(
    &FLAGS_output_type, &ValidateOutputType);
}  // namespace

using namespace com::ademovic::bubblesmp;

void DrawLine(
    const std::vector<double>& start, const std::vector<double>& goal) {
  double max_step = 3.6;
  std::vector<double> delta;
  for (size_t i = 0; i < start.size(); ++i)
    delta.push_back(goal[i] - start[i]);
  double distance = 0.0;
  for (double diff : delta)
    distance += fabs(diff);
  if (distance < max_step) {
    for (const auto& pos : goal)
      printf("%lf ", pos);
    printf("\n");
  } else {
    int steps = static_cast<int>(distance / max_step + 1);
    for (size_t i = 0; i < delta.size(); ++i)
      delta[i] /= steps;
    for (int i = 1; i <= steps; ++i) {
      for (size_t j = 0; j < start.size(); ++j)
        printf("%lf ", start[j] + i * delta[j]);
      printf("\n");
    }
  }
}

void OutputPath(std::vector<std::shared_ptr<TreePoint> > points) {
  if (!points.empty())
    DrawLine(points[0]->position(), points[0]->position());
  for (size_t i = 1; i < points.size(); ++i)
    DrawLine(points[i - 1]->position(), points[i]->position());
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
  if (argc < 2) {
    fprintf(stdout, "%s: %s\n",
        google::ProgramInvocationShortName(), google::ProgramUsage());
    return 2;
  }

  Rrt rrt(argv[1]);
  int step = 0;
  bool done = false;
  std::vector<long int> durations;
  while (!done) {
    durations.push_back(static_cast<long int>(
        TimeMeasure<std::chrono::microseconds>::Run(
          [&retval = done, &obj = rrt]{retval = obj.Step();})));
    done = rrt.Step();
    ++step;
    if (FLAGS_verbose)
      fprintf(stderr, "Current step: %6d\n", step);
  }
  if (FLAGS_output_type == "path")
    OutputPath(rrt.GetSolution());
  else if (FLAGS_output_type == "times")
    OutputTimes(durations);
  return 0;
}
