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

#include "bubblesmp/environment/make-environment.h"

#include <fstream>
#include <string>

#include <glog/logging.h>
#include <google/protobuf/text_format.h>

#include "bubblesmp/environment/environment.pb.h"
#include "bubblesmp/environment/environment-interface.h"
#include "bubblesmp/environment/fcl-environment.h"

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace environment {

EnvironmentInterface* NewEnvironmentFromProtoBuffer(
    const std::string& configuration) {
  boost::filesystem::path config_file_path(configuration);
  config_file_path.remove_filename();
  std::ifstream fin(configuration);
  EnvironmentConfig config_pb;
  std::string input_string((std::istreambuf_iterator<char>(fin)),
                           std::istreambuf_iterator<char>());
  bool success = google::protobuf::TextFormat::ParseFromString(
      input_string, &config_pb);
  fin.close();
  CHECK(success) << "Failed parsing file: " << configuration << std::endl;
  return NewEnvironmentFromProtoBuffer(config_pb, config_file_path);
}

EnvironmentInterface* NewEnvironmentFromProtoBuffer(
    const EnvironmentConfig& configuration,
    const boost::filesystem::path& config_file_path) {
  switch (configuration.type()) {
    case EnvironmentConfig::FCL:
      return new FclEnvironment(configuration, config_file_path);
      break;
    default:
      LOG(FATAL) << (configuration.has_type() ? "Unsuported" : "Missing")
                 << " type in EnvironmentConfig:" << std::endl
                 << configuration.DebugString();
  }
}

}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
