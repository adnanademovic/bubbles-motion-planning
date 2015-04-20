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

#ifndef COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_FCL_ENVIRONMENT_H_
#define COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_FCL_ENVIRONMENT_H_

#include <memory>
#include <mutex>
#include <string>

#define BOOST_SPIRIT_THREADSAFE
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>

#include <fcl/collision.h>

#include "bubblesmp/environment/environment.pb.h"
#include "environment-interface.h"

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace environment {

class FclEnvironment : public EnvironmentInterface {
 public:
  FclEnvironment(const std::string& configuration);
  FclEnvironment(const EnvironmentConfig& configuration,
                 const boost::filesystem::path& config_file_path);
  virtual ~FclEnvironment() {}

  bool IsCollision(const std::vector<double>& q) const;
  EnvironmentInterface::DistanceProfile GetDistanceProfile(
      const std::vector<double>& q) const;
  std::vector<std::pair<double, double> > GetAngleRanges() const;

 private:
  void LoadDhAndRanges(const Robot& robot);
  void ConfigureFromPB(const EnvironmentConfig& configuration,
                       const boost::filesystem::path& config_file_path);

  int part_count_;
  double variance_;
  std::vector<std::unique_ptr<fcl::Transform3f> > dh_parameters_;
  std::vector<std::unique_ptr<fcl::Transform3f> > dh_inverted_;
  std::vector<bool> is_joint_start_;
  std::vector<boost::shared_ptr<fcl::CollisionGeometry> > part_meshes_;
  std::vector<std::unique_ptr<fcl::CollisionObject> > parts_;
  std::vector<std::pair<fcl::Vec3f, double> > cylinders_;
  boost::shared_ptr<fcl::CollisionGeometry> environment_mesh_;
  std::unique_ptr<fcl::CollisionObject> environment_;
  std::vector<std::pair<double, double> > limits_;

  mutable std::mutex guard_mutex_;
};

}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com

#endif  // COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_FCL_ENVIRONMENT_H_
