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

#ifndef COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_PQP_ENVIRONMENT_H_
#define COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_PQP_ENVIRONMENT_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <PQP/PQP.h>

#include "../transforms/transformation.h"

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace environment {

class PqpEnvironment {
 public:
  typedef std::vector<std::pair<double, std::vector<double> > > DistanceProfile;

  PqpEnvironment(const std::string& configuration);

  bool IsCollision(const std::vector<double>& q) const;
  DistanceProfile GetDistanceProfile(const std::vector<double>& q) const;

  int part_count() const;

 private:
  // Configuration consists of lines of "a alpha d theta" format
  void LoadDh(const std::vector<std::vector<double> >& configuration);

  int part_count_;
  double variance_;
  std::vector<std::unique_ptr<transforms::Transformation> > dh_parameters_;
  std::vector<std::unique_ptr<transforms::Transformation> > dh_inverted_;
  std::vector<bool> is_joint_start_;
  std::vector<std::unique_ptr<PQP_Model> > parts_;
  std::vector<std::pair<std::vector<double>, double> > cylinders_;
  std::unique_ptr<PQP_Model> environment_;

  mutable std::mutex guard_mutex_;
};

}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com

#endif  // COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_PQP_ENVIRONMENT_H_
