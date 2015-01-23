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

#ifndef COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_SIMPLE_BUBBLE_SOURCE_H_
#define COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_SIMPLE_BUBBLE_SOURCE_H_

#include <memory>
#include <mutex>
#include <vector>

#include "../bubble-source-interface.h"
#include "obstacle-interface.h"
#include "robot-interface.h"

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace environment {
namespace simple {

class BubbleSource : public BubbleSourceInterface {
 public:
  // The class instance takes ownership of RobotInterface.
  BubbleSource(RobotInterface* robot);

  Bubble* NewBubble(const std::vector<double>& coordinates) const;
  bool IsCollision(const std::vector<double>& coordinates) const;

  // The class instance takes ownership of RobotInterface.
  void AddObstacle(ObstacleInterface* obstacle);
  void AddObstacle(std::shared_ptr<ObstacleInterface> obstacle);
  void ClearObstacles();

 private:
  std::unique_ptr<RobotInterface> robot_;
  std::unique_ptr<std::vector<std::shared_ptr<ObstacleInterface> > > obstacles_;

  mutable std::mutex object_mutex_;
};

}  // namespace simple
}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com

#endif  // COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_SIMPLE_BUBBLE_SOURCE_H_
