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

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE BubbleSource
#include "../../bubble.h"
#include "bubble-source.h"
#include <boost/test/unit_test.hpp>
#include <boost/test/test_tools.hpp>

using ::com::ademovic::bubblesmp::Bubble;
using ::com::ademovic::bubblesmp::environment::simple::BubbleSource;
using ::com::ademovic::bubblesmp::environment::simple::ObstacleInterface;
using ::com::ademovic::bubblesmp::environment::simple::RobotInterface;

class MockRobot : public RobotInterface {
 public:
  virtual void set_coordinates(const std::vector<double>& coordinates) {};
  virtual std::vector<double> coordinates() const {
    return {0.0, 0.0, 0.0};
  };
  virtual double DistanceToObstacle(const ObstacleInterface& obstacle) const {
    return obstacle.DistanceToLine(0.0, 0.0, 0.0, 1.0, 1.0, 0.0);
  }
  virtual std::vector<double> FurthestDistances() const {
    return {4.0, 2.0, 1.0};
  }
};

class MockObstacle : public ObstacleInterface {
 public:
  MockObstacle(double dist)
      : dist_(dist) {}
  double DistanceToLine(double x1, double y1, double z1,
      double x2, double y2, double z2) const {
    return dist_;
  }

 private:
  double dist_;
};

BOOST_AUTO_TEST_CASE(connects_properly) {
  BubbleSource source(new MockRobot);
  source.AddObstacle(new MockObstacle(5.0));
  source.AddObstacle(std::shared_ptr<ObstacleInterface>(new MockObstacle(2.0)));
  std::vector<double> input{0.5, 1.0, 2.0};
  std::unique_ptr<Bubble> output(source.NewBubble({0.0, 0.0, 0.0}));
  std::vector<double> output_size(output->size());
  BOOST_CHECK_EQUAL_COLLECTIONS(output_size.begin(), output_size.end(),
                                input.begin(), input.end());
}

BOOST_AUTO_TEST_CASE(clear_obstacles) {
  BubbleSource source(new MockRobot);
  source.AddObstacle(new MockObstacle(5.0));
  source.AddObstacle(new MockObstacle(2.0));
  source.ClearObstacles();
  source.AddObstacle(new MockObstacle(6.0));
  std::vector<double> input{1.5, 3.0, 6.0};
  std::unique_ptr<Bubble> output(source.NewBubble({0.0, 0.0, 0.0}));
  std::vector<double> output_size(output->size());
  BOOST_CHECK_EQUAL_COLLECTIONS(output_size.begin(), output_size.end(),
                                input.begin(), input.end());
}
