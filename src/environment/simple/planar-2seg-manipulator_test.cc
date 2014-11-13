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
#define BOOST_TEST_MODULE Planar2SegManipulator
#include "planar-2seg-manipulator.h"
#include <boost/test/unit_test.hpp>
#include <boost/test/test_tools.hpp>

using ::com::ademovic::bubblesmp::environment::simple::Planar2SegManipulator;

BOOST_AUTO_TEST_CASE(constructor) {
  Planar2SegManipulator manipulator(1.0, 2.0);
  std::vector<double> input{0.0, 0.0};
  std::vector<double> output = manipulator.coordinates();

  BOOST_CHECK_EQUAL_COLLECTIONS(
      output.begin(), output.end(), input.begin(), input.end());
}

BOOST_AUTO_TEST_CASE(set_coordinates) {
  Planar2SegManipulator manipulator(1.0, 2.0);
  std::vector<double> input{0.2, 0.3};
  manipulator.set_coordinates({0.2, 0.3});
  std::vector<double> output = manipulator.coordinates();

  BOOST_CHECK_EQUAL_COLLECTIONS(
      output.begin(), output.end(), input.begin(), input.end());
}
