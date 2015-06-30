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

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE HaltonSequenceGenerator
#include "bubblesmp/generators/halton-sequence-generator.h"
#include <boost/test/unit_test.hpp>

using ::com::ademovic::bubblesmp::generators::HaltonSequenceGenerator;

BOOST_AUTO_TEST_CASE(one_dimension) {
  HaltonSequenceGenerator generator_2({{8.0, 40.0}}, {2}, 0);

  BOOST_CHECK_CLOSE(generator_2.NextPoint()[0], 24.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_2.NextPoint()[0], 16.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_2.NextPoint()[0], 32.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_2.NextPoint()[0], 12.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_2.NextPoint()[0], 28.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_2.NextPoint()[0], 20.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_2.NextPoint()[0], 36.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_2.NextPoint()[0], 10.0, 0.000001);

  HaltonSequenceGenerator generator_3({{3.0, 30.0}}, {3}, 0);

  BOOST_CHECK_CLOSE(generator_3.NextPoint()[0], 12.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_3.NextPoint()[0], 21.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_3.NextPoint()[0],  6.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_3.NextPoint()[0], 15.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_3.NextPoint()[0], 24.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_3.NextPoint()[0],  9.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_3.NextPoint()[0], 18.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_3.NextPoint()[0], 27.0, 0.000001);

  HaltonSequenceGenerator generator_17({{3.0, 20.0}}, {17}, 0);

  BOOST_CHECK_CLOSE(generator_17.NextPoint()[0],  4.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_17.NextPoint()[0],  5.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_17.NextPoint()[0],  6.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_17.NextPoint()[0],  7.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_17.NextPoint()[0],  8.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_17.NextPoint()[0],  9.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_17.NextPoint()[0], 10.0, 0.000001);
  BOOST_CHECK_CLOSE(generator_17.NextPoint()[0], 11.0, 0.000001);
}

BOOST_AUTO_TEST_CASE(one_dimension_delay) {
  HaltonSequenceGenerator generator({{8.0, 40.0}}, {5}, 0);
  for (unsigned i = 0; i < 12; ++i)
    generator.NextPoint();
  HaltonSequenceGenerator generator_delayed({{8.0, 40.0}}, {5}, 12);
  for (unsigned i = 0; i < 20; ++i)
    BOOST_CHECK_CLOSE(
        generator.NextPoint()[0], generator_delayed.NextPoint()[0], 0.000001);
}

BOOST_AUTO_TEST_CASE(three_dimension) {
  HaltonSequenceGenerator generator(
      {{8.0, 40.0}, {3.0, 12.0}, {5.0, 30.0}}, {2, 3, 5}, 0);
  std::vector<double> point;
  point = generator.NextPoint();
  BOOST_CHECK_CLOSE(point[0], 24.0, 0.000001);
  BOOST_CHECK_CLOSE(point[1],  6.0, 0.000001);
  BOOST_CHECK_CLOSE(point[2], 10.0, 0.000001);
  point = generator.NextPoint();
  BOOST_CHECK_CLOSE(point[0], 16.0, 0.000001);
  BOOST_CHECK_CLOSE(point[1],  9.0, 0.000001);
  BOOST_CHECK_CLOSE(point[2], 15.0, 0.000001);
  point = generator.NextPoint();
  BOOST_CHECK_CLOSE(point[0], 32.0, 0.000001);
  BOOST_CHECK_CLOSE(point[1],  4.0, 0.000001);
  BOOST_CHECK_CLOSE(point[2], 20.0, 0.000001);
  point = generator.NextPoint();
  BOOST_CHECK_CLOSE(point[0], 12.0, 0.000001);
  BOOST_CHECK_CLOSE(point[1],  7.0, 0.000001);
  BOOST_CHECK_CLOSE(point[2], 25.0, 0.000001);
  point = generator.NextPoint();
  BOOST_CHECK_CLOSE(point[0], 28.0, 0.000001);
  BOOST_CHECK_CLOSE(point[1], 10.0, 0.000001);
  BOOST_CHECK_CLOSE(point[2],  6.0, 0.000001);
  point = generator.NextPoint();
  BOOST_CHECK_CLOSE(point[0], 20.0, 0.000001);
  BOOST_CHECK_CLOSE(point[1],  5.0, 0.000001);
  BOOST_CHECK_CLOSE(point[2], 11.0, 0.000001);
  point = generator.NextPoint();
  BOOST_CHECK_CLOSE(point[0], 36.0, 0.000001);
  BOOST_CHECK_CLOSE(point[1],  8.0, 0.000001);
  BOOST_CHECK_CLOSE(point[2], 16.0, 0.000001);
  point = generator.NextPoint();
  BOOST_CHECK_CLOSE(point[0], 10.0, 0.000001);
  BOOST_CHECK_CLOSE(point[1], 11.0, 0.000001);
  BOOST_CHECK_CLOSE(point[2], 21.0, 0.000001);
}
