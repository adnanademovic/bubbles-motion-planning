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

#ifndef COM_ADEMOVIC_BUBBLESMP_TRANSFORMS_TRANSFORMATION_H_
#define COM_ADEMOVIC_BUBBLESMP_TRANSFORMS_TRANSFORMATION_H_

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace transforms {

class Transformation {
 public:
  enum Axis {
    X = 0,
    Y = 1,
    Z = 2,
  };
  Transformation();
  static Transformation Rotation(Axis axis, double angle);
  static Transformation Translation(double x, double y, double z);
  void Transform(const Transformation& transform);
  double coefficient(int i, int j) const;

 private:
  double rotation[3][3];
  double translation[3];
};

}  // namespace transforms
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com

#endif  // COM_ADEMOVIC_BUBBLESMP_TRANSFORMS_TRANSFORMATION_H_
