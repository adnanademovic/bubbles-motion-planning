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

#include "bubblesmp/generators/make-generator.h"

#include <glog/logging.h>

#include "bubblesmp/generators/generator.pb.h"
#include "bubblesmp/generators/halton-sequence-generator.h"
#include "bubblesmp/generators/random-point-generator-interface.h"
#include "bubblesmp/generators/simple-generator.h"

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace generators {

RandomPointGeneratorInterface* NewGeneratorFromProtoBuffer(
    const std::vector<std::pair<double, double> >& limits,
    const GeneratorSettings& settings) {
  switch (settings.type()) {
    case GeneratorSettings::SIMPLE:
      return settings.has_seed()
          ? new SimpleGenerator(limits, settings.seed())
          : new SimpleGenerator(limits);
      break;
    case GeneratorSettings::HALTON:
      {
        std::vector<unsigned> keys;
        for (unsigned key : settings.keys())
          keys.push_back(key);
        CHECK_EQ(keys.size(), limits.size())
            << "The number of keys for the Halton sequence must be equal to "
            << "the number of dimensions of the space";
        return settings.has_seed()
            ? new HaltonSequenceGenerator(limits, keys, settings.seed())
            : new HaltonSequenceGenerator(limits, keys);
      }
      break;
    default:
      LOG(FATAL) << (settings.has_type() ? "Unsuported" : "Missing")
                 << " type in GeneratorSettings:" << std::endl
                 << settings.DebugString();
  }
}

}  // namespace generators
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
