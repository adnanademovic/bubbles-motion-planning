#
# Copyright (c) 2015, Adnan Ademovic
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

licenses(["notice"])  # FreeBSD

package(default_visibility = ["//visibility:public"])

load("/tools/protobuf/protobuf", "proto_library")

cc_library(
    name = "halton-sequence-generator",
    srcs = ["halton-sequence-generator.cc"],
    hdrs = ["halton-sequence-generator.h"],
    deps = [":random-point-generator-interface"],
)

cc_library(
    name = "make-generator",
    srcs = ["make-generator.cc"],
    hdrs = ["make-generator.h"],
    deps = [
        ":generator_proto",
        ":halton-sequence-generator",
        ":random-point-generator-interface",
        ":simple-generator",
        "//third_party/google:glog",
    ],
)

cc_library(
    name = "random-point-generator-interface",
    hdrs = ["random-point-generator-interface.h"],
)

cc_library(
    name = "simple-generator",
    srcs = ["simple-generator.cc"],
    hdrs = ["simple-generator.h"],
    deps = [":random-point-generator-interface"],
)

proto_library(
    name = "generator_proto",
    srcs = ["generator.proto"],
)

cc_test(
    name = "halton-sequence-generator_test",
    srcs = ["halton-sequence-generator_test.cc"],
    deps = [
        ":halton-sequence-generator",
        "//third_party/boost:unit-test-framework",
    ],
    size = "small",
)
