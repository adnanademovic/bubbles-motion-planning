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

cc_library(
    name = "environment-interface",
    hdrs = ["environment-interface.h"],
)

cc_library(
    name = "environment-feedback",
    srcs = ["environment-feedback.cc"],
    hdrs = ["environment-feedback.h"],
    deps = [
        ":environment-interface",
        "//bubbles-motion-planning/src:bubble",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_serialization",
        "-lboost_system",
        "-lboost_thread",
    ],
)

cc_library(
    name = "fcl-environment",
    srcs = ["fcl-environment.cc"],
    hdrs = ["fcl-environment.h"],
    deps = [
        ":environment-interface",
        ":environment-proto",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_thread",
        "-lfcl",
        "-lglog",
        "-lm",
        "-lprotobuf",
    ],
)

cc_library(
    name = "make-environment",
    srcs = ["make-environment.cc"],
    hdrs = ["make-environment.h"],
    deps = [
        ":environment-interface",
        ":environment-proto",
        ":fcl-environment",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_thread",
        "-lglog",
        "-lprotobuf",
    ],
)

# TODO: Replace with proper protobuffer rules when implemented in Bazel.
cc_library(
    name = "environment-proto",
    srcs = ["environment.pb.cc"],
    hdrs = ["environment.pb.h"],
    linkopts = ["-lprotobuf"],
)

cc_test(
    name = "environment-feedback_test",
    srcs = ["environment-feedback_test.cc"],
    deps = [
        ":environment-feedback",
        ":make-environment",
    ],
    linkopts = ["-lboost_unit_test_framework"],
    size = "small",
)

cc_test(
    name = "fcl-environment_test",
    srcs = ["fcl-environment_test.cc"],
    deps = [":fcl-environment"],
    linkopts = ["-lboost_unit_test_framework"],
    size = "small",
)
