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
    name = "bubble",
    srcs = ["bubble.cc"],
    hdrs = ["bubble.h"],
    deps = [":tree-node"],
)

cc_library(
    name = "bubble-tree",
    srcs = ["bubble-tree.cc"],
    hdrs = ["bubble-tree.h"],
    deps = [
        ":bubble",
        ":rrt-tree",
        "//bubbles-motion-planning/src/environment:environment-feedback",
    ],
)

cc_library(
    name = "classic-tree",
    srcs = ["classic-tree.cc"],
    hdrs = ["classic-tree.h"],
    deps = [
        ":rrt-tree",
        "//bubbles-motion-planning/src/environment:environment-feedback",
    ],
)

cc_library(
    name = "point-index",
    srcs = ["point-index.cc"],
    hdrs = ["point-index.h"],
    deps = [
        ":index-proto",
        ":tree-node",
    ],
    linkopts = [
        "-lflann_cpp",
        "-lgflags",
    ],
)

cc_library(
    name = "rrt",
    srcs = ["rrt.cc"],
    hdrs = ["rrt.h"],
    deps = [
        ":bubble-tree",
        ":classic-tree",
        ":rrt-tree",
        ":task-proto",
        "//bubbles-motion-planning/src/environment:environment-feedback",
        "//bubbles-motion-planning/src/environment:make-environment",
        "//bubbles-motion-planning/src/generators:make-generator",
        "//bubbles-motion-planning/src/generators" +
            ":random-point-generator-interface",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_thread",
        "-lprotobuf",
    ],
)

cc_library(
    name = "rrt-tree",
    srcs = ["rrt-tree.cc"],
    hdrs = ["rrt-tree.h"],
    deps = [":point-index"],
)

cc_library(
    name = "tree-node",
    srcs = ["tree-node.cc"],
    hdrs = ["tree-node.h"],
)

# TODO: Replace with proper protobuffer rules when implemented in Bazel.
cc_library(
    name = "index-proto",
    srcs = ["index.pb.cc"],
    hdrs = ["index.pb.h"],
    linkopts = ["-lprotobuf"],
)

# TODO: Replace with proper protobuffer rules when implemented in Bazel.
cc_library(
    name = "task-proto",
    srcs = ["task.pb.cc"],
    hdrs = ["task.pb.h"],
    deps = [
        ":index-proto",
        "//bubbles-motion-planning/src/environment:environment-proto",
        "//bubbles-motion-planning/src/generators:generator-proto",
    ],
    linkopts = ["-lprotobuf"],
)

cc_binary(
    name = "bubble-main",
    srcs = ["bubble-main.cc"],
    deps = [
        ":bubble-tree",
        ":rrt",
        "//bubbles-motion-planning/src/environment:make-environment",
        "//bubbles-motion-planning/src/environment:environment-feedback",
        "//bubbles-motion-planning/src/generators:generator-proto",
        "//bubbles-motion-planning/src/generators:make-generator",
    ],
    linkopts = ["-lgflags"],
    data = ["//motion-planning-data/abb-irb-120:cases"],
)

cc_binary(
    name = "classic-main",
    srcs = ["classic-main.cc"],
    deps = [
        ":classic-tree",
        ":rrt",
        "//bubbles-motion-planning/src/environment:make-environment",
        "//bubbles-motion-planning/src/environment:environment-feedback",
        "//bubbles-motion-planning/src/generators:generator-proto",
        "//bubbles-motion-planning/src/generators:make-generator",
    ],
    linkopts = ["-lgflags"],
    data = ["//motion-planning-data/abb-irb-120:cases"],
)

cc_binary(
    name = "compare-main",
    srcs = ["compare-main.cc"],
    deps = [
        ":bubble-tree",
        ":classic-tree",
        ":rrt",
        "//bubbles-motion-planning/src/environment:make-environment",
        "//bubbles-motion-planning/src/environment:environment-feedback",
        "//bubbles-motion-planning/src/generators:generator-proto",
        "//bubbles-motion-planning/src/generators:make-generator",
    ],
    linkopts = ["-lgflags"],
    data = ["//motion-planning-data/abb-irb-120:cases"],
)

cc_binary(
    name = "twoseg-bubble-main",
    srcs = ["twoseg-bubble-main.cc"],
    deps = [
        ":bubble-tree",
        ":rrt",
        "//bubbles-motion-planning/src/environment:make-environment",
        "//bubbles-motion-planning/src/environment:environment-feedback",
        "//bubbles-motion-planning/src/generators:generator-proto",
        "//bubbles-motion-planning/src/generators:make-generator",
    ],
    linkopts = ["-lgflags"],
)

cc_binary(
    name = "twoseg-classic-main",
    srcs = ["twoseg-classic-main.cc"],
    deps = [
        ":classic-tree",
        ":rrt",
        "//bubbles-motion-planning/src/environment:make-environment",
        "//bubbles-motion-planning/src/environment:environment-feedback",
        "//bubbles-motion-planning/src/generators:generator-proto",
        "//bubbles-motion-planning/src/generators:make-generator",
    ],
    linkopts = ["-lgflags"],
)

cc_binary(
    name = "twoseg-compare-main",
    srcs = ["twoseg-compare-main.cc"],
    deps = [
        ":bubble-tree",
        ":classic-tree",
        ":rrt",
        "//bubbles-motion-planning/src/environment:make-environment",
        "//bubbles-motion-planning/src/environment:environment-feedback",
        "//bubbles-motion-planning/src/generators:generator-proto",
        "//bubbles-motion-planning/src/generators:make-generator",
    ],
    linkopts = ["-lgflags"],
)

cc_binary(
    name = "rrtplan",
    srcs = ["rrtplan.cc"],
    deps = [":rrt"],
    linkopts = ["-lgflags"],
    data = ["//motion-planning-data/abb-irb-120:tasks"],
)

cc_test(
    name = "bubble_test",
    srcs = ["bubble_test.cc"],
    deps = [":bubble"],
    linkopts = [
        "-lboost_unit_test_framework",
        "-lm",
    ],
)
