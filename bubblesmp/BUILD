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
    name = "bertram-bubble-tree",
    srcs = ["bertram-bubble-tree.cc"],
    hdrs = ["bertram-bubble-tree.h"],
    deps = [
        ":bubble",
        ":rrt-tree",
        "//bubblesmp/environment:environment-feedback",
        "//third_party/google:glog",
    ],
)

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
        "//bubblesmp/environment:environment-feedback",
        "//third_party/google:glog",
    ],
)

cc_library(
    name = "classic-tree",
    srcs = ["classic-tree.cc"],
    hdrs = ["classic-tree.h"],
    deps = [
        ":rrt-tree",
        ":tree-node",
        "//bubblesmp/environment:environment-feedback",
        "//third_party/google:glog",
    ],
)

cc_library(
    name = "crawling-bubble-tree",
    srcs = ["crawling-bubble-tree.cc"],
    hdrs = ["crawling-bubble-tree.h"],
    deps = [
        ":bubble",
        ":rrt-tree",
        "//bubblesmp/environment:environment-feedback",
        "//third_party/google:glog",
    ],
)

cc_library(
    name = "greedy-bubble-tree",
    srcs = ["greedy-bubble-tree.cc"],
    hdrs = ["greedy-bubble-tree.h"],
    deps = [
        ":bubble",
        ":rrt-tree",
        "//bubblesmp/environment:environment-feedback",
        "//third_party/google:glog",
    ],
)

cc_library(
    name = "greedy-classic-tree",
    srcs = ["greedy-classic-tree.cc"],
    hdrs = ["greedy-classic-tree.h"],
    deps = [
        ":rrt-tree",
        ":tree-node",
        "//bubblesmp/environment:environment-feedback",
        "//third_party/google:glog",
    ],
)

cc_library(
    name = "point-index",
    srcs = ["point-index.cc"],
    hdrs = ["point-index.h"],
    deps = [
        ":index_proto",
        ":tree-node",
        "//third_party:flann",
    ],
)

cc_library(
    name = "rrt",
    srcs = ["rrt.cc"],
    hdrs = ["rrt.h"],
    deps = [
        ":bertram-bubble-tree",
        ":bubble-tree",
        ":classic-tree",
        ":crawling-bubble-tree",
        ":greedy-bubble-tree",
        ":greedy-classic-tree",
        ":rrt-tree",
        ":task_proto",
        ":tree-node",
        "//bubblesmp/environment:environment-feedback",
        "//bubblesmp/environment:make-environment",
        "//bubblesmp/generators:make-generator",
        "//bubblesmp/generators:random-point-generator-interface",
        "//third_party:pthread",
        "//third_party/boost:filesystem",
        "//third_party/google:glog",
        "//third_party/google:protobuf",
    ],
)

cc_library(
    name = "rrt-tree",
    srcs = ["rrt-tree.cc"],
    hdrs = ["rrt-tree.h"],
    deps = [
        ":point-index",
        ":tree-node",
    ],
)

cc_library(
    name = "tree-node",
    srcs = ["tree-node.cc"],
    hdrs = ["tree-node.h"],
)

proto_library(
    name = "index_proto",
    srcs = ["index.proto"],
)

proto_library(
    name = "task_proto",
    srcs = ["task.proto"],
    deps = [
        ":index_proto",
        "//bubblesmp/environment:environment_proto",
        "//bubblesmp/generators:generator_proto",
    ],
)

cc_binary(
    name = "rrtplan",
    srcs = ["rrtplan.cc"],
    deps = [
        ":rrt",
        ":tree-node",
        "//third_party:pthread",
        "//third_party/google:gflags",
        "//third_party/google:glog",
        "//third_party/google:protobuf",
    ],
)

cc_test(
    name = "bubble_test",
    srcs = ["bubble_test.cc"],
    deps = [
        ":bubble",
        "//third_party/boost:unit-test-framework",
        "//third_party:math",
    ],
    size = "small",
)
