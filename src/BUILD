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
    deps = [":tree-node"],
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
        ":rrt-tree",
        "//bubbles-motion-planning/src/generators" +
            ":random-point-generator-interface",
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

cc_binary(
    name = "bubble-main",
    srcs = ["bubble-main.cc"],
    deps = [
        ":bubble-tree",
        ":rrt",
        "//bubbles-motion-planning/src/environment:fcl-environment",
        "//bubbles-motion-planning/src/environment:environment-feedback",
        "//bubbles-motion-planning/src/generators:simple-generator",
    ],
    linkopts = [
        "-lgflags",
        "-lm",
    ],
    data = ["//motion-planning-data/abb-irb-120:cases"],
)

cc_binary(
    name = "classic-main",
    srcs = ["classic-main.cc"],
    deps = [
        ":classic-tree",
        ":rrt",
        "//bubbles-motion-planning/src/environment:fcl-environment",
        "//bubbles-motion-planning/src/environment:environment-feedback",
        "//bubbles-motion-planning/src/generators:simple-generator",
    ],
    linkopts = [
        "-lgflags",
        "-lm",
    ],
    data = ["//motion-planning-data/abb-irb-120:cases"],
)

cc_binary(
    name = "compare-main",
    srcs = ["compare-main.cc"],
    deps = [
        ":bubble-tree",
        ":classic-tree",
        ":rrt",
        "//bubbles-motion-planning/src/environment:fcl-environment",
        "//bubbles-motion-planning/src/environment:environment-feedback",
        "//bubbles-motion-planning/src/generators:simple-generator",
    ],
    linkopts = [
        "-lgflags",
        "-lm",
    ],
    data = ["//motion-planning-data/abb-irb-120:cases"],
)

cc_binary(
    name = "twoseg-bubble-main",
    srcs = ["twoseg-bubble-main.cc"],
    deps = [
        ":bubble-tree",
        ":rrt",
        "//bubbles-motion-planning/src/environment:fcl-environment",
        "//bubbles-motion-planning/src/environment:environment-feedback",
        "//bubbles-motion-planning/src/generators:simple-generator",
    ],
    linkopts = [
        "-lgflags",
        "-lm",
    ],
)

cc_binary(
    name = "twoseg-classic-main",
    srcs = ["twoseg-classic-main.cc"],
    deps = [
        ":classic-tree",
        ":rrt",
        "//bubbles-motion-planning/src/environment:fcl-environment",
        "//bubbles-motion-planning/src/environment:environment-feedback",
        "//bubbles-motion-planning/src/generators:simple-generator",
    ],
    linkopts = [
        "-lgflags",
        "-lm",
    ],
)

cc_binary(
    name = "twoseg-compare-main",
    srcs = ["twoseg-compare-main.cc"],
    deps = [
        ":bubble-tree",
        ":classic-tree",
        ":rrt",
        "//bubbles-motion-planning/src/environment:fcl-environment",
        "//bubbles-motion-planning/src/environment:environment-feedback",
        "//bubbles-motion-planning/src/generators:simple-generator",
    ],
    linkopts = [
        "-lgflags",
        "-lm",
    ],
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
