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
)

cc_test(
    name = "fcl-environment_test",
    srcs = ["fcl-environment_test.cc"],
    deps = [":fcl-environment"],
    linkopts = ["-lboost_unit_test_framework"],
)
