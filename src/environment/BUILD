package(default_visibility = ["//visibility:public"])

cc_library(
    name = "environment-feedback-interface",
    hdrs = ["environment-feedback-interface.h"],
)

cc_library(
    name = "pqp-environment",
    srcs = ["pqp-environment.cc"],
    hdrs = ["pqp-environment.h"],
    deps = ["//bubbles-motion-planning/src/transforms:transformation"],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_serialization",
        "-lboost_system",
        "-lboost_thread",
        "-lPQP",
    ],
)

cc_library(
    name = "pqp-environment-feedback",
    srcs = ["pqp-environment-feedback.cc"],
    hdrs = ["pqp-environment-feedback.h"],
    deps = [
        ":environment-feedback-interface",
        ":pqp-environment",
        "//bubbles-motion-planning/src:bubble",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_serialization",
        "-lboost_system",
        "-lboost_thread",
        "-lPQP",
    ],
)

cc_test(
    name = "pqp-environment_test",
    srcs = ["pqp-environment_test.cc"],
    deps = [":pqp-environment"],
    linkopts = ["-lboost_unit_test_framework"],
)

cc_test(
    name = "pqp-environment-feedback_test",
    srcs = ["pqp-environment-feedback_test.cc"],
    deps = [":pqp-environment-feedback"],
    linkopts = ["-lboost_unit_test_framework"],
)
