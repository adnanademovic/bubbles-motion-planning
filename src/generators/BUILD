package(default_visibility = ["//visibility:public"])

cc_library(
    name = "make-generator",
    srcs = ["make-generator.cc"],
    hdrs = ["make-generator.h"],
    deps = [
        ":simple-generator",
        ":random-point-generator-interface",
    ],
    data = [":generator-proto"],
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

# TODO: Replace with proper protobuffer rules when implemented in Bazel.
cc_library(
    name = "generator-proto",
    srcs = ["generator.pb.cc"],
    hdrs = ["generator.pb.h"],
)
