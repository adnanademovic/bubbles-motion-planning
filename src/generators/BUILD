package(default_visibility = ["//visibility:public"])

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
