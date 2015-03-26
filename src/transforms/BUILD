package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transformation",
    srcs = ["transformation.cc"],
    hdrs = ["transformation.h"],
)

cc_test(
    name = "transformation_test",
    srcs = ["transformation_test.cc"],
    deps = [":transformation"],
    linkopts = ["-lboost_unit_test_framework"],
)
