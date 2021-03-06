# Tools for proto compilation

def proto_library(name, srcs, deps=None, cc_api_version=None, visibility=None):
  if not deps: deps = []
  hdrs = []
  ccs = []
  for p in srcs:
    if (not p.endswith(".proto")):
      fail("Proto must end in \".proto\"", p)
    short_p = p.replace(".proto", "")
    hdrs += [short_p + ".pb.h"]
    ccs += [short_p + ".pb.cc"]
  protogen_name = name + "_genprotoc"
  native.genrule(
    name = protogen_name,
    srcs = srcs,
    outs = hdrs + ccs,
    cmd = "protoc --proto_path=. --cpp_out=$(GENDIR) $(SRCS)",
    tools = deps,
    visibility = visibility,
  )
  native.cc_library(
    name = name,
    hdrs = hdrs,
    srcs = [protogen_name],
    deps = ["//third_party/google:protobuf"] + deps,
    visibility = visibility,
  )
