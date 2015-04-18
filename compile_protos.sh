#!/bin/bash

for PROTO_NAME in $(find . -wholename \*bubblesmp/\*.proto -print); do
  echo $PROTO_NAME
  protoc --cpp_out=./ --grpc_out=./ --plugin=protoc-gen-grpc=grpc_cpp_plugin -I./ $PROTO_NAME
done
