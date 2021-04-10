cmake_minimum_required(VERSION 3.5.1)

project(HakoniwaDeviceProtobuf C CXX)

if(NOT MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
  add_definitions(-D_WIN32_WINNT=0x600)
endif()

find_package(Threads REQUIRED)

# This branch assumes that gRPC and all its dependencies are already installed
# on this system, so they can be located by find_package().

# Find Protobuf installation
# Looks for protobuf-config.cmake file installed by Protobuf's cmake installation.
set(protobuf_MODULE_COMPATIBLE TRUE)
find_package(Protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${Protobuf_VERSION}")

set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
set(_REFLECTION gRPC::grpc++_reflection)
if(CMAKE_CROSSCOMPILING)
  find_program(_PROTOBUF_PROTOC protoc)
else()
  set(_PROTOBUF_PROTOC $<TARGET_FILE:protobuf::protoc>)
endif()

# Find gRPC installation
# Looks for gRPCConfig.cmake file installed by gRPC's cmake installation.
find_package(gRPC CONFIG REQUIRED)
message(STATUS "Using gRPC ${gRPC_VERSION}")

set(_GRPC_GRPCPP gRPC::grpc++)
if(CMAKE_CROSSCOMPILING)
  find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
else()
  set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:gRPC::grpc_cpp_plugin>)
endif()

#Packet
get_filename_component(hakoniwa_proto "/root/workspace/hakoniwa-core/spec/micon/hakoniwa_micon.proto" ABSOLUTE)
get_filename_component(hakoniwa_proto_path "${hakoniwa_proto}" PATH)
set(hakoniwa_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/hakoniwa_micon.pb.cc")
set(hakoniwa_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/hakoniwa_micon.pb.h")
add_custom_command(
      OUTPUT "${hakoniwa_proto_srcs}" "${hakoniwa_proto_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${hakoniwa_proto_path}"
        "${hakoniwa_proto}"
      DEPENDS "${hakoniwa_proto}")

# Include generated *.pb.h files
include_directories("${CMAKE_CURRENT_BINARY_DIR}")


add_library(hakoniwa_protobuf STATIC
	hakoniwa_packet.cc
	${hakoniwa_proto_srcs})
target_link_libraries(hakoniwa_protobuf
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF})

