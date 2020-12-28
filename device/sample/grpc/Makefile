CXX_IFLAGS	:= -I./lib/cpp
CXX_IFLAGS	+= -I./lib/cpp/proto

CXX_WFLAGS	:= -Wall

CXX_OBJS	:= sample.pb.o
CXX_OBJS	+= sample.grpc.pb.o
CXX_OBJS	+= sample_client.o

VPATH		:= ./lib/cpp/proto
VPATH		+= ./client/cpp

TARGET		:= cpp_client

all: grpc_proto $(CXX_OBJS)
	clang++ -L/usr/local/lib `pkg-config --libs  protobuf grpc++` $(CXX_OBJS) -o $(TARGET) -lpthread -lgrpc++_reflection -ldl

grpc_proto:
	grpc_tools_ruby_protoc -I ./proto --ruby_out=lib/ruby --grpc_out=lib/ruby ./proto/sample.proto
	protoc --cpp_out=lib/cpp --grpc_out=lib/cpp --plugin=protoc-gen-grpc=/usr/local/bin/grpc_cpp_plugin ./proto/sample.proto

.cc.o:
	clang++ -std=c++11 `pkg-config --cflags  protobuf` -g $(CXX_IFLAGS) $(CXX_WFLAGS) -c $<

clean:
	rm -f lib/ruby/*
	rm -f lib/cpp/*
