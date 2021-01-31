all: 
	grpc_tools_ruby_protoc -I ./proto --ruby_out=lib/ruby --grpc_out=lib/ruby ./proto/serial.proto

clean:
	rm -f lib/ruby/*
