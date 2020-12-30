/*
 *
 * Copyright 2015 gRPC authors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <iostream>
#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>

#include "sample.grpc.pb.h"
#include "sample_client.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using example::SampleRequest;
using example::SampleReply;
using example::SampleService;

class SampleClient {
 public:
  SampleClient(std::shared_ptr<Channel> channel)
      : stub_(SampleService::NewStub(channel)) {}

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  std::string Request(const std::string& user) {
    // Data we are sending to the server.
    SampleRequest request;
    request.set_name(user);

    // Container for the data we expect from the server.
    SampleReply reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    Status status = stub_->Request(&context, request, &reply);

    // Act upon its status.
    if (status.ok()) {
      return reply.message();
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
      return "RPC failed";
    }
  }

 private:
  std::unique_ptr<SampleService::Stub> stub_;
};
#if 0
int main(int argc, char** argv) 
{
  sample_client_init();
  sample_client_request(NULL);
  return 0;
}
#endif

static SampleClient *gl_client;

void sample_client_init(void)
{
  std::string target_str;
  target_str = "localhost:50051";
  static SampleClient client(grpc::CreateChannel(
      target_str, grpc::InsecureChannelCredentials()));
  gl_client = &client;
  return;
}

void sample_client_request(const char* strp)
{
  std::string user("world");
  std::string reply = gl_client->Request(user);
  std::cout << "Client received: " << reply << std::endl;
  return;
}
