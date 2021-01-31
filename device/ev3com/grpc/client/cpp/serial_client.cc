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

#include "serial.grpc.pb.h"
#include "serial_client.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using serial::SerialPutData;
using serial::SerialPutResult;
using serial::SerialGetData;
using serial::SerialGetResult;
using serial::SerialService;

class SerialServiceClient {
 public:
  SerialServiceClient(std::shared_ptr<Channel> channel)
      : stub_(SerialService::NewStub(channel)) {}

  ErcdType PutData(ChannelType channel, std::string indata) {
    SerialPutData request;
    request.set_channel(channel);
    request.set_data(indata);
    SerialPutResult reply;
    ClientContext context;

    Status status = stub_->PutData(&context, request, &reply);

    if (status.ok()) {
      return Ercd_OK;
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
      return Ercd_NG;
    }
  }
  ErcdType GetData(ChannelType channel, char *outdata, int len) {
    SerialGetData request;
    request.set_channel(channel);
    SerialGetResult reply;
    ClientContext context;

    Status status = stub_->GetData(&context, request, &reply);

    if (status.ok()) {
      if (len > reply.data().length()) {
        memcpy(outdata, reply.data().c_str(), reply.data().length());
        outdata[reply.data().length()] = '\0';
      }
      else {
        std::cout << "lost data.."  << std::endl;
        memcpy(outdata, reply.data().c_str(), (len -1));
        outdata[len -1] = '\0';
      }
      return Ercd_OK;
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
      return Ercd_NG;
    }
  }

 private:
  std::unique_ptr<SerialService::Stub> stub_;
};

static SerialServiceClient *gl_client;

void serial_client_init(void)
{
  std::string target_str;
  target_str = "localhost:50051";
  static SerialServiceClient client(grpc::CreateChannel(
      target_str, grpc::InsecureChannelCredentials()));
  gl_client = &client;
  return;
}

ErcdType serial_client_put_data(ChannelType channel, const char* indata)
{
  std::string str(indata);
  ErcdType ercd = gl_client->PutData(channel, str);
  std::cout << "Client PutData reply received: " << std::endl;
  return ercd;
}
ErcdType serial_client_get_data(ChannelType channel, char* outdata, int len)
{
  ErcdType ercd = gl_client->GetData(channel, outdata, len);
  std::cout << "Client GetData reply received: " << std::endl;
  return ercd;
}