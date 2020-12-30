// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: proto/sample.proto

#include "proto/sample.pb.h"
#include "proto/sample.grpc.pb.h"

#include <functional>
#include <grpcpp/impl/codegen/async_stream.h>
#include <grpcpp/impl/codegen/async_unary_call.h>
#include <grpcpp/impl/codegen/channel_interface.h>
#include <grpcpp/impl/codegen/client_unary_call.h>
#include <grpcpp/impl/codegen/client_callback.h>
#include <grpcpp/impl/codegen/message_allocator.h>
#include <grpcpp/impl/codegen/method_handler.h>
#include <grpcpp/impl/codegen/rpc_service_method.h>
#include <grpcpp/impl/codegen/server_callback.h>
#include <grpcpp/impl/codegen/server_callback_handlers.h>
#include <grpcpp/impl/codegen/server_context.h>
#include <grpcpp/impl/codegen/service_type.h>
#include <grpcpp/impl/codegen/sync_stream.h>
namespace example {

static const char* SampleService_method_names[] = {
  "/example.SampleService/Request",
};

std::unique_ptr< SampleService::Stub> SampleService::NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options) {
  (void)options;
  std::unique_ptr< SampleService::Stub> stub(new SampleService::Stub(channel));
  return stub;
}

SampleService::Stub::Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel)
  : channel_(channel), rpcmethod_Request_(SampleService_method_names[0], ::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  {}

::grpc::Status SampleService::Stub::Request(::grpc::ClientContext* context, const ::example::SampleRequest& request, ::example::SampleReply* response) {
  return ::grpc::internal::BlockingUnaryCall(channel_.get(), rpcmethod_Request_, context, request, response);
}

void SampleService::Stub::experimental_async::Request(::grpc::ClientContext* context, const ::example::SampleRequest* request, ::example::SampleReply* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall(stub_->channel_.get(), stub_->rpcmethod_Request_, context, request, response, std::move(f));
}

void SampleService::Stub::experimental_async::Request(::grpc::ClientContext* context, const ::example::SampleRequest* request, ::example::SampleReply* response, ::grpc::experimental::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create(stub_->channel_.get(), stub_->rpcmethod_Request_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::example::SampleReply>* SampleService::Stub::PrepareAsyncRequestRaw(::grpc::ClientContext* context, const ::example::SampleRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderFactory< ::example::SampleReply>::Create(channel_.get(), cq, rpcmethod_Request_, context, request, false);
}

::grpc::ClientAsyncResponseReader< ::example::SampleReply>* SampleService::Stub::AsyncRequestRaw(::grpc::ClientContext* context, const ::example::SampleRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncRequestRaw(context, request, cq);
  result->StartCall();
  return result;
}

SampleService::Service::Service() {
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      SampleService_method_names[0],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< SampleService::Service, ::example::SampleRequest, ::example::SampleReply>(
          [](SampleService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::example::SampleRequest* req,
             ::example::SampleReply* resp) {
               return service->Request(ctx, req, resp);
             }, this)));
}

SampleService::Service::~Service() {
}

::grpc::Status SampleService::Service::Request(::grpc::ServerContext* context, const ::example::SampleRequest* request, ::example::SampleReply* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}


}  // namespace example
