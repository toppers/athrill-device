# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: sample.proto

require 'google/protobuf'

Google::Protobuf::DescriptorPool.generated_pool.build do
  add_file("sample.proto", :syntax => :proto3) do
    add_message "example.SampleRequest" do
      optional :name, :string, 1
      optional :clock, :uint64, 2
    end
    add_message "example.SampleReply" do
      optional :message, :string, 1
      optional :ercd, :string, 2
    end
  end
end

module Example
  SampleRequest = ::Google::Protobuf::DescriptorPool.generated_pool.lookup("example.SampleRequest").msgclass
  SampleReply = ::Google::Protobuf::DescriptorPool.generated_pool.lookup("example.SampleReply").msgclass
end