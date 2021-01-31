# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: serial.proto

require 'google/protobuf'

Google::Protobuf::DescriptorPool.generated_pool.build do
  add_file("serial.proto", :syntax => :proto3) do
    add_message "serial.SerialPutData" do
      optional :channel, :int32, 1
      optional :data, :string, 2
    end
    add_message "serial.SerialPutResult" do
      optional :channel, :int32, 1
      optional :ercd, :string, 2
    end
    add_message "serial.SerialGetData" do
      optional :channel, :int32, 1
    end
    add_message "serial.SerialGetResult" do
      optional :channel, :int32, 1
      optional :ercd, :string, 2
      optional :data, :string, 3
    end
  end
end

module Serial
  SerialPutData = ::Google::Protobuf::DescriptorPool.generated_pool.lookup("serial.SerialPutData").msgclass
  SerialPutResult = ::Google::Protobuf::DescriptorPool.generated_pool.lookup("serial.SerialPutResult").msgclass
  SerialGetData = ::Google::Protobuf::DescriptorPool.generated_pool.lookup("serial.SerialGetData").msgclass
  SerialGetResult = ::Google::Protobuf::DescriptorPool.generated_pool.lookup("serial.SerialGetResult").msgclass
end
