
this_dir = File.expand_path(File.dirname(__FILE__))
lib_dir = File.join(this_dir, 'lib')
$LOAD_PATH.unshift(lib_dir) unless $LOAD_PATH.include?(lib_dir)

require 'grpc'
require 'serial_services_pb'

def main
  msg = 'Hello World!!'
  hostname = ARGV.size > 1 ?  ARGV[1] : 'localhost:50051'
  stub = Serial::SerialService::Stub.new(hostname, :this_channel_is_insecure)
  begin
    req = Serial::SerialPutData.new(channel: 1, data: msg)
    p req
    res = stub.put_data(req)

  rescue GRPC::BadStatus => e
    abort "ERROR: #{e.message}"
  end
end

main