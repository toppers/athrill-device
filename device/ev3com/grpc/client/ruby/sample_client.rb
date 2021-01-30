
this_dir = File.expand_path(File.dirname(__FILE__))
lib_dir = File.join(this_dir, 'lib')
$LOAD_PATH.unshift(lib_dir) unless $LOAD_PATH.include?(lib_dir)

require 'grpc'
require 'sample_services_pb'

def main
  user = ARGV.size > 0 ?  ARGV[0] : 'world'
  hostname = ARGV.size > 1 ?  ARGV[1] : 'localhost:50051'
  stub = Example::SampleService::Stub.new(hostname, :this_channel_is_insecure)
  begin
    p "Start: user=" + user
    req = Example::SampleRequest.new(name: user)
    p req
    res = stub.request(req)
    #p "Greeting:" + res
  rescue GRPC::BadStatus => e
    abort "ERROR: #{e.message}"
  end
end

main