
this_dir = File.expand_path(File.dirname(__FILE__))
lib_dir = File.join(this_dir, 'lib')
$LOAD_PATH.unshift(lib_dir) unless $LOAD_PATH.include?(lib_dir)

require 'grpc'
require 'serial_services_pb'

class SerialServer < Serial::SerialService::Service

  def put_data(req, _unused_call)
    p sprintf("PUT:: channel=%d data=%s", req.channel, req.data)
    #puts "aaaa"
    Serial::SerialPutResult.new(channel: 1, ercd: "OK")
  end
  # TODO get_data
  def get_data(req, _unused_call)
    p sprintf("GET:: channel=%d", req.channel)

    Serial::SerialGetResult.new(channel: 1, data: "Hello world from tmori", ercd: "OK")
  end

end

# main starts an RpcServer that receives requests to GreeterServer at the sample
# server port.
def main
  server_port = '0.0.0.0:50051'
  puts "Serial Server " + server_port + ": UP"
  s = GRPC::RpcServer.new
  s.add_http2_port(server_port, :this_port_is_insecure)
  s.handle(SerialServer)
  # Runs the server with SIGHUP, SIGINT and SIGQUIT signal handlers to 
  #   gracefully shutdown.
  # User could also choose to run server via call to run_till_terminated
  s.run_till_terminated_or_interrupted([1, 'int', 'SIGQUIT'])
end

main
