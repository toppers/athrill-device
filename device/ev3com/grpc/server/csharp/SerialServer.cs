using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using Google.Protobuf;
using Google.Protobuf.WellKnownTypes;
using Grpc.Core;
using GrpcSerial;

namespace BtSerial
{
    public class SerialServer : SerialService.SerialServiceBase
    {
        private static Server server;
        static public void StartServer(string ipaddr, int portno, MemoryFifo in_fifo, MemoryFifo out_fifo)
        {
            if (SerialServer.server != null)
            {
                throw new InvalidOperationException();
            }
            SerialServer.server = new Server
            {
                Services = { SerialService.BindService(new SerialServer(in_fifo, out_fifo)) },
                Ports = { new ServerPort(ipaddr, portno, ServerCredentials.Insecure) }
            };
            server.Start();
        }
        static public void ShutdownServer()
        {
            SerialServer.server.ShutdownAsync().Wait();
            SerialServer.server = null;
        }
        private MemoryFifo in_fifo;
        private MemoryFifo out_fifo;
        public SerialServer(MemoryFifo inf, MemoryFifo outf) : base()
        {
            this.in_fifo = inf;
            this.out_fifo = outf;
        }
        public override Task<SerialPutResult> PutData(SerialPutData request, ServerCallContext context)
        {
            //Console.WriteLine("[PUT] " + request.Data.ToStringUtf8());
            this.in_fifo.Write(request.Data.ToByteArray());
            //Console.WriteLine("len=" + this.input_stream.Length);

            return Task.FromResult(new SerialPutResult
            {
                Channel = 1,
                Ercd = "OK"
            });
        }
        public override Task<SerialGetResult> GetData(SerialGetData request, ServerCallContext context)
        {
            string data = this.out_fifo.ReadLine();
            //Console.WriteLine("[GET] channel= " + request.Channel.ToString() + " data=" + data);

            return Task.FromResult(new SerialGetResult
            {
                Channel = 1,
                Ercd = "OK",
                Data = ByteString.CopyFrom(data, Encoding.ASCII)
            });
        }
    }
}
