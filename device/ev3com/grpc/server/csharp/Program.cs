using System;
using System.Threading.Tasks;
using Google.Protobuf.WellKnownTypes;
using Grpc.Core;
using GrpcSerial;

namespace BtSerial
{
    class Program
    {
        static void Main(string[] args)
        {
            string ipaddr = "172.25.0.1";
            int portno = 50051;
            Console.WriteLine("ipaddr=" + ipaddr + " portno=" + portno.ToString());

            VirtualBluetoothSerial serial = new VirtualBluetoothSerial();
            serial.SetServerInfo(ipaddr, portno, 1024*1024);
            serial.Open();

            serial.WriteLine("Hello");


            Console.WriteLine("Press any key to shutdown the server...");
            Console.ReadKey();
            string data = serial.ReadLine();

            Console.WriteLine("INPUT:" + data);

            serial.Close();
        }
    }

}
