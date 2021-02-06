using System;
using System.Threading;
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
            string data = null;
            Console.WriteLine("start write:");
            for (int i = 0; i < 10; i++)
            {
                Console.WriteLine("OUTPUT: Hello World: " + i.ToString());
                serial.WriteLine("Hello World: " + i.ToString());
                data = serial.ReadLine();
                Console.WriteLine("INPUT:" + data);
            }
            Console.WriteLine("end write:");

            serial.Close();
        }
    }

}
