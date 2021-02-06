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
            int buffer_size = 1024 * 1024; /* 1MB */
            Console.WriteLine("ipaddr=" + ipaddr + " portno=" + portno.ToString());

            VirtualBluetoothSerial serial = new VirtualBluetoothSerial();
            serial.SetServerInfo(ipaddr, portno, buffer_size);
            serial.Open();
            string data = null;
            Console.WriteLine("START BT TEST>>>>");
            for (int i = 1; i <= 10; i++)
            {
                data = "Hello World[ " + i.ToString() + " / 10 ]";
                Console.WriteLine("SEND DATA:  " + data);
                serial.WriteLine(data);
                string rcv_data = serial.ReadLine().Trim();
                Console.WriteLine("RECV DATA:  " + rcv_data);

                if (data.Equals(rcv_data))
                {
                    Console.WriteLine("TEST PASSED [ " + i.ToString() + " ]");
                }
                else
                {
                    Console.WriteLine("TEST FAILED [ " + i.ToString() + " ]");
                }
            }
            Console.WriteLine("<<<<END BT TEST");

            serial.Close();
        }
    }

}
