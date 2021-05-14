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
        static void DoHelloWorld(VirtualBluetoothSerial serial)
        {
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
            return;
        }

        static void DoKeyInput(VirtualBluetoothSerial serial)
        {
            Console.WriteLine("Bluetooth Serial Input>");
            while (true)
            {
                var c = Console.ReadKey();
                if ((c.KeyChar != '\n') && (c.KeyChar != '\r') && (c.KeyChar != '\0'))
                {
                    serial.WriteLine((char)c.KeyChar);
                    Thread.Sleep(100);
                }
            }
        }
        static void Main(string[] args)
        {
            string ipaddr = "172.28.160.1";
            int portno = 50061;
            int buffer_size = 1024 * 1024; /* 1MB */
            Console.WriteLine("ipaddr=" + ipaddr + " portno=" + portno.ToString());

            VirtualBluetoothSerial serial = new VirtualBluetoothSerial();
            serial.SetServerInfo(ipaddr, portno, buffer_size);
            serial.Open();

            DoKeyInput(serial);

            while (true)
            {
                Thread.Sleep(1000);
            }

            serial.Close();
        }
    }

}
