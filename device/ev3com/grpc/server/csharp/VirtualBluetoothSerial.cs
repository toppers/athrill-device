using System;
using System.Collections.Generic;
using System.Text;
using System.IO.Ports;
using System.IO;

namespace BtSerial
{
    class VirtualBluetoothSerial
    {
        private string server_ipaddr = "127.0.0.1";
        private int server_portno = 50051;
        private int memory_size = (1024 * 1024);
        private MemoryFifo in_fifo;
        private MemoryFifo out_fifo;

        public void SetServerInfo(string ipaddr, int portno, int msize)
        {
            this.server_ipaddr = ipaddr;
            this.server_portno = portno;
            this.memory_size = msize;
        }
        public void Open()
        {
            this.in_fifo = new MemoryFifo(this.memory_size);
            this.out_fifo = new MemoryFifo(this.memory_size);
            SerialServer.StartServer(server_ipaddr, server_portno, this.in_fifo, this.out_fifo);
        }

        public void Close()
        {
            this.Dispose();
        }
        public void Dispose()
        {
            SerialServer.ShutdownServer();
        }
        public string ReadLine()
        {
            return this.in_fifo.ReadLine();
        }
        public void WriteLine(string text)
        {
            if (this.out_fifo.HasWritableArea(text.Length + 1))
            {
                string newline_text = text + "\n";
                this.out_fifo.Write(newline_text);
                //Console.WriteLine("WriteLine: " + text);
            }
            else
            {
                Console.WriteLine("ERROR: Can not write on data because of no space: data= " + text);
            }
        }
        public void WriteLine(char text)
        {
            if (this.out_fifo.HasWritableArea(1 + 1))
            {
                string newline_text = text + "\n";
                this.out_fifo.Write(newline_text);
                //Console.WriteLine("WriteLine: " + text);
            }
            else
            {
                Console.WriteLine("ERROR: Can not write on data because of no space: data= " + text);
            }
        }
    }
}
