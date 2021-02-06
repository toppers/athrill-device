using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;

namespace BtSerial
{
    public class MemoryFifo
    {
        public int count;
        private int roff;
        private int woff;
        private byte[] memory;
        private object lockobj = new object();


        public MemoryFifo(int size)
        {
            this.memory = new byte[size];
            this.count = 0;
            this.roff = 0;
            this.woff = 0;
        }

        public bool IsEmpty()
        {
            return  (count == 0);
        }
        public bool IsFull()
        {
            return (count == memory.Length);
        }
        public bool HasWritableArea(int write_size)
        {
            return ((this.memory.Length - this.count) > write_size);
        }
        public byte Peek()
        {
            return this.memory[this.roff];
        }
        public byte Read()
        {
            byte[] data = new byte[1];
            int ret = this.Read(data);
            if (ret > 0) {
                return data[0];
            }
            else
            {
                throw new InvalidOperationException();
            }
        }
        public int Write(byte[] data)
        {
            int ret = 0;
            lock (this.lockobj)
            {
                for (int i = 0; i < data.Length; i++)
                {
                    if (this.IsFull())
                    {
                        break;
                    }
                    this.memory[this.woff] = data[i];
                    ret++;
                    this.woff++;
                    this.count++;
                    if (this.woff >= this.memory.Length)
                    {
                        this.woff = 0;
                    }
                }
            }
            //Console.WriteLine("Write:count=" + this.count);
            //Console.WriteLine("Write:woff=" + this.woff);
            return ret;
        }
        public int Write(string data)
        {
            byte[] buf = System.Text.Encoding.ASCII.GetBytes(data);
            return this.Write(buf);
        }

        public int Read(byte[] data)
        {
            int ret = 0;
            lock (this.lockobj)
            {
                for (int i = 0; i < data.Length; i++)
                {
                    if (this.IsEmpty())
                    {
                        break;
                    }
                    data[i] = this.memory[this.roff]; 
                    ret++;
                    this.roff++;
                    this.count--;
                    if (this.roff >= this.memory.Length)
                    {
                        this.roff = 0;
                    }
                }
            }
            return ret;
        }
        public string ReadLine()
        {
            int i = 0;
            bool need_loop = true;
            byte[] data = new byte[this.memory.Length];
            //Console.WriteLine("ReadLine: enter" + this.ToString());
            while (need_loop)
            {
                bool need_sleep = false;
                lock (this.lockobj)
                {
                    if (this.IsEmpty())
                    {
                        //Console.WriteLine("ReadLine: IsEmpty=true beause count=" + this.count);
                        need_sleep = true;
                    }
                    else
                    {
                        byte c = this.Read();
                        if ((c == '\0') || (c == '\n') || (c == '\r'))
                        //if ((c == '\0'))
                        {
                            need_loop = false;
                        }
                        //Console.WriteLine("ReadLine: char=" + c.ToString());
                        data[i++] = c;
                    }
                }
                if (need_sleep)
                {
                    //Console.WriteLine("ReadLine: sleep");
                    Thread.Sleep(100);
                }
            }
            byte[] reply = new byte[i];
            for (int j = 0; j < i; j++)
            {
                reply[j] = data[j];
            }
            string text = System.Text.Encoding.ASCII.GetString(reply);

            return text;
        }
    }
}
