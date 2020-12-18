include ../Makefile.env

OBJS	:= ev3com.o
OBJS	+= ev3com_mmap.o
OBJS	+= ev3com_udp.o

TARGET	:= ev3com.so

IFLAGS	:= -I$(CORE_DIR)/inc
IFLAGS	+= -I$(CORE_DIR)/cpu
IFLAGS	+= -I$(CORE_DIR)/bus
IFLAGS	+= -I$(CORE_DIR)/device/mpu
IFLAGS	+= -I$(CORE_DIR)/device/peripheral/serial/fifo
IFLAGS	+= -I$(CORE_DIR)/lib

CFLAGS	:= $(WFLAGS)
CFLAGS	+= $(IFLAGS)

VPATH	:= .

$(TARGET): $(OBJS)
	$(GCC) $(DLFLAGS)  -o $(TARGET) $(OBJS)


clean:
	rm -f $(OBJS) $(TARGET)
