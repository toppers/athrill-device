#ifndef _ATHRILL_SERIAL_H_
#define _ATHRILL_SERIAL_H_

#include "driver_types.h"
#include "athrill_serial_fifo.h"

extern int athrill_serial_init(DrvInt32Type channel, DrvUint32Type baud);
extern int athrill_serial_set_baud(DrvInt32Type channel, DrvUint32Type baud);
extern int athrill_serial_send(DrvInt32Type channel, const char* at_cmd);
extern int athrill_serial_send_data(DrvInt32Type channel, const char* datap, DrvInt32Type datalen);
extern int athrill_serial_readline(DrvInt32Type channel, char* bufferp, DrvInt32Type bufflen);
extern int athrill_serial_read_data(DrvInt32Type channel, char* bufferp, DrvInt32Type bufflen);
extern int athrill_serial_skip_newline(DrvInt32Type channel);

#endif /* _ATHRILL_SERIAL_H_ */
