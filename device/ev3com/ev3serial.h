#ifndef _EV3SERIAL_H_
#define _EV3SERIAL_H_

#include "athrill_exdev.h"
extern AthrillExDevOperationType *athrill_ex_devop;

extern void ev3serial_init(void);

#define EV3_SERIAL_BUF_MAX_SIZE 2048U
typedef struct {
    sint32                       channel_id;
	AthrillSerialFifoType       *serial_fifop;

    sint32                       rxbuflen;
    char                         rxbuffer[EV3_SERIAL_BUF_MAX_SIZE];

    sint32                       txbuflen;
    char                         txbuffer[EV3_SERIAL_BUF_MAX_SIZE];
} Ev3SerialControlType;

extern Ev3SerialControlType ev3_serial_control;
#endif /* _EV3SERIAL_H_ */