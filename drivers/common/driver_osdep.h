#ifndef _DRIVER_OSDEP_H_
#define _DRIVER_OSDEP_H_

#include "driver_types.h"
#include "v850_ins.h"

static inline void driver_os_lock(void)
{
	disable_int_all();
	return;
}
static inline void driver_os_unlock(void)
{
	enable_int_all();
	return;
}

static inline void driver_clear_intno(DrvUint32Type intno)
{
	//TODO
	return;
}
static inline void driver_os_sleep(DrvUint32Type msec)
{
	//TODO
	return;
}
#endif /* _DRIVER_OSDEP_H_ */
