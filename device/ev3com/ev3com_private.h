#ifndef _EV3COM_PRIVATE_H_
#define _EV3COM_PRIVATE_H_

#include "ev3com.h"
#include "std_errno.h"
#include "udp/udp_comm.h"

typedef enum {
	Ev3ComIoOperation_UDP = 0,
	Ev3ComIoOperation_MMAP,
} Ev3ComIoOperationType;

typedef struct {
	UdpCommConfigType config;
	MpuAddressRegionType *region;
	uint32		cpu_freq;
	uint64 		vdev_sim_time[EV3COM_SIM_INX_NUM]; /* usec */
	/*
	 * for UDP ONLY
	 */
	UdpCommType comm;
	char *remote_ipaddr;
	char *local_ipaddr;

	/*
	 * for MMAP ONLY
	 */
	uint32		tx_data_addr;
	uint8		*tx_data;
	uint32		rx_data_addr;
	uint8		*rx_data;
} Ev3ComControlType;
extern Ev3ComControlType ev3com_control;

extern AthrillExDevOperationType *athrill_ex_devop;

/*
 * UDP operations
 */
extern void device_init_ev3com_udp(MpuAddressRegionType *region);
extern void device_supply_clock_ev3com_udp(DeviceClockType *dev_clock);
extern MpuAddressRegionOperationType	ev3com_udp_memory_operation;


/*
 * MMAP operations
 */
extern void device_init_ev3com_mmap(MpuAddressRegionType *region);
extern void device_supply_clock_ev3com_mmap(DeviceClockType *dev_clock);
extern MpuAddressRegionOperationType	ev3com_mmap_memory_operation;

#endif /* _EV3COM_PRIVATE_H_ */
