#include "athrill_exdev.h"
#include "ev3com_private.h"
#include "assert.h"
#include <stdio.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

static Std_ReturnType ev3com_udp_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data);
static Std_ReturnType ev3com_udp_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data);
static Std_ReturnType ev3com_udp_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data);
static Std_ReturnType ev3com_udp_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data);
static Std_ReturnType ev3com_udp_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data);
static Std_ReturnType ev3com_udp_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data);
static Std_ReturnType ev3com_udp_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data);

MpuAddressRegionOperationType	ev3com_udp_memory_operation = {
		.get_data8 		= 	ev3com_udp_get_data8,
		.get_data16		=	ev3com_udp_get_data16,
		.get_data32		=	ev3com_udp_get_data32,

		.put_data8 		= 	ev3com_udp_put_data8,
		.put_data16		=	ev3com_udp_put_data16,
		.put_data32		=	ev3com_udp_put_data32,

		.get_pointer	= ev3com_udp_get_pointer,
};


static MpthrIdType ev3com_thrid;
static MpthrIdType ev3com_send_thrid;

static Std_ReturnType ev3com_thread_do_init(MpthrIdType id);
static Std_ReturnType ev3com_thread_do_proc(MpthrIdType id);
static Std_ReturnType ev3com_send_thread_do_init(MpthrIdType id);
static Std_ReturnType ev3com_send_thread_do_proc(MpthrIdType id);
static uint32 enable_complemental_send = 0; // defaule false
static uint32 reset_area_off = 0;
static uint32 reset_area_size  = 0; 
static char* ev3com_simtime_sync_filepath = NULL;
static uint32 ev3com_simtime_sync_linenum = 10240;

#define COLOR_NUM	8
static char *color_name[COLOR_NUM] = {
	"NONE",
	"BLACK",
	"BLUE",
	"GREEN",
	"YELLOW",
	"RED",
	"WHITE",
	"BROWN",
};

typedef struct {
	uint64 unity_simtime; /* usec */
	uint64 athrill_simtime; /* usec */
	float64 pos_x;
	float64 pos_y;
	uint32 color;
	uint32 reflect;
	uint32 ultrasonic;
	uint32 motor_angle_a;
	uint32 motor_angle_b;
	uint32 padding;
} Ev3ComSimTimeSynDataType;
typedef struct {
	int fd;
	Ev3ComSimTimeSynDataType *bufferp;
	int current_line;
	int max_line;
} Ev3ComSimTimeSyncFileType;
static Ev3ComSimTimeSyncFileType ev3com_simtime_sync_file;
static void ev3com_simtime_sync_init(void)
{
	ev3com_simtime_sync_file.bufferp = malloc(ev3com_simtime_sync_linenum * sizeof(Ev3ComSimTimeSynDataType));
	ASSERT(ev3com_simtime_sync_file.bufferp != NULL);
	ev3com_simtime_sync_file.current_line = 0;
	ev3com_simtime_sync_file.max_line = ev3com_simtime_sync_linenum;
	ev3com_simtime_sync_file.fd = open(ev3com_simtime_sync_filepath, O_CREAT|O_TRUNC|O_RDWR, 0644);
	ASSERT(ev3com_simtime_sync_file.fd >= 0);
	return;
}

static void ev3com_simtime_sync_write(uint64 unity_time, uint64 athrill_time)
{
	if (ev3com_simtime_sync_file.current_line >= ev3com_simtime_sync_file.max_line) {
		return;
	}
	ev3com_simtime_sync_file.bufferp[ev3com_simtime_sync_file.current_line].unity_simtime = unity_time;
	ev3com_simtime_sync_file.bufferp[ev3com_simtime_sync_file.current_line].athrill_simtime = athrill_time;
	//double* data_double = (double*)&ev3com_control.comm.read_data.buffer[EV3COM_RX_DATA_BODY_OFF + 480];
	//ev3com_simtime_sync_file.bufferp[ev3com_simtime_sync_file.current_line].pos_x = *data_double;
	//data_double = (double*)&ev3com_control.comm.read_data.buffer[EV3COM_RX_DATA_BODY_OFF + 488];
	//ev3com_simtime_sync_file.bufferp[ev3com_simtime_sync_file.current_line].pos_y = *data_double;
	memcpy((void*)&ev3com_simtime_sync_file.bufferp[ev3com_simtime_sync_file.current_line].pos_x, 
		&ev3com_control.comm.read_data.buffer[EV3COM_RX_DATA_BODY_OFF + 480], 8U);
	memcpy((void*)&ev3com_simtime_sync_file.bufferp[ev3com_simtime_sync_file.current_line].pos_y, 
		&ev3com_control.comm.read_data.buffer[EV3COM_RX_DATA_BODY_OFF + 488], 8U);


	uint32 *data = (uint32*)&ev3com_control.comm.read_data.buffer[EV3COM_RX_DATA_BODY_OFF + 8];
	ev3com_simtime_sync_file.bufferp[ev3com_simtime_sync_file.current_line].color = *data;

	data = (uint32*)&ev3com_control.comm.read_data.buffer[EV3COM_RX_DATA_BODY_OFF + 12];
	ev3com_simtime_sync_file.bufferp[ev3com_simtime_sync_file.current_line].reflect = *data;

	data = (uint32*)&ev3com_control.comm.read_data.buffer[EV3COM_RX_DATA_BODY_OFF + 88];
	ev3com_simtime_sync_file.bufferp[ev3com_simtime_sync_file.current_line].ultrasonic = *data;

	data = (uint32*)&ev3com_control.comm.read_data.buffer[EV3COM_RX_DATA_BODY_OFF + 256];
	ev3com_simtime_sync_file.bufferp[ev3com_simtime_sync_file.current_line].motor_angle_a = *data;

	data = (uint32*)&ev3com_control.comm.read_data.buffer[EV3COM_RX_DATA_BODY_OFF + 260];
	ev3com_simtime_sync_file.bufferp[ev3com_simtime_sync_file.current_line].motor_angle_b = *data;
	ev3com_simtime_sync_file.current_line++;
	return;
}
static void ev3com_simtime_sync_flush(void)
{
	int i;
	dprintf(ev3com_simtime_sync_file.fd, "unity, athrill,color,pos_x,pos_y,reflect,ultrasonic,motor_angle_a,motor_angle_b\n");
	for (i = 0; i < ev3com_simtime_sync_file.current_line; i++) {
		dprintf(ev3com_simtime_sync_file.fd, "%lf, %lf,%s,%lf,%lf,%u,%u,%u,%u\n", 
				((double)ev3com_simtime_sync_file.bufferp[i].unity_simtime)/((double)1000000),
				((double)ev3com_simtime_sync_file.bufferp[i].athrill_simtime)/((double)1000000),
				color_name[ev3com_simtime_sync_file.bufferp[i].color],
				ev3com_simtime_sync_file.bufferp[i].pos_x,
				ev3com_simtime_sync_file.bufferp[i].pos_y,
				ev3com_simtime_sync_file.bufferp[i].reflect,
				ev3com_simtime_sync_file.bufferp[i].ultrasonic,
				ev3com_simtime_sync_file.bufferp[i].motor_angle_a,
				ev3com_simtime_sync_file.bufferp[i].motor_angle_b
				);
	}
	close(ev3com_simtime_sync_file.fd);
	ev3com_simtime_sync_file.fd = -1;
	return;
}
void ev3com_udp_cleanup(void)
{
	if (ev3com_simtime_sync_filepath != NULL) {
		ev3com_simtime_sync_flush();
	}
	return;
}

static MpthrOperationType ev3com_op = {
	.do_init = ev3com_thread_do_init,
	.do_proc = ev3com_thread_do_proc,
};

// Complemental TX Communication
static MpthrOperationType ev3com_send_op = {
	.do_init = ev3com_send_thread_do_init,
	.do_proc = ev3com_send_thread_do_proc,
};
struct timespec previous_sent = {0};

static void save_sent_time(void)
{
	clock_gettime(CLOCK_MONOTONIC,&previous_sent);
}

static long get_time_from_previous_sending(void)
{
	struct timespec cur;
	clock_gettime(CLOCK_MONOTONIC,&cur);
	return (cur.tv_sec-previous_sent.tv_sec)*1000000000 + cur.tv_nsec-previous_sent.tv_nsec;
}


static void lock_send_mutex(void)
{
	if ( enable_complemental_send ) {
		athrill_ex_devop->libs.thread.lock(ev3com_send_thrid);
	}
}
static void unlock_send_mutex(void)
{
	if ( enable_complemental_send ) {
		athrill_ex_devop->libs.thread.unlock(ev3com_send_thrid);
	}
}


void device_init_ev3com_udp(MpuAddressRegionType *region)
{
	Std_ReturnType err;
	uint32 portno;

	(void)athrill_ex_devop->param.get_devcfg_string("DEBUG_FUNC_EV3COM_TIMESYNC_FPATH", &ev3com_simtime_sync_filepath);
	(void)athrill_ex_devop->param.get_devcfg_value("DEBUG_FUNC_EV3COM_TIMESYNC_LINENUM", (uint32*)&ev3com_simtime_sync_linenum);
	if (ev3com_simtime_sync_filepath != NULL) {
		ev3com_simtime_sync_init();
	}

	ev3com_control.remote_ipaddr = "127.0.0.1";
	(void)athrill_ex_devop->param.get_devcfg_string("DEBUG_FUNC_EV3COM_TX_IPADDR", &ev3com_control.remote_ipaddr);
	printf("VDEV:TX IPADDR=%s\n", ev3com_control.remote_ipaddr);
	err = athrill_ex_devop->param.get_devcfg_value("DEBUG_FUNC_EV3COM_TX_PORTNO", &portno);
	if (err != STD_E_OK) {
		printf("ERROR: can not load param DEBUG_FUNC_EV3COM_TX_PORTNO\n");
		ASSERT(err == STD_E_OK);
	}
	printf("VDEV:TX PORTNO=%d\n", portno);
	ev3com_control.local_ipaddr = "127.0.0.1";
	(void)athrill_ex_devop->param.get_devcfg_string("DEBUG_FUNC_EV3COM_RX_IPADDR", &ev3com_control.local_ipaddr);
	printf("VDEV:RX IPADDR=%s\n", ev3com_control.local_ipaddr);
	ev3com_control.config.client_port = (uint16)portno;
	err = athrill_ex_devop->param.get_devcfg_value("DEBUG_FUNC_EV3COM_RX_PORTNO", &portno);
	if (err != STD_E_OK) {
		printf("ERROR: can not load param DEBUG_FUNC_EV3COM_RX_PORTNO\n");
		ASSERT(err == STD_E_OK);
	}
	ev3com_control.config.server_port = (uint16)portno;
	printf("VDEV:RX PORTNO=%d\n", portno);

	err = athrill_ex_devop->libs.udp.create_ipaddr(&ev3com_control.config, &ev3com_control.comm, ev3com_control.local_ipaddr);
	ASSERT(err == STD_E_OK);
	//initialize udp write buffer header
	{
		Ev3ComTxDataHeadType *tx_headp = (Ev3ComTxDataHeadType*)&ev3com_control.comm.write_data.buffer[0];
		memset((void*)tx_headp, 0, EV3COM_TX_DATA_HEAD_SIZE);
		memcpy((void*)tx_headp->header, EV3COM_TX_DATA_HEAD_HEADER, strlen(EV3COM_TX_DATA_HEAD_HEADER));
		tx_headp->version = EV3COM_TX_DATA_HEAD_VERSION;
		tx_headp->ext_off = EV3COM_TX_DATA_HEAD_EXT_OFF;
		tx_headp->ext_size = EV3COM_TX_DATA_HEAD_EXT_SIZE;
		ev3com_control.comm.write_data.len = EV3COM_TX_DATA_COMM_SIZE;
	}

	err = athrill_ex_devop->libs.thread.thr_register(&ev3com_thrid, &ev3com_op);
	ASSERT(err == STD_E_OK);

	err = athrill_ex_devop->libs.thread.start_proc(ev3com_thrid);
	ASSERT(err == STD_E_OK);

	// Complemental TX sending thread
	err = athrill_ex_devop->param.get_devcfg_value("DEVICE_CONFIG_COMPLEMENTAL_TX_SENDING", &enable_complemental_send);
	printf("DEVICE_CONFIG_COMPLEMENTAL_TX_SENDING=%d\n",enable_complemental_send);

	if ( enable_complemental_send ) {
		err = athrill_ex_devop->libs.thread.thr_register(&ev3com_send_thrid, &ev3com_send_op);
		ASSERT(err == STD_E_OK);

		err = athrill_ex_devop->libs.thread.start_proc(ev3com_send_thrid);
		ASSERT(err == STD_E_OK);
	}

	// Reset Area
	err = athrill_ex_devop->param.get_devcfg_value("DEVICE_CONFIG_RESET_AREA_OFFSET", &reset_area_off);
	printf("DEVICE_CONFIG_RESET_AREA_OFFSET=%d\n",reset_area_off);

	err = athrill_ex_devop->param.get_devcfg_value("DEVICE_CONFIG_RESET_AREA_SIZE", &reset_area_size);
	printf("DEVICE_CONFIG_RESET_AREA_SIZE=%d\n",reset_area_size);

	if ( enable_complemental_send ) {
		// If complemental_send is enable, reset information is required
		ASSERT( reset_area_off && reset_area_size);
	}

	return;
}
void device_supply_clock_ev3com_udp(DeviceClockType *dev_clock)
{
	uint64 interval;
	uint64 unity_sim_time;

	unity_sim_time = ev3com_control.vdev_sim_time[EV3COM_SIM_INX_YOU] * ((uint64)ev3com_control.cpu_freq);

	ev3com_control.vdev_sim_time[EV3COM_SIM_INX_ME] = ( dev_clock->clock / ((uint64)ev3com_control.cpu_freq) );
	if (unity_sim_time == 0) {
		dev_clock->min_intr_interval = 1U;
	}
	else if (dev_clock->min_intr_interval != DEVICE_CLOCK_MAX_INTERVAL) {
		if ((unity_sim_time <= dev_clock->clock)) {
			interval = 1U;
			//printf("UNITY <= MICON:%llu %llu\n", ev3com_control.ev3com_sim_time[EV3COM_SIM_INX_YOU], ev3com_control.ev3com_sim_time[EV3COM_SIM_INX_ME]);
		}
		else {
			//interval = (unity_sim_time - dev_clock->clock) + ((unity_interval_vtime  * ((uint64)ev3com_udp_control.cpu_freq)) / 2);
			interval = (unity_sim_time - dev_clock->clock);
			//printf("UNITY > MICON:%llu %llu\n", ev3com_control.ev3com_sim_time[EV3COM_SIM_INX_YOU], ev3com_control.ev3com_sim_time[EV3COM_SIM_INX_ME]);
		}
		if (interval < dev_clock->min_intr_interval) {
			dev_clock->min_intr_interval = interval;
		}
	}
	return;
}

static Std_ReturnType ev3com_thread_do_init(MpthrIdType id)
{
	//nothing to do
	return STD_E_OK;
}

static Std_ReturnType ev3com_udp_packet_check(const char *p)
{
	const uint32 *p_int = (const uint32 *)&ev3com_control.comm.read_data.buffer[0];
	if (strncmp(p, EV3COM_RX_DATA_HEAD_HEADER, 4) != 0) {
		printf("ERROR: INVALID HEADER:%c%c%c%c\n", p[0], p[1], p[2], p[3]);
		return STD_E_INVALID;
	}
	if (p_int[1] != EV3COM_RX_DATA_HEAD_VERSION) {
		printf("ERROR: INVALID VERSION:0x%x\n", p_int[1]);
		return STD_E_INVALID;
	}
	if (p_int[6] != EV3COM_RX_DATA_HEAD_EXT_OFF) {
		printf("ERROR: INVALID EXT_OFF:0x%x\n", p_int[6]);
		return STD_E_INVALID;
	}
	if (p_int[7] != EV3COM_RX_DATA_HEAD_EXT_SIZE) {
		printf("ERROR: INVALID EXT_SIZE:0x%x\n", p_int[7]);
		return STD_E_INVALID;
	}
	return STD_E_OK;
}

static Std_ReturnType ev3com_thread_do_proc(MpthrIdType id)
{
	Std_ReturnType err;
	uint32 off = EV3COM_RX_DATA_BASE - EV3COM_BASE;
	uint64 curr_stime;

	while (1) {
		err = athrill_ex_devop->libs.udp.read(&ev3com_control.comm);
		
		if (err != STD_E_OK) {
			continue;
		} else if (ev3com_udp_packet_check((const char*)&ev3com_control.comm.read_data.buffer[0]) != STD_E_OK) {
			continue;
		}
		//gettimeofday(&unity_notify_time, NULL);
		memcpy(&ev3com_control.region->data[off], &ev3com_control.comm.read_data.buffer[0], ev3com_control.comm.read_data.len);
		memcpy((void*)&curr_stime, &ev3com_control.comm.read_data.buffer[EV3COM_RX_SIM_TIME(EV3COM_SIM_INX_ME)], 8U);

		//unity_interval_vtime = curr_stime - ev3com_udp_control.ev3com_sim_time[EV3COM_SIM_INX_YOU];
		//ev3com_calc_predicted_virtual_time(ev3com_udp_control.ev3com_sim_time[EV3COM_SIM_INX_YOU], curr_stime);
		ev3com_control.vdev_sim_time[EV3COM_SIM_INX_YOU] = curr_stime;
		if (ev3com_simtime_sync_filepath != NULL) {
			ev3com_simtime_sync_write(ev3com_control.vdev_sim_time[EV3COM_SIM_INX_YOU], ev3com_control.vdev_sim_time[EV3COM_SIM_INX_ME]);
		}
	}
	return STD_E_OK;


}


static Std_ReturnType ev3com_send_thread_do_proc(MpthrIdType id)
{
	Std_ReturnType err;

	save_sent_time();

	while (1) {
		err = 0;
		long diff = get_time_from_previous_sending();
		if ( diff < 10*1000000) { 
			// Wait 10msec from previous sending 
			long left = 10*1000000 - diff;
			struct timespec next = {0,left};
			nanosleep(&next,0);
		} 
		lock_send_mutex();
		// check if another thread has sent TX massage to Unity
		diff = get_time_from_previous_sending();
		if ( diff >= 10*1000000 )  {
				err = athrill_ex_devop->libs.udp.remote_write(&ev3com_control.comm, ev3com_control.remote_ipaddr);
				if (err != STD_E_OK) {
					printf("WARNING: vdevput_data8: udp send error=%d\n", err);
				}
				save_sent_time();
		} else {
			// It seems another thread has sent TX message. Skip sending in this case.
		}
		unlock_send_mutex();
	}
	return STD_E_OK;
}

static Std_ReturnType ev3com_send_thread_do_init(MpthrIdType id)
{
	//nothing to do
	return STD_E_OK;
}
static Std_ReturnType ev3com_udp_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data)
{
	uint32 off = (addr - region->start) + EV3COM_RX_DATA_BODY_OFF;
	*data = *((uint8*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType ev3com_udp_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data)
{
	uint32 off = (addr - region->start) + EV3COM_RX_DATA_BODY_OFF;
	*data = *((uint16*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType ev3com_udp_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data)
{
	uint32 off = (addr - region->start) + EV3COM_RX_DATA_BODY_OFF;
	*data = *((uint32*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType ev3com_udp_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data)
{
	uint32 off = (addr - region->start) + EV3COM_TX_DATA_BODY_OFF;
	*((uint8*)(&region->data[off])) = data;

	if (addr == EV3COM_TX_FLAG(0)) {

		uint32 tx_off = EV3COM_TX_DATA_BASE - region->start;
		Std_ReturnType err;
	
		lock_send_mutex();

		memcpy(&ev3com_control.comm.write_data.buffer[EV3COM_TX_DATA_BODY_OFF], &region->data[tx_off + EV3COM_TX_DATA_BODY_OFF], EV3COM_TX_DATA_BODY_SIZE);
		memcpy(&ev3com_control.comm.write_data.buffer[EV3COM_TX_SIM_TIME(EV3COM_SIM_INX_ME)],  (void*)&ev3com_control.vdev_sim_time[EV3COM_SIM_INX_ME], 8U);
		memcpy(&ev3com_control.comm.write_data.buffer[EV3COM_TX_SIM_TIME(EV3COM_SIM_INX_YOU)], (void*)&ev3com_control.vdev_sim_time[EV3COM_SIM_INX_YOU], 8U);
		//printf("sim_time=%llu\n", ev3com_udp_control.ev3com_sim_time[EV3COM_SIM_INX_ME]);
		err = athrill_ex_devop->libs.udp.remote_write(&ev3com_control.comm, ev3com_control.remote_ipaddr);

		// Clear reset area
		if ( reset_area_off && reset_area_size ) {
				memset(&ev3com_control.comm.write_data.buffer[reset_area_off],0,reset_area_size);
				memset(&region->data[tx_off+reset_area_off],0,reset_area_size);
		}
		save_sent_time();
		unlock_send_mutex();

		if (err != STD_E_OK) {
			printf("WARNING: vdevput_data8: udp send error=%d\n", err);
		}

	}
	else {
	}

	return STD_E_OK;
}
static Std_ReturnType ev3com_udp_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data)
{
	uint32 off = (addr - region->start) + EV3COM_TX_DATA_BODY_OFF;
	*((uint16*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType ev3com_udp_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data)
{
	uint32 off = (addr - region->start) + EV3COM_TX_DATA_BODY_OFF;
	*((uint32*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType ev3com_udp_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data)
{
	uint32 off = (addr - region->start) + EV3COM_TX_DATA_BODY_OFF;
	*data = &region->data[off];
	return STD_E_OK;
}


