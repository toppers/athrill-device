#include "athrill_exdev.h"
#include "gps.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>

static Std_ReturnType gps_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data);
static Std_ReturnType gps_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data);
static Std_ReturnType gps_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data);
static Std_ReturnType gps_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data);
static Std_ReturnType gps_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data);
static Std_ReturnType gps_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data);
static Std_ReturnType gps_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data);

typedef struct {
	double		easting;
	double		northing;
} AthrillGpsInputType;
typedef struct {
	double		latitude;
	double		longitude;
} AthrillGpsOutputType;
static AthrillGpsInputType *athrill_gps_input;
static AthrillGpsOutputType *athrill_gps_output;
static uint32 athrill_gpsdev_addr;
static AthrillExDevOperationType *athrill_ex_devop;
static uint32 serial_channel;
static AthrillSerialFifoType *serial;
static void athrill_gps_convert(AthrillGpsInputType *in, AthrillGpsOutputType *out);

typedef struct {
	int			zone_id;
	double		easting;
	double		northing;
} AthrillGpsBasePosType;
static AthrillGpsBasePosType athrill_gps_base_pos;
static MpuAddressRegionType *my_region;

#define EX_DEVICE_MEMORY_SIZE 1024U
static char ex_device_memory_data[EX_DEVICE_MEMORY_SIZE];

static MpuAddressRegionOperationType	ex_device_memory_operation = {
		.get_data8 		= 	gps_get_data8,
		.get_data16		=	gps_get_data16,
		.get_data32		=	gps_get_data32,

		.put_data8 		= 	gps_put_data8,
		.put_data16		=	gps_put_data16,
		.put_data32		=	gps_put_data32,

		.get_pointer	= gps_get_pointer,
};
static void ex_device_init(MpuAddressRegionType *region, AthrillExDevOperationType *athrill_ops)
{
	Std_ReturnType err;
	athrill_ex_devop = athrill_ops;
	my_region = region;
	err = athrill_ex_devop->param.get_devcfg_value_hex("DEBUG_FUNC_GPS_DEVADDR", &athrill_gpsdev_addr);
	if (err != STD_E_OK) {
		printf("ERROR: can not find paramter DEBUG_FUNC_GPS_DEVADDR\n");
		return;
	}
	err = athrill_ex_devop->param.get_devcfg_value("DEVICE_CONFIG_GPS_ZONE_ID", (uint32 *)&athrill_gps_base_pos.zone_id);
	if (err != STD_E_OK) {
		printf("ERROR: can not find paramter DEVICE_CONFIG_GPS_ZONE_ID\n");
		return;
	}
	printf("DEVICE_CONFIG_GPS_ZONE_ID=%d\n", athrill_gps_base_pos.zone_id);
	char *strp;
	err = athrill_ex_devop->param.get_devcfg_string("DEVICE_CONFIG_GPS_BASE_EASTING", &strp);
	if (err != STD_E_OK) {
		printf("ERROR: can not find paramter DEVICE_CONFIG_GPS_BASE_EASTING\n");
		return;
	}
	athrill_gps_base_pos.easting = strtod(strp, NULL);

	err = athrill_ex_devop->param.get_devcfg_string("DEVICE_CONFIG_GPS_BASE_NORTHING", &strp);
	if (err != STD_E_OK) {
		printf("ERROR: can not find paramter DEVICE_CONFIG_GPS_BASE_NORTHING\n");
		return;
	}
	athrill_gps_base_pos.northing = strtod(strp, NULL);
	err = athrill_ex_devop->param.get_devcfg_value("DEBUG_FUNC_GPS_SERIAL_CH", &serial_channel);
	if (err != STD_E_OK) {
		printf("ERROR: can not find paramter DEBUG_FUNC_GPS_SERIAL_CH\n");
		return;
	}
	printf("DEVICE_CONFIG_GPS_BASE_EASTING=%lf\n", athrill_gps_base_pos.easting);
	printf("DEVICE_CONFIG_GPS_BASE_NORTHING=%lf\n", athrill_gps_base_pos.northing);

	err = athrill_ex_devop->dev.get_memory(athrill_gpsdev_addr, (uint8**)&athrill_gps_input);
	if (err != STD_E_OK) {
		printf("ERROR: can not find memory addr=0x%x\n", athrill_gpsdev_addr);
		return;
	}
	err = athrill_ex_devop->dev.get_memory(my_region->start, (uint8**)&athrill_gps_output);
	if (err != STD_E_OK) {
		printf("ERROR: can not find memory addr=0x%x\n", athrill_gpsdev_addr);
		return;
	}
	printf("###INFO gps device input addr=0x%x output addr=0x%x.\n", athrill_gpsdev_addr, my_region->start);
	return;
}
static char output_buffer[1024];

static void ex_device_supply_clock(DeviceClockType *dev_clock)
{
	if ((dev_clock->clock % 10000) == 0) {
		time_t t = time(NULL);
		static time_t prev_t = 0;
		if ((t - prev_t) >= 1) {
			//printf("easting=%lf northing=%lf\n", athrill_gps_input->easting, athrill_gps_input->northing);
			athrill_gps_convert(athrill_gps_input, athrill_gps_output);
			//(CommFifoBufferType *fifop, const char* datap, uint32 datalen, uint32 *res);
			if (serial == NULL) {
				athrill_ex_devop->dev.get_serial_fifo(serial_channel, &serial);
			}
			if (serial != NULL) {
				//printf("latitude=%lf longtitude=%lf\n", athrill_gps_output->latitude, athrill_gps_output->longtitude);
				uint32 len = snprintf(output_buffer, sizeof(output_buffer), "$GPRMC:%ld,A,%lf,N,%lf,W\n", t, athrill_gps_output->latitude, athrill_gps_output->longitude);
				uint32 res;
				if ((serial->rd.max_size - serial->rd.count) >= len) {
					(void)athrill_ex_devop->libs.fifo.add(&serial->rd, output_buffer, len, &res);
				}
			}
			prev_t = t;
		}
	}
	return;
}

/**************************************
 * START: external symbols
 **************************************/
AthrillExDeviceType athrill_ex_device = {
		.header.magicno = ATHRILL_EXTERNAL_DEVICE_MAGICNO,
		.header.version = ATHRILL_EXTERNAL_DEVICE_VERSION,
		.header.memory_size = EX_DEVICE_MEMORY_SIZE, /* Bytes */
		.datap = ex_device_memory_data,
		.ops = &ex_device_memory_operation,
		.devinit = ex_device_init,
		.supply_clock = ex_device_supply_clock,
};
/**************************************
 * END: external symbols
 **************************************/
static void athrill_gps_convert(AthrillGpsInputType *in, AthrillGpsOutputType *out)
{
	double x = in->easting  + athrill_gps_base_pos.easting- 500000L;
	double y = in->northing + athrill_gps_base_pos.northing;
	double m = y / GPS_CONST_K0;
	double mu = m / (GPS_CONST_R * GPS_CONST_M1);

	double p_rad = mu +
			GPS_CONST_P2 * sin(2 * mu) +
			GPS_CONST_P3 * sin(4 * mu) +
			GPS_CONST_P4 * sin(6 * mu) +
			GPS_CONST_P5 * sin(8 * mu);

	double p_sin = sin(p_rad);
	double p_sin2 = p_sin * p_sin;

	double p_cos = cos(p_rad);

	double p_tan = p_sin / p_cos;
	double p_tan2 = p_tan * p_tan;
	double p_tan4 = p_tan2 * p_tan2;

	double ep_sin = 1 - GPS_CONST_E * p_sin2;
	double ep_sin_sqrt = sqrt(1 - GPS_CONST_E * p_sin2);

	double n = GPS_CONST_R / ep_sin_sqrt;
	double r = (1 - GPS_CONST_E) / ep_sin;

	double c = GPS_CONST__E * p_cos * p_cos;
	double c2 = c * c;

	double d = x / (n * GPS_CONST_K0);
	double d2 = d * d;
	double d3 = d2 * d;
	double d4 = d3 * d;
	double d5 = d4 * d;
	double d6 = d5 * d;

	double lat = p_rad - (p_tan / r) *
	                (d2 / 2 -
	                d4 / 24 * (5 + 3 * p_tan2 + 10 * c - 4 * c2 - 9 * GPS_CONST_E_P2)) +
	                d6 / 720 * (61 + 90 * p_tan2 + 298 * c + 45 * p_tan4 - 252 * GPS_CONST_E_P2 - 3 * c2);

	double lon = (d -
	                d3 / 6 * (1 + 2 * p_tan2 + c) +
	                d5 / 120 * (5 - 2 * c + 28 * p_tan2 - 3 * c2 + 8 * GPS_CONST_E_P2 + 24 * p_tan4)) / p_cos;
	out->latitude = lat * 180.0 / M_PI;
	out->longitude = lon * 180.0 / M_PI;
    if (athrill_gps_base_pos.zone_id > 0) {
    	out->longitude += (athrill_gps_base_pos.zone_id - 1) * 6 - 180 + 3;
    }
	return;
}

static Std_ReturnType gps_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint8*)(&region->data[off]));
	return STD_E_OK;
}

static Std_ReturnType gps_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint16*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType gps_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint32*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType gps_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data)
{
	uint32 off = (addr - region->start);
	*((uint8*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType gps_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data)
{
	uint32 off = (addr - region->start);
	*((uint16*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType gps_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data)
{
	uint32 off = (addr - region->start);
	*((uint32*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType gps_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data)
{
	uint32 off = (addr - region->start);
	*data = &region->data[off];
	return STD_E_OK;
}
