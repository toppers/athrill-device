#ifndef _HAKONIWA_EV3_H_
#define _HAKONIWA_EV3_H_

#include "std_types.h"

#define EV3COM_BASE			0x090F0000

#define EV3COM_RX_DATA_BASE	EV3COM_BASE
#define EV3COM_RX_DATA_SIZE	0x1000

#define EV3COM_RX_DATA_COMM_SIZE	1024U
#define EV3COM_RX_DATA_HEAD_OFF	0U
#define EV3COM_RX_DATA_HEAD_SIZE	32U
#define EV3COM_RX_DATA_BODY_OFF	EV3COM_RX_DATA_HEAD_SIZE
#define EV3COM_RX_DATA_HEAD_HEADER	"ETRX"
#define EV3COM_RX_DATA_HEAD_VERSION	1U
#define EV3COM_RX_DATA_HEAD_EXT_OFF	512U
#define EV3COM_RX_DATA_HEAD_EXT_SIZE	512U
typedef struct {
	uint8	header[4U];				/* 0 */
	uint32	version;				/* 4 */
	uint8	reserve[8U];			/* 8 */
	uint64	unity_simtime;			/* 16 */
	uint32	ext_off;				/* 24 */
	uint32	ext_size;				/* 28 */
} Ev3ComRxDataHeadType;


#define EV3COM_TX_DATA_BASE	(EV3COM_BASE + EV3COM_RX_DATA_SIZE)
#define EV3COM_TX_DATA_SIZE	0x1000

#define EV3COM_TX_DATA_COMM_SIZE	1024U
#define EV3COM_TX_DATA_HEAD_OFF	0U
#define EV3COM_TX_DATA_HEAD_SIZE	32U
#define EV3COM_TX_DATA_BODY_OFF		EV3COM_TX_DATA_HEAD_SIZE
#define EV3COM_TX_DATA_BODY_SIZE	(EV3COM_TX_DATA_COMM_SIZE - EV3COM_TX_DATA_HEAD_SIZE)
#define EV3COM_TX_DATA_HEAD_HEADER	"ETTX"
#define EV3COM_TX_DATA_HEAD_VERSION	1U
#define EV3COM_TX_DATA_HEAD_EXT_OFF	512U
#define EV3COM_TX_DATA_HEAD_EXT_SIZE	512U
typedef struct {
	uint8	header[4U];				/* 0 */
	uint32	version;				/* 4 */
	uint64	micon_simtime;			/* 8 */
	uint64	unity_simtime;			/* 16 */
	uint32	ext_off;				/* 24 */
	uint32	ext_size;				/* 28 */
} Ev3ComTxDataHeadType;




#define EV3COM_TX_FLAG_BASE	(EV3COM_TX_DATA_BASE + EV3COM_TX_DATA_SIZE)
#define EV3COM_TX_FLAG_SIZE	0x1000

#define EV3COM_TX_SIM_TIME_BASE	0x008
#define EV3COM_TX_SIM_TIME_SIZE	0x010

#define EV3COM_RX_SIM_TIME_BASE	0x010
#define EV3COM_RX_SIM_TIME_SIZE	0x008

#define EV3COM_TX_SIM_TIME(inx)		( EV3COM_TX_SIM_TIME_BASE + ( (inx) * 8U ) )
#define EV3COM_RX_SIM_TIME(inx)		( EV3COM_RX_SIM_TIME_BASE + ( (inx) * 8U ) )

#define EV3COM_SIM_INX_NUM		2U
#define EV3COM_SIM_INX_ME			0U
#define EV3COM_SIM_INX_YOU		1U

/*
 * RX EV3COM DATA ADDR
 */
#define EV3COM_RX_DATA(index)	(EV3COM_RX_DATA_BASE + ( ( 4 * (index) + 0 ) ))

/*
 * TX EV3COM DATA ADDR
 */
#define EV3COM_TX_DATA(index)	(EV3COM_TX_DATA_BASE + ( ( 4 * (index) + 0 ) ))

/*
 * TX EV3COM FLAG ADDR
 */
#define EV3COM_TX_FLAG(index)	(EV3COM_TX_FLAG_BASE + ( ( 1 * (index) + 0 ) ))


#endif /* _HAKONIWA_EV3_H_ */
