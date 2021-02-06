#include "ev3serial.h"
#include "assert.h"
#include "grpc/client/cpp/serial_client.h"
#include <stdio.h>

static Std_ReturnType rx_thread_do_init(MpthrIdType id);
static Std_ReturnType tx_thread_do_init(MpthrIdType id);
static Std_ReturnType rx_thread_do_proc(MpthrIdType id);
static Std_ReturnType tx_thread_do_proc(MpthrIdType id);


Ev3SerialControlType ev3_serial_control;
static MpthrOperationType	rx_thread_ops = {
	.do_init = rx_thread_do_init,
	.do_proc = rx_thread_do_proc,
};
static MpthrOperationType	tx_thread_ops = {
	.do_init = tx_thread_do_init,
	.do_proc = tx_thread_do_proc,
};

void ev3serial_init(void)
{
	char *server = "localhost";
	char *portno = "50051";
	static char ip_port[1024];
    ev3_serial_control.channel_id = -1;
	Std_ReturnType err = athrill_ex_devop->param.get_devcfg_value("DEVICE_CONFIG_EV3_CH1_SERIAL_ID", (uint32*)&ev3_serial_control.channel_id);
	if (err != STD_E_OK) {
		return;
	}
	(void)athrill_ex_devop->param.get_devcfg_string("DEVICE_CONFIG_EV3_BT_SERVER_IPADDR", &server);
	printf("DEVICE_CONFIG_EV3_BT_SERVER_IPADDR=%s\n", server);
	(void)athrill_ex_devop->param.get_devcfg_string("DEVICE_CONFIG_EV3_BT_SERVER_PORTNO", &portno);
	printf("DEVICE_CONFIG_EV3_BT_SERVER_PORTNO=%s\n", portno);
	sprintf(ip_port, "%s:%s", server, portno);
	serial_client_init(ip_port);
    /*
     * Get Serial fifo
     */
	athrill_ex_devop->dev.get_serial_fifo(ev3_serial_control.channel_id, &ev3_serial_control.serial_fifop);
	ASSERT (ev3_serial_control.serial_fifop != NULL);

    /*
     * Thread start
     */
	err = athrill_ex_devop->libs.thread.thr_register(&ev3_serial_control.serial_fifop->rx_thread, &rx_thread_ops);
	ASSERT(err == STD_E_OK);
	err = athrill_ex_devop->libs.thread.thr_register(&ev3_serial_control.serial_fifop->tx_thread, &tx_thread_ops);
	ASSERT(err == STD_E_OK);

	err = athrill_ex_devop->libs.thread.start_proc(ev3_serial_control.serial_fifop->rx_thread);
	ASSERT(err == STD_E_OK);
	err = athrill_ex_devop->libs.thread.start_proc(ev3_serial_control.serial_fifop->tx_thread);
	ASSERT(err == STD_E_OK);

}

static Std_ReturnType rx_thread_do_init(MpthrIdType id)
{
    //nothing to do
    return STD_E_OK;
}
static Std_ReturnType tx_thread_do_init(MpthrIdType id)
{
    //nothing to do
    return STD_E_OK;
}
static int get_serial_data(uint8 *p)
{
	uint32 res;
	athrill_ex_devop->libs.thread.lock(ev3_serial_control.serial_fifop->tx_thread);
	if (ev3_serial_control.serial_fifop->wr.count == 0) {
		athrill_ex_devop->libs.thread.unlock(ev3_serial_control.serial_fifop->tx_thread);
		return -1;
	}
	Std_ReturnType err = athrill_ex_devop->libs.fifo.get(&ev3_serial_control.serial_fifop->wr, (char*)p, 1, &res);
	ASSERT(err == STD_E_OK);
	athrill_ex_devop->libs.thread.unlock(ev3_serial_control.serial_fifop->tx_thread);
	return 0;
}

static void tx_thread_do_put(sint32 id, const char *bufp, int buflen)
{
	while (TRUE) {
        ErcdType ercd = serial_client_put_data(id, bufp);
		if (ercd == Ercd_OK) {
			break;
		}
	    target_os_api_sleep(1000);
	}
	return;
}

static Std_ReturnType tx_thread_do_proc(MpthrIdType id)
{
    int ret;
	uint8 data;
    printf ("ev3com_tx_thread:READY\n");
	while (TRUE) {
        /*
         * get send buffer
         */
		ret = get_serial_data(&data);
		if (ret < 0) {
		    target_os_api_sleep(1000);
			continue;
		}
		ev3_serial_control.txbuffer[ev3_serial_control.txbuflen] = data;
		ev3_serial_control.txbuflen++;
		if ( (ev3_serial_control.txbuflen >= (EV3_SERIAL_BUF_MAX_SIZE - 1)) ||
		     ((data == '\0') || (data == '\r') || (data == '\n')) ) {
    		ev3_serial_control.txbuffer[ev3_serial_control.txbuflen] = '\0';
			tx_thread_do_put(ev3_serial_control.channel_id, ev3_serial_control.txbuffer, ev3_serial_control.txbuflen);
            ev3_serial_control.txbuflen = 0;
        }
	}
	return STD_E_OK;
}

static void rx_put_buffer(const char* datap, int len)
{
	Std_ReturnType err;
    uint32 res;
    athrill_ex_devop->libs.thread.lock(ev3_serial_control.serial_fifop->rx_thread);
    while (TRUE) {
    	int freespace = EV3_SERIAL_BUF_MAX_SIZE - ev3_serial_control.serial_fifop->rd.count;
    	if (freespace < len) {
    		athrill_ex_devop->libs.thread.unlock(ev3_serial_control.serial_fifop->rx_thread);
            target_os_api_sleep(100);
            athrill_ex_devop->libs.thread.lock(ev3_serial_control.serial_fifop->rx_thread);
    		continue;
    	}
		//printf("RECV:%s\n", datap);
        err = athrill_ex_devop->libs.fifo.add(&ev3_serial_control.serial_fifop->rd, (const char*)datap, len, &res);
        ASSERT(err == STD_E_OK);
        break;
    }
    athrill_ex_devop->libs.thread.unlock(ev3_serial_control.serial_fifop->rx_thread);
	return;
}

static Std_ReturnType rx_thread_do_proc(MpthrIdType id)
{
    printf ("ev3com_rx_thread:READY\n");

    while (TRUE) {
        ErcdType ercd = serial_client_get_data(ev3_serial_control.channel_id, 
                            ev3_serial_control.rxbuffer, 
                            EV3_SERIAL_BUF_MAX_SIZE,
                            &ev3_serial_control.rxbuflen);
        if (ercd != Ercd_OK) {
	        target_os_api_sleep(1000);
            continue;
        }
        rx_put_buffer(ev3_serial_control.rxbuffer, (ev3_serial_control.rxbuflen + 1));
        target_os_api_sleep(1000);
    }
    return STD_E_OK;
}
