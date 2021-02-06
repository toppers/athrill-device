#ifndef _SERIAL_CLIENT_H_
#define _SERIAL_CLIENT_H_


#ifdef __cplusplus
extern "C" {
#endif

extern void serial_client_init(const char* server);
typedef int ChannelType;
typedef enum {
    Ercd_OK = 0,
    Ercd_NG
} ErcdType;
extern ErcdType serial_client_put_data(ChannelType channel, const char* indata, int len);
extern ErcdType serial_client_get_data(ChannelType channel, char* outdata, int buflen, int *retlen);

#ifdef __cplusplus
}
#endif

#endif /* _SERIAL_CLIENT_H_ */
