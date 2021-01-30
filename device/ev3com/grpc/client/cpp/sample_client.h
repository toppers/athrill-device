#ifndef _SAMPLE_CLIENT_H_
#define _SAMPLE_CLIENT_H_


#ifdef __cplusplus
extern "C" {
#endif

extern void sample_client_init(void);
extern void sample_client_request(const char* strp, unsigned long long clock);

#ifdef __cplusplus
}
#endif

#endif
