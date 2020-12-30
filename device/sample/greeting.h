#ifndef _GREETING_H_
#define _GREETING_H_

#ifdef __cplusplus
extern "C" {
#endif

extern void greeting_init(void);
extern void greeting(const char* strp, unsigned long long clock);

#ifdef __cplusplus
}
#endif

#endif