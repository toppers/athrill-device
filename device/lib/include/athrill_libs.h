#ifndef _ATHRILL_LIBS_H_
#define _ATHRILL_LIBS_H_

#include "athrill_types.h"

/*
 * param
 */
extern std_rettype athrill_get_devcfg_value(const char* key, std_uint32 *value);
extern std_rettype athrill_get_devcfg_value_hex(const char* key, std_uint32 *value);
extern std_rettype athrill_get_devcfg_string(const char* key, char **value);

/*
 * intr
 */
extern std_rettype athrill_add_intr(const char* name, std_uint32 intno, std_uint32 priority); //TODO target dependent function
extern void athrill_raise_intr(std_uint32 intno);

#endif /* _ATHRILL_LIBS_H_ */
