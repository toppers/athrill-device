#ifndef _ATHRILL_TYPES_H_
#define _ATHRILL_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef signed char std_sint8;
typedef signed short std_sint16;
typedef signed int std_sint32;
typedef signed long long std_sint64;

typedef unsigned char std_uint8;
typedef unsigned short std_uint16;
typedef unsigned int std_uint32;
typedef unsigned long long std_uint64;
typedef int std_bool;
typedef float std_float32;
typedef double std_float64;

typedef std_uint32 std_rettype;

/*
 * must be equal std_errno.h of athrill
 */
#define	STD_E_OK		0U
#define STD_E_DECODE	1U
#define STD_E_EXEC		2U
#define STD_E_SEGV		3U
#define STD_E_INVALID	4U
#define STD_E_NOENT		5U
#define STD_E_LIMIT		6U
#define STD_E_PERM		7U

#ifdef __cplusplus
}
#endif


#endif /* _ATHRILL_TYPES_H_ */