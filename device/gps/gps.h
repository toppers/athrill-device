#ifndef _GPS_H_
#define _GPS_H_

#include <math.h>

#define GPS_CONST_R  6378137L
#define GPS_CONST_K0  0.9996L
#define GPS_CONST_E  0.00669438L
#define GPS_CONST_E2   ( GPS_CONST_E * GPS_CONST_E )
#define GPS_CONST_E3   ( GPS_CONST_E2 * GPS_CONST_E )
#define GPS_CONST_E_P2 ( GPS_CONST_E / (1.0L - GPS_CONST_E) )

#define GPS_CONST_SQRT_E  	( sqrt(1 - GPS_CONST_E) )
#define GPS_CONST__E  		( (1 - GPS_CONST_SQRT_E) / (1L + GPS_CONST_SQRT_E) )
#define GPS_CONST__E2  		( GPS_CONST__E * GPS_CONST__E )
#define GPS_CONST__E3  		( GPS_CONST__E2 * GPS_CONST__E )
#define GPS_CONST__E4  		( GPS_CONST__E3 * GPS_CONST__E )
#define GPS_CONST__E5  		( GPS_CONST__E4 * GPS_CONST__E )

#define GPS_CONST_M1  ( 1L - GPS_CONST_E / 4L - 3L * GPS_CONST_E2 / 64L - 5L * GPS_CONST_E3 / 256L )
#define GPS_CONST_M2  ( 3L * GPS_CONST_E / 8L + 3L * GPS_CONST_E2 / 32L + 45L * GPS_CONST_E3 / 1024L )
#define GPS_CONST_M3  ( 15L * GPS_CONST_E2 / 256L + 45L * GPS_CONST_E3 / 1024L )
#define GPS_CONST_M4  ( 35L * GPS_CONST_E3 / 3072L )

#define GPS_CONST_P2  ( 3.0L / 2L * GPS_CONST__E - 27.0L / 32 * GPS_CONST__E3 + 269.0L / 512L * GPS_CONST__E5 )
#define GPS_CONST_P3  ( 21.0L / 16L * GPS_CONST__E2 - 55.0L / 32 * GPS_CONST__E4 )
#define GPS_CONST_P4  ( 151.0L / 96L * GPS_CONST__E3 - 417.0L / 128 * GPS_CONST__E5 )
#define GPS_CONST_P5  ( 1097.0L / 512L * GPS_CONST__E4 )

#endif /* _GPS_H_ */
