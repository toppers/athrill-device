#ifndef _TEST_CHECK_H_
#define _TEST_CHECK_H_

#include "test_serial.h"

#define DO_TEST(test_func)	\
do {	\
	if (test_func() == 0) {	\
		test_print("PASSED : " #test_func);	\
	}	\
	else {	\
		test_print("FAILED : " #test_func);	\
	}	\
} while (0)

#endif /* _TEST_CHECK_H_ */
