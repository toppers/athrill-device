#!/bin/bash

if [ $# -ne 1 ]
then
	echo "$0 {all|clean}"
	exit 1
fi

if [ $1 = "all" ]
then
	cmake ../..
	make
else
	rm -rf ./CMakeFiles
	rm -f Makefile
	rm -f sample_client*
	rm -f sample.*
	rm -f CMake*
	rm -f cmake*
fi
