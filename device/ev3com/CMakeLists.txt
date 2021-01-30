cmake_minimum_required(VERSION 3.5.1)

add_compile_options(-fPIC)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")

add_subdirectory(grpc)

project(EV3COM C CXX)

add_compile_options(-g)
add_compile_options(-Wall)
add_compile_options(-Wunknown-pragmas)
add_compile_options(-Wtrigraphs)

set(CMAKE_C_FLAGS "-std=gnu99")
set(CMAKE_CXX_FLAGS "-std=c++11")

set(CORE_DIR ../../../athrill/src)

include_directories("${CORE_DIR}/inc")
include_directories("${CORE_DIR}/cpu")
include_directories("${CORE_DIR}/bus")
include_directories("${CORE_DIR}/device/mpu")
include_directories("${CORE_DIR}/device/peripheral/serial/fifo")
include_directories("${CORE_DIR}/lib")
include_directories(../lib/include)
include_directories(../lib/src)

add_compile_definitions(OS_LINUX)
add_compile_definitions(ATHRILL_EXT_DEVICE)
add_compile_definitions(CXX_SAMPLE)

add_library(ev3com SHARED 
    ev3com.c 
    ev3com_mmap.c
    ev3com_udp.c
    ../lib/src/athrill_priv.c)
target_compile_options(ev3com PRIVATE -fPIC)
target_link_libraries(ev3com sample_client)

