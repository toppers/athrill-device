cmake_minimum_required(VERSION 3.5.1)

link_directories(
#    ./ros2_ws/install/athrill_device/lib
)

add_compile_options(-fPIC)
#set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")

if (ENABLE_GCOV)
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} --coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
endif()

add_subdirectory(./ros2_ws/athrill_device)


project(ros2dev C CXX)

add_compile_options(-g)
add_compile_options(-Wall)
add_compile_options(-Wunknown-pragmas)
add_compile_options(-Wtrigraphs)

set(CMAKE_C_FLAGS "-std=gnu99")
set(CMAKE_CXX_FLAGS "-std=c++11")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++ -lc++abi")

set(CORE_DIR ../../../athrill/src)

include_directories("${CORE_DIR}/inc")
include_directories("${CORE_DIR}/cpu")
include_directories("${CORE_DIR}/bus")
include_directories("${CORE_DIR}/device/mpu")
include_directories("${CORE_DIR}/device/peripheral/serial/fifo")
include_directories("${CORE_DIR}/lib")
include_directories(../lib/include)
include_directories(../lib/src)
include_directories(./include)
include_directories(./gen)
include_directories(./ros2_ws/athrill_device/include)
include_directories(./ros2_ws/athrill_device/include/gen)

add_compile_definitions(OS_LINUX)
add_compile_definitions(ATHRILL_EXT_DEVICE)


add_library(rosdev SHARED 
    rosdev.c 
    gen/rosdev_gen.c
    gen/rosdev_gen_enc.c
    gen/rosdev_gen_dec.c
)
target_compile_options(rosdev PRIVATE -fPIC)
target_link_libraries(rosdev athrill_device)
