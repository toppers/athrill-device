cmake_minimum_required(VERSION 3.5.1)

add_compile_options(-fPIC)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")

if (SAMPLE_ROS2)
    add_subdirectory(ros2/workspace/src/hello_world)
else()
    add_subdirectory(grpc)
endif()

project(Sample C CXX)

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

add_compile_definitions(OS_LINUX)
add_compile_definitions(ATHRILL_EXT_DEVICE)
add_compile_definitions(CXX_SAMPLE)

add_library(devsample SHARED sample_device.c greeting.cc)
target_compile_options(devsample PRIVATE -fPIC)
target_link_libraries(devsample sample_client)

