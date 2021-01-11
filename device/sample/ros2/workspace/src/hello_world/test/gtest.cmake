cmake_minimum_required(VERSION 2.8)

# Google Test settings
include(ExternalProject)

ExternalProject_Add(
    googletest
    URL https://github.com/google/googletest/archive/release-1.8.1.zip
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/lib
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
    )

ExternalProject_Get_Property(googletest source_dir)
set(GTEST_INCLUDE_PATH ${source_dir}/googletest/include)
set(GMOCK_INCLUDE_PATH ${source_dir}/googlemock/include)

ExternalProject_Get_Property(googletest binary_dir)

set(GTEST_LIBRARY_PATH ${binary_dir}/googlemock/gtest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest.a) # in Unix, libgtest.a
set(GTEST_LIBRARY GTest::GTest)


add_library(${GTEST_LIBRARY} UNKNOWN IMPORTED)
set_target_properties(${GTEST_LIBRARY} PROPERTIES
  IMPORTED_LOCATION ${GTEST_LIBRARY_PATH}
  INTERFACE_LINK_LIBRARIES Threads::Threads)
add_dependencies(${GTEST_LIBRARY} googletest)

set(GMOCK_LIBRARY_PATH ${binary_dir}/googlemock/${CMAKE_FIND_LIBRARY_PREFIXES}gmock.a) # in Unix, libgmock.a
set(GMOCK_LIBRARY GTest::GMock)
add_library(${GMOCK_LIBRARY} UNKNOWN IMPORTED)
set_target_properties(${GMOCK_LIBRARY} PROPERTIES
  IMPORTED_LOCATION ${GMOCK_LIBRARY_PATH}
  INTERFACE_LINK_LIBRARIES Threads::Threads)
add_dependencies(${GMOCK_LIBRARY} googletest)