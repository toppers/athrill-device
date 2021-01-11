cmake_minimum_required(VERSION 2.8)

add_executable(add-test test.cpp)


target_include_directories(add-test PUBLIC
  ${GTEST_INCLUDE_PATH}
  ${GMOCK_INCLUDE_PATH}
  )

target_link_libraries(add-test
  GTest::GTest
  GTest::GMock
  )

add_dependencies(add-test
  googletest
  )

add_test(
    NAME add
    COMMAND $<TARGET_FILE:add-test>
    )

# run with: ctest -L lib
set_property(
    TEST add
    PROPERTY LABELS lib add
    )