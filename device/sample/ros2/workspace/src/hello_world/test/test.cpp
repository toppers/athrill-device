/*
 * main.cpp
 * */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

template <typename T> constexpr T add(const T& a, const T& b) noexcept {
    return a + b;
}

TEST(Add, integer) {
    constexpr int expected = 3;
    constexpr int actual = add(1, 2);
    ASSERT_EQ(expected, actual);
}

int main(int argc, char **argv)
{
	::testing::InitGoogleMock(&argc, argv);
	return RUN_ALL_TESTS();
}