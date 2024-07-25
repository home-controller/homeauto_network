#include "CppUTest/TestHarness.h"

/**
 * @brief use this style of include if the unit is a .c file
 *
 *
 */
// extern "C" {
//     #include "average.h"
// }

#include "../../src/hn.h"

// Create a test group
TEST_GROUP(home_network_test_group){void setup(){
    // Initialize before each test
    // circular_bufC buf;
}

                                    void teardown(){
                                        // Deinitialize after each test
                                    }};

// Test the average function
TEST(home_network_test_group, setup_test) {
    // float array[] = {-1.0, 0.0, 1.0, 2.0, 3.0};
    // float avg = average(array, 5);
    circular_bufC buf;
    CHECK_EQUAL(buf.maxLen(), 16);
    CHECK_EQUAL(buf.getLength(), 0);
}

// Test null array
TEST(home_network_test_group, pull_empty_test) {
    circular_bufC buf;
    byte x = buf.pull();
    byte t = buf.getLastError();
    CHECK_EQUAL(x, 0);
    CHECK_EQUAL(t, ReadEmptyBuf);
}

TEST(home_network_test_group, push_test) {
    circular_bufC buf;
    buf.push(1);
    CHECK_EQUAL(1, buf.getLength());
    CHECK_EQUAL(1, buf.peek());
    CHECK_EQUAL(1, buf.peek(0));
    byte x;
    for (x = 2; x <= 16; x++) {
        buf.push(x);
    }
    CHECK_EQUAL(16, buf.getBufArrayElement(15));
    buf.setLength(15);
    buf.push(17);
    CHECK_EQUAL(17, buf.getBufArrayElement(15));
    buf.push(18);
    x = buf.getLastError();
    CHECK_EQUAL(OutOfSpace, x);
    CHECK_EQUAL(17, buf.getBufArrayElement(15));
    CHECK_EQUAL(0, buf.getHeadP());
    CHECK_EQUAL(16, buf.getLength());
    for (x = 1; x <= 15; x++) {
        CHECK_EQUAL(x, buf.getBufArrayElement(x - 1));
    }
}

TEST(home_network_test_group, pull_test) {
    circular_bufC buf;
    buf.push(1);
    byte x, z;
    for (x = 2; x <= 16; x++) {
        buf.push(x);
    }
    buf.setLength(15);
    buf.push(17);
    buf.push(18);
    for (x = 1; x <= 15; x++) {
        z = buf.pull();
        CHECK_EQUAL(x, z);
    }
    z = buf.pull();
    CHECK_EQUAL(17, z);
    z = buf.pull();
    CHECK_EQUAL(0, z);
    x = buf.getLastError();
    CHECK_EQUAL(x, ReadEmptyBuf);
}