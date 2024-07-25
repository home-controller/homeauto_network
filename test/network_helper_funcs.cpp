// this is a fail
#include <hn.h>
#include <unity.h>

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}
SlowHomeNet hNet(2);  // these tests should not need the board so the pin should not be used.
void getLenCode_test(void) {
  TEST_ASSERT_EQUAL(hNet.getLenCode(0, 0), 0);  // Or maybe error.
  TEST_ASSERT_EQUAL(hNet.getLenCode(1, 0), 0);
  TEST_ASSERT_EQUAL(hNet.getLenCode(1, 2), 2);
  TEST_ASSERT_EQUAL(hNet.getLenCode(1, 3), 3);  // this is code for 4 bytes of data, or should maybe be error.
  TEST_ASSERT_EQUAL(hNet.getLenCode(1, 4), 3);
  TEST_ASSERT_EQUAL(hNet.getLenCode(1, 8), 3);               // currently maxes out at 4
  TEST_ASSERT_EQUAL(hNet.getLenCode(2, 7), (3 + (1 << 3)));  // max out at 4
}

void getDataLen_test(void) {
  TEST_ASSERT_EQUAL(hNet.getDataLen(0), 0);  // Or maybe error.
  TEST_ASSERT_EQUAL(hNet.getDataLen(1), 1);
  TEST_ASSERT_EQUAL(hNet.getDataLen(2), 2);
  TEST_ASSERT_EQUAL(hNet.getDataLen(3), 4);  // this is code for 4 bytes of data, or should maybe be error.
  TEST_ASSERT_EQUAL(4, hNet.getDataLen(4));
  TEST_ASSERT_EQUAL(hNet.getDataLen((3 + (1 << 3))), 4);  // currently maxes out at 4
  TEST_ASSERT_EQUAL(hNet.getDataLen((3 + (4 << 3))), 4);  // max out at 4
}
void getMessageLen_test(void) {
  TEST_ASSERT_EQUAL(1, hNet.getMessageLen(0));
  TEST_ASSERT_EQUAL(1, hNet.getMessageLen(7));
  TEST_ASSERT_EQUAL(1, hNet.getMessageLen(2));
  TEST_ASSERT_EQUAL(2, hNet.getMessageLen(7 + (1 << 3)));
  TEST_ASSERT_EQUAL(4, hNet.getMessageLen(0 + (2 << 4)));
}

void getMessageDataLen_test(void) {
  TEST_ASSERT_EQUAL(1, hNet.getMessageDataLen(0));
  TEST_ASSERT_EQUAL(4, hNet.getMessageDataLen(7));
  TEST_ASSERT_EQUAL(2, hNet.getMessageDataLen(0 + (1 << 3)));
  TEST_ASSERT_EQUAL(6, hNet.getMessageDataLen(7 + (2 << 3)));
  TEST_ASSERT_EQUAL(8, hNet.getMessageDataLen(7 + (1 << 4)));
  TEST_ASSERT_EQUAL(4, hNet.getMessageDataLen(0 + (4 << 3)));
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(getLenCode_test);
  RUN_TEST(getDataLen_test);
  RUN_TEST(getMessageLen_test);
  UNITY_END();
}
