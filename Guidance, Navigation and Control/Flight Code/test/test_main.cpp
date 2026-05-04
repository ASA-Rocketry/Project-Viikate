#include <Arduino.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

void test_example() {
    TEST_ASSERT_EQUAL(1, 1);
}

void setup() {
    Serial.begin(115200);
    delay(2000); // allow serial to connect

    UNITY_BEGIN();
    RUN_TEST(test_example);
    UNITY_END();
}

void loop() {}