//
// Created by Rob Wills on 8/8/16.
//

#include "test_packet.h"
#include "Packet.h"

#ifdef UNIT_TEST


// void tearDown(void) {
// // clean stuff up here
// }


void setup() {
    UNITY_BEGIN();    // IMPORTANT LINE!
//    RUN_TEST(test_led_builtin_pin_number);

    pinMode(LED_BUILTIN, OUTPUT);
}

void test_packet_build_sensor(void) {
    TEST_ASSERT_EQUAL(MTS::Type::SENSOR, 1);
}

void loop() {
    RUN_TEST(test_packet_build_sensor);
    UNITY_END(); // stop unit testing
}

#endif // UNIT_TEST