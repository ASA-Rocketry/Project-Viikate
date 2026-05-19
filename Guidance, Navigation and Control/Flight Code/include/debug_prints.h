#if defined(PRODUCTION_FLIGHT_MODE)
    #define MODE_NAME "PRODUCTION_FLIGHT_MODE"
#elif defined(TEST_STATE_MACHINE_MODE)
    #define MODE_NAME "TEST_STATE_MACHINE_MODE"
    #define ENABLE_TEST_PRINTS
#elif defined(TEST_PID_AND_CALIBRATION_MODE)
    #define MODE_NAME "TEST_PID_AND_CALIBRATION_MODE"
    #define ENABLE_TEST_PRINTS
#endif

#ifdef ENABLE_TEST_PRINTS
    #define DEBUG_BEGIN(...) Serial.begin(__VA_ARGS__)
    #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
    #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
    #define DEBUG_BEGIN(...)
    #define DEBUG_PRINT(...)
    #define DEBUG_PRINTLN(...)
#endif