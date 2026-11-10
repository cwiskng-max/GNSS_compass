#pragma once

// Selects the BLE backend at compile time.
//   1: Arduino/ESP32 build. Uses the Bluedroid stack (ble_serial_manager_esp32.cpp).
//   0: Host build (unit tests). Uses the simulated transport (ble_serial_manager_host.cpp).
#if defined(ARDUINO)
#define BLE_SERIAL_HW 1
#else
#define BLE_SERIAL_HW 0
#endif

