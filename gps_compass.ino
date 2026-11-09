#include "src/config.h"
#include "src/system/board.h"
#include "src/ui/display_ui.h"
#include "src/ui/input_manager.h"
#include "src/gnss/gps_manager.h"
#include "src/system/calibration.h"
#include "src/gnss/simulator.h"
#include "src/system/rover_log_manager.h"
#include "src/protocol/ble_serial_manager.h"
#include "src/protocol/serial_cli.h"
#include "src/system/app_controller.h"

// Hardware & Subsystem Instances
Board            board;
DisplayUi        displayUi;
InputManager     inputManager;
GpsManager       gpsManager;
Calibration      calibration;
GpsSimulator     gpsSimulator;
RoverLogManager  roverLogManager;
BleSerialManager bleSerialManager;
SerialCli        serialCli(gpsManager, calibration, gpsSimulator, &roverLogManager, &bleSerialManager);

// Top-Level Application Controller
AppController app(board, displayUi, inputManager, gpsManager, calibration, gpsSimulator, serialCli, &roverLogManager, &bleSerialManager);

void setup() {
  Serial.begin(115200);
  Serial.print("GNSS Compass ");
  Serial.println(FW_VERSION);
  Serial.println(F("Commands: 'status' (GPS stream), 'rover' (transit tracking), 'cal' (antenna), 'sim' (simulation)"));
  Serial.print(F("BLE Device: "));
  Serial.println(bleSerialManager.getDeviceName());

  app.begin();
}

void loop() {
  app.update();

  // Cooperative FreeRTOS yield for background tasks & watchdog
  delay(1);
}
