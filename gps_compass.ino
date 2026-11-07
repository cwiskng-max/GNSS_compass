#include "src/config.h"
#include "src/system/board.h"
#include "src/ui/display_ui.h"
#include "src/ui/input_manager.h"
#include "src/gnss/gps_manager.h"
#include "src/system/calibration.h"
#include "src/gnss/simulator.h"
#include "src/protocol/serial_cli.h"
#include "src/system/app_controller.h"

// Hardware & Subsystem Instances
Board         board;
DisplayUi     displayUi;
InputManager  inputManager;
GpsManager    gpsManager;
Calibration   calibration;
GpsSimulator  gpsSimulator;
SerialCli     serialCli(gpsManager, calibration, gpsSimulator);

// Top-Level Application Controller
AppController app(board, displayUi, inputManager, gpsManager, calibration, gpsSimulator, serialCli);

void setup() {
  Serial.begin(115200);
  Serial.print("GNSS Compass ");
  Serial.println(FW_VERSION);
  Serial.println(F("Commands: 'status' (GPS stream diagnosis), 'test' (visual demo), 'sim' (simulation)"));

  app.begin();
}

void loop() {
  app.update();

  // Cooperative FreeRTOS yield for background tasks & watchdog
  delay(1);
}
