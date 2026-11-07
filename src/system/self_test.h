#pragma once

#include <cstdint>
#include <cstddef>
#include "../types.h"
#include "calibration.h"
#include "../gnss/gps_manager.h"
#include "../protocol/serial_cli.h"
#include "../ui/input_manager.h"
#include "../ui/display_ui.h"

struct SelfTestContext {
    DisplayUi &displayUi;
    GpsManager &gpsManager;
    Calibration &calibration;
    SerialCli &serialCli;
    InputManager &inputManager;

    char *targetGrid;
    char *myGrid;
    GeoPoint &currentPosition;
    GeoPoint &targetPosition;
    double &distance;
    double &bearing;
    double &revbearing;
    bool &hasHeading;
    BearingQuality &currentBearingQuality;
    unsigned long &lastHeadingUpdate;
    bool &isRoverAlertActive;
    bool &isEditing;
    int &cursorColumn;
    UiMode &currentUiMode;
    int &hours;
    int &minutes;
    bool &hasTime;

    void (*redrawMainScreen)();
    void (*updateBearings)();
    void (*drawCurrentBearing)();
};

class SelfTest {
public:
    static void run(SelfTestContext &ctx);
};

