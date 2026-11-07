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

    char *remote;
    char *mygrid;
    double &mylatitude;
    double &mylongitude;
    double &remlatitude;
    double &remlongitude;
    double &distance;
    double &bearing;
    double &revbearing;
    bool &hasHeading;
    BearingQuality &currentBearingQuality;
    unsigned long &lastHeadingUpdate;
    bool &rovdist;
    bool &isEditing;
    int &remotec;
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

