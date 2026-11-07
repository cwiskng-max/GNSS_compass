#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include "../config.h"
#include "../types.h"
#include "../geo/geo_math.h"
#include "../geo/maidenhead.h"
#include "../ui/grid_editor.h"
#include "board.h"
#include "../ui/display_ui.h"
#include "../ui/input_manager.h"
#include "../gnss/gps_manager.h"
#include "calibration.h"
#include "../gnss/simulator.h"
#include "../protocol/serial_cli.h"

/**
 * @brief Top-level application coordinator managing active station/target state,
 *        event dispatching, geodesy updates, and screen presentation.
 */
class AppController {
public:
    AppController(Board &board,
                  DisplayUi &displayUi,
                  InputManager &inputManager,
                  GpsManager &gpsManager,
                  Calibration &calibration,
                  GpsSimulator &gpsSimulator,
                  SerialCli &serialCli);

    void begin();
    void update(unsigned long nowMs = 0);
    void handleUserEvent(const UserEvent &event, unsigned long nowMs = 0);

    // Presentation updates
    void updateBearings();
    void drawCurrentBearing();
    void redrawMainScreen();
    void runVisualSelfTest();

    // State accessors
    GridEditor& getGridEditor() { return gridEditor_; }
    const GridEditor& getGridEditor() const { return gridEditor_; }
    const char* getTargetGrid() const { return gridEditor_.getGrid(); }
    void setTargetGrid(const char *grid);

    const char* getMyGrid() const { return myGrid_; }
    void setMyGrid(const char *grid);

    const GeoPoint& getCurrentPosition() const { return currentPosition_; }
    void setCurrentPosition(const GeoPoint &pos);

    const GeoPoint& getTargetPosition() const { return targetPosition_; }
    double getDistance() const { return distance_; }
    double getBearing() const { return bearing_; }
    double getReverseBearing() const { return revbearing_; }

    bool hasHeading() const { return hasHeading_; }
    BearingQuality getBearingQuality() const { return currentBearingQuality_; }
    unsigned long getLastHeadingUpdate() const { return lastHeadingUpdate_; }

    bool isRoverAlertActive() const { return isRoverAlertActive_; }
    void setRoverAlertActive(bool active) { isRoverAlertActive_ = active; }

    UiMode getUiMode() const { return currentUiMode_; }
    void setUiMode(UiMode mode);

    bool hasTime() const { return hasTime_; }
    int getHours() const { return hours_; }
    int getMinutes() const { return minutes_; }

    // Direct subsystem references
    Board& getBoard() { return board_; }
    DisplayUi& getDisplayUi() { return displayUi_; }
    InputManager& getInputManager() { return inputManager_; }
    GpsManager& getGpsManager() { return gpsManager_; }
    Calibration& getCalibration() { return calibration_; }
    GpsSimulator& getGpsSimulator() { return gpsSimulator_; }
    SerialCli& getSerialCli() { return serialCli_; }

private:
    Board &board_;
    DisplayUi &displayUi_;
    InputManager &inputManager_;
    GpsManager &gpsManager_;
    Calibration &calibration_;
    GpsSimulator &gpsSimulator_;
    SerialCli &serialCli_;

    GridEditor gridEditor_;

    char myGrid_[9];
    bool hasLocation_;
    bool hasTime_;
    int hours_;
    int minutes_;

    bool hasHeading_;
    BearingQuality currentBearingQuality_;
    unsigned long lastHeadingUpdate_;
    bool isRoverAlertActive_;

    GeoPoint currentPosition_;
    GeoPoint targetPosition_;
    double distance_;
    double bearing_;
    double revbearing_;

    UiMode currentUiMode_;
    unsigned long calModalOpenedMillis_;
    ModalButtonId pendingModalButtonRelease_;
    unsigned long pendingModalButtonReleaseMillis_;
};

