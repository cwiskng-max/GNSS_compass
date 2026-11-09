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
#include "rover_log_manager.h"
#include "../ui/rover_card.h"
#include "../ui/rover_history_modal.h"

/**
 * @brief High-level operational mode of the compass system.
 */
enum class OperationalMode : uint8_t {
    Bearing = 0,
    Rover = 1,
    MODE_BEARING = Bearing,
    MODE_ROVER = Rover
};
constexpr OperationalMode MODE_BEARING = OperationalMode::Bearing;
constexpr OperationalMode MODE_ROVER = OperationalMode::Rover;

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
                  SerialCli &serialCli,
                  RoverLogManager *roverLogManager = nullptr);

    void begin();
    void update(unsigned long nowMs = 0);
    void handleUserEvent(const UserEvent &event, unsigned long nowMs = 0);

    // Navigation & presentation updates
    void recalculateGeodesy();
    void renderNavigationRows();
    void updateNavigation();
    void drawCurrentBearing();
    void redrawMainScreen();
    void runVisualSelfTest();

    // Rover mode transitions & gestures
    void startRoverMode(unsigned long nowMs = 0);
    void stopRoverMode(unsigned long nowMs = 0);
    void onTargetEdited(unsigned long nowMs = 0);
    void discardLastStopAndRearm(unsigned long nowMs = 0);

    // State accessors
    OperationalMode getOperationalMode() const { return operationalMode_; }
    void setOperationalMode(OperationalMode mode);
    bool isRoverMode() const { return operationalMode_ == OperationalMode::Rover; }

    const GeoPoint& getOriginPoint() const { return originPoint_; }
    const char* getOriginGrid() const { return originGrid_; }
    RoverState getCurrentRoverState() const { return lastRoverState_; }
    double getLastValidDisplacementKm() const { return lastValidDisplacementKm_; }

    RoverLogManager& getRoverLogManager() { return *roverLogManagerRef_; }
    const RoverLogManager& getRoverLogManager() const { return *roverLogManagerRef_; }
    GridEditor& getGridEditor() { return gridEditor_; }
    const GridEditor& getGridEditor() const { return gridEditor_; }
    const char* getTargetGrid() const { return gridEditor_.getGrid(); }
    void setTargetGrid(const char *grid);

    const char* getMyGrid() const { return myGrid_; }
    void setMyGrid(const char *grid);

    const GeoPoint& getCurrentPosition() const { return currentPosition_; }
    void setCurrentPosition(const GeoPoint &pos);

    const GeoPoint& getTargetPosition() const { return targetPosition_; }
    void setTargetPosition(const GeoPoint &pos) { targetPosition_ = pos; }
    double getDistance() const { return distance_; }
    double getBearing() const { return bearing_; }
    double getReverseBearing() const { return revbearing_; }

    bool hasHeading() const { return hasHeading_; }
    BearingQuality getBearingQuality() const { return currentBearingQuality_; }
    unsigned long getLastHeadingUpdate() const { return lastHeadingUpdate_; }
    void setHeadingState(bool hasHeading, BearingQuality quality, unsigned long lastUpdate = 0);

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

    RoverLogManager roverLogManagerOwned_;
    RoverLogManager *roverLogManagerRef_;

    GridEditor gridEditor_;

    char myGrid_[9];
    bool hasLocation_;
    bool hasTime_;
    int hours_;
    int minutes_;

    bool hasHeading_;
    BearingQuality currentBearingQuality_;
    unsigned long lastHeadingUpdate_;

    OperationalMode operationalMode_;
    OperationalMode lastRenderedMode_;
    GeoPoint originPoint_;
    char originGrid_[9];
    double lastValidDisplacementKm_;
    unsigned long lastLocationUpdate_;
    RoverState lastRoverState_;
    unsigned long currentNowMs_;

    GeoPoint currentPosition_;
    GeoPoint targetPosition_;
    double distance_;
    double bearing_;
    double revbearing_;

    UiMode currentUiMode_;
    unsigned long calModalOpenedMillis_;
    ModalButtonId pendingModalButtonRelease_;
    unsigned long pendingModalButtonReleaseStartMillis_;
};

