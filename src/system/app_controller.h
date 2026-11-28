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
#include "../protocol/ble_serial_manager.h"
#include "rover_log_manager.h"
#include "qso_log_manager.h"
#include "target_preset_manager.h"
#include "../ui/rover_card.h"
#include "../ui/rover_history_modal.h"
#include "../ui/qso_card.h"

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
                  RoverLogManager *roverLogManager = nullptr,
                  BleSerialManager *bleManager = nullptr,
                  QsoLogManager *qsoLogManager = nullptr,
                  TargetPresetManager *targetPresetManager = nullptr);

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

    // QSO Pop-Up Card modal controls
    void showQsoCard(const QsoEntry &qso, size_t qsoIndex, unsigned long nowMs = 0);
    void dismissQsoCard();
    unsigned long getQsoCardOpenedMs() const { return qsoCardOpenedMillis_; }

    // Target List modal state accessors
    size_t getSelectedPresetIndex() const { return displayUi_.getTargetListModal().getSelectedIndex(); }
    size_t getPresetScrollOffset() const { return displayUi_.getTargetListModal().getScrollOffset(); }
    unsigned long getTargetModalOpenedMs() const { return targetModalOpenedMillis_; }
    const char* resolveTargetLabel(const char *grid) const;

    // State accessors
    OperationalMode getOperationalMode() const { return operationalMode_; }
    void setOperationalMode(OperationalMode mode);
    bool isRoverMode() const { return operationalMode_ == OperationalMode::Rover; }
    bool isRoverWarningActive(unsigned long nowMs = 0) const {
        if (!roverWarningActive_) return false;
        unsigned long now = (nowMs == 0 ? currentNowMs_ : nowMs);
        return (now - roverWarningStartMs_) < ROVER_WARNING_DURATION_MS;
    }

    const GeoPoint& getOriginPoint() const { return originPoint_; }
    const char* getOriginGrid() const { return originGrid_; }
    RoverState getCurrentRoverState() const { return lastRoverState_; }
    double getLastValidDisplacementKm() const { return lastValidDisplacementKm_; }

    RoverLogManager& getRoverLogManager() { return *roverLogManagerRef_; }
    const RoverLogManager& getRoverLogManager() const { return *roverLogManagerRef_; }
    QsoLogManager& getQsoLogManager() { return *qsoLogManagerRef_; }
    const QsoLogManager& getQsoLogManager() const { return *qsoLogManagerRef_; }
    TargetPresetManager& getTargetPresetManager() { return *targetPresetManagerRef_; }
    const TargetPresetManager& getTargetPresetManager() const { return *targetPresetManagerRef_; }
    GridEditor& getGridEditor() { return gridEditor_; }
    const GridEditor& getGridEditor() const { return gridEditor_; }
    const char* getTargetGrid() const { return gridEditor_.getGrid(); }
    void setTargetGrid(const char *grid, unsigned long nowMs = 0);

    const char* getMyGrid() const { return myGrid_; }
    void setMyGrid(const char *grid);

    bool hasLocation() const { return hasLocation_; }
    const GeoPoint& getCurrentPosition() const { return currentPosition_; }
    void setCurrentPosition(const GeoPoint &pos);
    void clearLocation() { hasLocation_ = false; currentPosition_ = GeoPoint(); lastLocationUpdate_ = 0; }

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

    bool isSelfTesting() const { return isSelfTesting_; }
    void setSelfTesting(bool testing) { isSelfTesting_ = testing; }

    // Direct subsystem references
    Board& getBoard() { return board_; }
    DisplayUi& getDisplayUi() { return displayUi_; }
    InputManager& getInputManager() { return inputManager_; }
    GpsManager& getGpsManager() { return gpsManager_; }
    Calibration& getCalibration() { return calibration_; }
    GpsSimulator& getGpsSimulator() { return gpsSimulator_; }
    SerialCli& getSerialCli() { return serialCli_; }
    // BLE accessors used by the unit tests to check wiring and UI connection state
    BleSerialManager* getBleManager() const { return bleManager_; }
    bool isBleConnected() const { return bleConnected_; }

    // Invoked (on the main loop) by BleSerialManager's connection callback
    void handleBleConnectionChanged(bool connected);

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
    QsoLogManager qsoLogManagerOwned_;
    QsoLogManager *qsoLogManagerRef_;
    TargetPresetManager targetPresetManagerOwned_;
    TargetPresetManager *targetPresetManagerRef_;
    BleSerialManager *bleManager_;
    bool bleConnected_;

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
    bool roverWarningActive_;
    unsigned long roverWarningStartMs_;

    GeoPoint currentPosition_;
    GeoPoint targetPosition_;
    double distance_;
    double bearing_;
    double revbearing_;

    UiMode currentUiMode_;
    bool isSelfTesting_;
    unsigned long modalOpenedMillis_;
    unsigned long qsoCardOpenedMillis_;
    unsigned long targetModalOpenedMillis_;
    unsigned long quickSettingsOpenedMillis_;
    ModalButtonId pendingModalButtonRelease_;
    unsigned long pendingModalButtonReleaseStartMillis_;

    void clampTargetModalSelection();
    void drawTargetListModal();
    void drawTargetListRows();
    void loadSelectedTargetPreset(unsigned long nowMs);
    uint32_t currentTimestampSec(unsigned long nowMs) const;
};

