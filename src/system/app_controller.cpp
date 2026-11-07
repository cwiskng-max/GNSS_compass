#include "app_controller.h"
#include "self_test.h"
#include <cmath>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

AppController::AppController(Board &board,
                             DisplayUi &displayUi,
                             InputManager &inputManager,
                             GpsManager &gpsManager,
                             Calibration &calibration,
                             GpsSimulator &gpsSimulator,
                             SerialCli &serialCli)
    : board_(board),
      displayUi_(displayUi),
      inputManager_(inputManager),
      gpsManager_(gpsManager),
      calibration_(calibration),
      gpsSimulator_(gpsSimulator),
      serialCli_(serialCli),
      hasLocation_(false),
      hasTime_(false),
      hours_(0),
      minutes_(0),
      hasHeading_(false),
      currentBearingQuality_(BEARING_NONE),
      lastHeadingUpdate_(0),
      isRoverAlertActive_(false),
      distance_(0.0),
      bearing_(0.0),
      revbearing_(0.0),
      currentUiMode_(UiMode::Normal),
      calModalOpenedMillis_(0),
      pendingModalButtonRelease_(ModalButtonId::None),
      pendingModalButtonReleaseStartMillis_(0)
{
    std::strcpy(myGrid_, "JJ00AA00");
}

void AppController::setTargetGrid(const char *grid) {
    gridEditor_.setGrid(grid);
    updateNavigation();
}

void AppController::setMyGrid(const char *grid) {
    if (!grid) return;
    std::strncpy(myGrid_, grid, sizeof(myGrid_) - 1);
    myGrid_[sizeof(myGrid_) - 1] = '\0';
}

void AppController::setCurrentPosition(const GeoPoint &pos) {
    hasLocation_ = true;
    currentPosition_ = pos;
    geoPointToGrid(currentPosition_, myGrid_);
    if (currentUiMode_ != UiMode::Calibrating) {
        displayUi_.renderMyGrid(myGrid_);
    }
    updateNavigation();
}

void AppController::setUiMode(UiMode mode) {
    currentUiMode_ = mode;
    inputManager_.setUiMode(mode);
}

void AppController::drawCurrentBearing() {
    BearingFilter &bf = gpsManager_.getBearingFilter();
    if (currentUiMode_ == UiMode::Calibrating) {
        displayUi_.getCalModal().drawRawHeading(bf.getRawHeading(), hasHeading_);
        return;
    }
    displayUi_.renderCurrentBearing(bf.getCalibratedHeading(calibration_.getOffset()),
                                  currentBearingQuality_, hasHeading_);
}

void AppController::recalculateGeodesy() {
    if (!isRoverAlertActive_) {
        gridToGeoPoint(gridEditor_.getGrid(), targetPosition_);
    }
    calculateWgs84(currentPosition_, targetPosition_, distance_, bearing_);
    revbearing_ = calculateReverseBearing(currentPosition_, targetPosition_);
}

void AppController::renderNavigationRows() {
    if (currentUiMode_ == UiMode::Calibrating) {
        return;
    }

    displayUi_.renderReverseBearingAndDistance(revbearing_, distance_, isRoverAlertActive_);
    drawCurrentBearing();
    displayUi_.renderCalibrationPill(calibration_.getOffset(), calibration_.isCalibrated());
    displayUi_.renderTargetBearing(bearing_);
    displayUi_.renderArrlDistance(myGrid_, gridEditor_.getGrid());
}

void AppController::updateNavigation() {
    recalculateGeodesy();
    renderNavigationRows();
}

void AppController::redrawMainScreen() {
    BearingFilter &bf = gpsManager_.getBearingFilter();
    MainScreenState s;
    s.myGrid = myGrid_;
    s.targetGrid = gridEditor_.getGrid();
    s.cursorColumn = gridEditor_.getCursorColumn();
    s.isEditing = gridEditor_.isEditing();
    s.targetBearing = bearing_;
    s.revBearing = revbearing_;
    s.distanceMeters = distance_;
    s.isRoverAlertActive = isRoverAlertActive_;
    s.currentHeading = bf.getCalibratedHeading(calibration_.getOffset());
    s.bearingQuality = currentBearingQuality_;
    s.hasHeading = hasHeading_;
    s.calOffset = calibration_.getOffset();
    s.isCalibrated = calibration_.isCalibrated();
    s.hours = hours_;
    s.minutes = minutes_;
    s.hasTime = hasTime_;

    displayUi_.redrawMainScreen(s);
}

void AppController::runVisualSelfTest() {
    SelfTest::run(*this);
}

void AppController::begin() {
    board_.init();
    displayUi_.begin();

#if defined(ARDUINO)
    inputManager_.setPcf8574(&board_.getPcf8574());
#endif
    inputManager_.begin();
    calibration_.begin();

    gpsManager_.setSimulator(&gpsSimulator_);
#if defined(ARDUINO)
    gpsManager_.begin(GPS_BAUD, RXD2, TXD2);
#else
    gpsManager_.begin(GPS_BAUD);
#endif

    // Wire GPS Manager event callbacks (passing this as context)
    gpsManager_.setOnHeadingUpdated([](float /* heading */, BearingQuality quality, void *ctx) {
        auto *self = static_cast<AppController*>(ctx);
        if (!self) return;
        self->hasHeading_ = self->gpsManager_.hasHeading();
        self->currentBearingQuality_ = quality;
        self->lastHeadingUpdate_ = self->gpsManager_.getLastHeadingUpdate();
        self->drawCurrentBearing();
    }, this);

    gpsManager_.setOnLocationUpdated([](const GeoPoint &pos, void *ctx) {
        auto *self = static_cast<AppController*>(ctx);
        if (!self) return;
        self->setCurrentPosition(pos);
    }, this);

    gpsManager_.setOnTimeUpdated([](int h, int m, void *ctx) {
        auto *self = static_cast<AppController*>(ctx);
        if (!self) return;
        self->hasTime_ = true;
        self->hours_ = h;
        self->minutes_ = m;
        if (self->currentUiMode_ != UiMode::Calibrating) {
            self->displayUi_.renderUtcTime(h, m, true);
        }
    }, this);

    // Wire Serial CLI callbacks (passing this as context)
    serialCli_.begin();
    serialCli_.setNavigationContextProvider([](void *ctx) -> CliNavContext {
        CliNavContext nctx;
        auto *self = static_cast<AppController*>(ctx);
        if (self) {
            nctx.targetBearing = self->bearing_;
            nctx.isRoverAlertActive = self->isRoverAlertActive_;
            nctx.roverDistanceMeters = self->distance_;
        }
        return nctx;
    }, this);

    serialCli_.setRunTestCallback([](void *ctx) {
        auto *self = static_cast<AppController*>(ctx);
        if (self) self->runVisualSelfTest();
    }, this);

    serialCli_.setStateChangedCallback([](void *ctx) {
        auto *self = static_cast<AppController*>(ctx);
        if (!self) return;
        self->hasHeading_ = self->gpsManager_.hasHeading();
        self->currentBearingQuality_ = self->gpsManager_.getBearingQuality();
        self->lastHeadingUpdate_ = self->gpsManager_.getLastHeadingUpdate();
        if (self->gpsManager_.hasLocation()) {
            self->hasLocation_ = true;
            self->currentPosition_ = self->gpsManager_.getLocation();
            geoPointToGrid(self->currentPosition_, self->myGrid_);
            if (self->currentUiMode_ != UiMode::Calibrating) {
                self->displayUi_.renderMyGrid(self->myGrid_);
            }
        }
        if (self->gpsManager_.hasTime()) {
            self->hasTime_ = true;
            self->hours_ = self->gpsManager_.getHours();
            self->minutes_ = self->gpsManager_.getMinutes();
            if (self->currentUiMode_ != UiMode::Calibrating) {
                self->displayUi_.renderUtcTime(self->hours_, self->minutes_, true);
            }
        }
        self->updateNavigation();
    }, this);

    serialCli_.setRoverSimCallback([](const char *orig_grid, double dist_km, void *ctx) {
        auto *self = static_cast<AppController*>(ctx);
        if (!self) return;
        GeoPoint originPoint = self->currentPosition_;
        if (orig_grid && strlen(orig_grid) >= 4) {
            gridToGeoPoint(orig_grid, originPoint);
            self->gridEditor_.setGrid(orig_grid);
        } else {
            self->gridEditor_.setGrid(self->myGrid_);
        }
        self->targetPosition_ = originPoint;
        self->isRoverAlertActive_ = true;

        GeoPoint simulatedPoint = GpsSimulator::calculateRoverDisplacement(originPoint, dist_km, 45.0);
        self->gpsManager_.setLocation(simulatedPoint);
        self->displayUi_.renderTargetGrid(self->gridEditor_.getGrid(),
                                          self->gridEditor_.getCursorColumn(),
                                          self->gridEditor_.isEditing());
        self->updateNavigation();

#if defined(ARDUINO)
        int rdist = (int)std::floor(self->distance_ / 1000.0);
        Serial.printf("[SIM ROVER] Origin: %s (%.5f, %.5f) | Current: %s (%.5f, %.5f)\r\n",
                      self->gridEditor_.getGrid(), self->targetPosition_.latitude, self->targetPosition_.longitude,
                      self->myGrid_, self->currentPosition_.latitude, self->currentPosition_.longitude);
        Serial.printf("[SIM ROVER] Distance: %5.1f km (%d km) | Alert: %s\r\n",
                      self->distance_ / 1000.0, rdist, (rdist < 16) ? "ACTIVE (RED on WHITE)" : "CLEAR (>= 16 km)");
#endif
    }, this);

    geoPointToGrid(currentPosition_, myGrid_);
    displayUi_.renderMyGrid(myGrid_);

    gridEditor_.recordActivity(0);
    displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(), gridEditor_.isEditing());
    updateNavigation();

    displayUi_.renderUtcTime(0, 0, false);
    displayUi_.showFooterText(FW_VERSION, DIM_GRAY_TEXT);
}

void AppController::handleUserEvent(const UserEvent &event, unsigned long nowMs) {
#if defined(ARDUINO)
    if (nowMs == 0) nowMs = millis();
#endif

    switch (event.type) {
        case UserEventType::RotateClockwise:
            if (currentUiMode_ == UiMode::Calibrating) {
                calModalOpenedMillis_ = nowMs;
                calibration_.adjustProposedOffset(0.1f);
                displayUi_.getCalModal().drawProposedOffsetRow(calibration_.getProposedOffset());
            } else if (gridEditor_.isEditing()) {
                isRoverAlertActive_ = false;
                gridEditor_.rollChar(+1, nowMs);
                displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(), gridEditor_.isEditing());
                updateNavigation();
            }
            break;

        case UserEventType::RotateCounterClockwise:
            if (currentUiMode_ == UiMode::Calibrating) {
                calModalOpenedMillis_ = nowMs;
                calibration_.adjustProposedOffset(-0.1f);
                displayUi_.getCalModal().drawProposedOffsetRow(calibration_.getProposedOffset());
            } else if (gridEditor_.isEditing()) {
                isRoverAlertActive_ = false;
                gridEditor_.rollChar(-1, nowMs);
                displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(), gridEditor_.isEditing());
                updateNavigation();
            }
            break;

        case UserEventType::EncoderButtonPressed:
            // Toggle edit mode on knob press
            gridEditor_.setEditing(!gridEditor_.isEditing());
            gridEditor_.recordActivity(nowMs);
            displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(), gridEditor_.isEditing());
            break;

        case UserEventType::TouchCalibrationPill:
            if (hasHeading_) {
                calibration_.snapProposed(bearing_, gpsManager_.getBearingFilter().getRawHeading());
            } else {
                calibration_.syncProposedFromActive();
            }
            setUiMode(UiMode::Calibrating);
            calModalOpenedMillis_ = nowMs;
            gridEditor_.setEditing(false);
            displayUi_.getCalModal().drawModal(bearing_, gpsManager_.getBearingFilter().getRawHeading(), hasHeading_,
                                              calibration_.getOffset(), calibration_.getProposedOffset());
            break;

        case UserEventType::TouchLocalGrid:
            gridEditor_.setGrid(myGrid_);
            targetPosition_ = currentPosition_;
            isRoverAlertActive_ = true;
            displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(), gridEditor_.isEditing());
            updateNavigation();
            break;

        case UserEventType::TouchGridCharacter:
            isRoverAlertActive_ = false;
            gridEditor_.selectCharacter(event.gridIndex, nowMs);
            displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(), gridEditor_.isEditing());
            updateNavigation();
            break;

        case UserEventType::TouchVersionFooter:
            runVisualSelfTest();
            break;

        case UserEventType::ModalButtonDown:
            if (event.buttonId == ModalButtonId::Snap && hasHeading_) {
                displayUi_.getCalModal().drawSnapButton(true, hasHeading_);
            } else if (event.buttonId == ModalButtonId::Reset) {
                displayUi_.getCalModal().drawResetButton(true);
            } else if (event.buttonId == ModalButtonId::Cancel) {
                displayUi_.getCalModal().drawCancelButton(true);
            }
            break;

        case UserEventType::ModalButtonUp:
            displayUi_.getCalModal().drawSnapButton(false, hasHeading_);
            displayUi_.getCalModal().drawResetButton(false);
            displayUi_.getCalModal().drawCancelButton(false);
            break;

        case UserEventType::HoldSaveButtonProgress:
            calModalOpenedMillis_ = nowMs;
            displayUi_.getCalModal().drawSaveButton(event.progress > 0.0f, event.progress);
            break;

        case UserEventType::SaveButtonCommitted:
            calibration_.commitProposed();
            setUiMode(UiMode::Normal);
            redrawMainScreen();
            break;

        case UserEventType::SnapOffsetTriggered:
            calModalOpenedMillis_ = nowMs;
            displayUi_.getCalModal().drawSnapButton(true, hasHeading_);
            if (hasHeading_) {
                calibration_.snapProposed(bearing_, gpsManager_.getBearingFilter().getRawHeading());
                displayUi_.getCalModal().drawProposedOffsetRow(calibration_.getProposedOffset());
            }
            pendingModalButtonRelease_ = ModalButtonId::Snap;
            pendingModalButtonReleaseStartMillis_ = nowMs;
            break;

        case UserEventType::ResetOffsetTriggered:
            calModalOpenedMillis_ = nowMs;
            displayUi_.getCalModal().drawResetButton(true);
            calibration_.resetProposed();
            displayUi_.getCalModal().drawProposedOffsetRow(calibration_.getProposedOffset());
            pendingModalButtonRelease_ = ModalButtonId::Reset;
            pendingModalButtonReleaseStartMillis_ = nowMs;
            break;

        case UserEventType::CancelModalTriggered:
            displayUi_.getCalModal().drawCancelButton(true);
            pendingModalButtonRelease_ = ModalButtonId::Cancel;
            pendingModalButtonReleaseStartMillis_ = nowMs;
            break;

        default:
            break;
    }
}

void AppController::update(unsigned long nowMs) {
#if defined(ARDUINO)
    if (nowMs == 0) nowMs = millis();
#endif

    // 1. Ingest GPS UART bytes & check staleness timeouts
    gpsManager_.update(nowMs);

    // 2. Process USB host commands (SIM, CAL, GPS, TEST)
    serialCli_.update(nowMs);

    // 3. Process Input events (rotary encoder and touch gestures)
    inputManager_.update(nowMs);
    UserEvent event;
    while (inputManager_.pollEvent(event)) {
        handleUserEvent(event, nowMs);
    }

    // 4. Non-blocking modal button unpress timer (rollover safe)
    if (pendingModalButtonRelease_ != ModalButtonId::None &&
        (nowMs - pendingModalButtonReleaseStartMillis_) >= 120) {
        if (pendingModalButtonRelease_ == ModalButtonId::Snap) {
            displayUi_.getCalModal().drawSnapButton(false, hasHeading_);
        } else if (pendingModalButtonRelease_ == ModalButtonId::Reset) {
            displayUi_.getCalModal().drawResetButton(false);
        } else if (pendingModalButtonRelease_ == ModalButtonId::Cancel) {
            setUiMode(UiMode::Normal);
            redrawMainScreen();
        }
        pendingModalButtonRelease_ = ModalButtonId::None;
    }

    // 5. Modal timeout check
    if (currentUiMode_ == UiMode::Calibrating) {
        if ((nowMs - calModalOpenedMillis_) >= CAL_MODAL_TIMEOUT_MS) {
            setUiMode(UiMode::Normal);
            redrawMainScreen();
        }
    }

    // 6. Edit timeout check
    if (gridEditor_.checkTimeout(nowMs)) {
        displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(), gridEditor_.isEditing());
    }
}
