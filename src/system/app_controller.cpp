#include "app_controller.h"
#include "self_test.h"
#include <cmath>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

static AppController *s_appInstance = nullptr;

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
      pendingModalButtonReleaseMillis_(0)
{
    std::strcpy(myGrid_, "JJ00AA00");
    s_appInstance = this;
}

void AppController::setTargetGrid(const char *grid) {
    gridEditor_.setGrid(grid);
    updateBearings();
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
    updateBearings();
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

void AppController::updateBearings() {
    gridToGeoPoint(gridEditor_.getGrid(), targetPosition_);
    calculateWgs84(currentPosition_, targetPosition_, distance_, bearing_);
    revbearing_ = calculateReverseBearing(currentPosition_, targetPosition_);

    if (currentUiMode_ == UiMode::Calibrating) {
        return;
    }

    displayUi_.renderReverseBearingAndDistance(revbearing_, distance_, isRoverAlertActive_);
    drawCurrentBearing();
    displayUi_.renderCalibrationPill(calibration_.getOffset(), calibration_.isCalibrated());
    displayUi_.renderTargetBearing(bearing_);
    displayUi_.renderArrlDistance(myGrid_, gridEditor_.getGrid());
}

void AppController::redrawMainScreen() {
    BearingFilter &bf = gpsManager_.getBearingFilter();
    displayUi_.redrawMainScreen(myGrid_, gridEditor_.getGrid(), gridEditor_.getCursorColumn(), gridEditor_.isEditing(),
                               bearing_, revbearing_, distance_, isRoverAlertActive_,
                               bf.getCalibratedHeading(calibration_.getOffset()),
                               currentBearingQuality_, hasHeading_,
                               calibration_.getOffset(), calibration_.isCalibrated(),
                               hours_, minutes_, hasTime_);
}

void AppController::runVisualSelfTest() {
    SelfTestContext ctx{
        displayUi_, gpsManager_, calibration_, serialCli_, inputManager_,
        gridEditor_.getGridBuffer(), myGrid_, currentPosition_, targetPosition_,
        distance_, bearing_, revbearing_, hasHeading_, currentBearingQuality_,
        lastHeadingUpdate_, isRoverAlertActive_,
        gridEditor_.isEditingRef(), gridEditor_.cursorColumnRef(),
        currentUiMode_, hours_, minutes_, hasTime_,
        []() { if (s_appInstance) s_appInstance->redrawMainScreen(); },
        []() { if (s_appInstance) s_appInstance->updateBearings(); },
        []() { if (s_appInstance) s_appInstance->drawCurrentBearing(); }
    };
    SelfTest::run(ctx);
}

void AppController::begin() {
    s_appInstance = this;

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

    // Wire GPS Manager event callbacks
    gpsManager_.setOnHeadingUpdated([](float /* heading */, BearingQuality quality) {
        if (!s_appInstance) return;
        s_appInstance->hasHeading_ = s_appInstance->gpsManager_.hasHeading();
        s_appInstance->currentBearingQuality_ = quality;
        s_appInstance->lastHeadingUpdate_ = s_appInstance->gpsManager_.getLastHeadingUpdate();
        s_appInstance->drawCurrentBearing();
    });

    gpsManager_.setOnLocationUpdated([](const GeoPoint &pos) {
        if (!s_appInstance) return;
        s_appInstance->setCurrentPosition(pos);
    });

    gpsManager_.setOnTimeUpdated([](int h, int m) {
        if (!s_appInstance) return;
        s_appInstance->hasTime_ = true;
        s_appInstance->hours_ = h;
        s_appInstance->minutes_ = m;
        if (s_appInstance->currentUiMode_ != UiMode::Calibrating) {
            s_appInstance->displayUi_.renderUtcTime(h, m, true);
        }
    });

    // Wire Serial CLI callbacks
    serialCli_.begin();
    serialCli_.setNavigationContextProvider([]() -> CliNavContext {
        CliNavContext ctx;
        if (s_appInstance) {
            ctx.targetBearing = s_appInstance->bearing_;
            ctx.isRoverAlertActive = s_appInstance->isRoverAlertActive_;
            ctx.roverDistanceMeters = s_appInstance->distance_;
        }
        return ctx;
    });

    serialCli_.setRunTestCallback([]() {
        if (s_appInstance) s_appInstance->runVisualSelfTest();
    });

    serialCli_.setStateChangedCallback([]() {
        if (!s_appInstance) return;
        s_appInstance->hasHeading_ = s_appInstance->gpsManager_.hasHeading();
        s_appInstance->currentBearingQuality_ = s_appInstance->gpsManager_.getBearingQuality();
        s_appInstance->lastHeadingUpdate_ = s_appInstance->gpsManager_.getLastHeadingUpdate();
        if (s_appInstance->gpsManager_.hasLocation()) {
            s_appInstance->hasLocation_ = true;
            s_appInstance->currentPosition_ = s_appInstance->gpsManager_.getLocation();
            geoPointToGrid(s_appInstance->currentPosition_, s_appInstance->myGrid_);
            if (s_appInstance->currentUiMode_ != UiMode::Calibrating) {
                s_appInstance->displayUi_.renderMyGrid(s_appInstance->myGrid_);
            }
        }
        if (s_appInstance->gpsManager_.hasTime()) {
            s_appInstance->hasTime_ = true;
            s_appInstance->hours_ = s_appInstance->gpsManager_.getHours();
            s_appInstance->minutes_ = s_appInstance->gpsManager_.getMinutes();
            if (s_appInstance->currentUiMode_ != UiMode::Calibrating) {
                s_appInstance->displayUi_.renderUtcTime(s_appInstance->hours_, s_appInstance->minutes_, true);
            }
        }
        s_appInstance->updateBearings();
        s_appInstance->drawCurrentBearing();
    });

    serialCli_.setRoverSimCallback([](const char *orig_grid, double dist_km) {
        if (!s_appInstance) return;
        GeoPoint originPoint = s_appInstance->currentPosition_;
        if (orig_grid && strlen(orig_grid) >= 4) {
            gridToGeoPoint(orig_grid, originPoint);
            s_appInstance->gridEditor_.setGrid(orig_grid);
        }
        s_appInstance->targetPosition_ = originPoint;
        s_appInstance->isRoverAlertActive_ = true;

        GeoPoint simulatedPoint = GpsSimulator::calculateRoverDisplacement(originPoint, dist_km, 45.0);
        s_appInstance->gpsManager_.setLocation(simulatedPoint);
        s_appInstance->displayUi_.renderTargetGrid(s_appInstance->gridEditor_.getGrid(),
                                                  s_appInstance->gridEditor_.getCursorColumn(),
                                                  s_appInstance->gridEditor_.isEditing());
        s_appInstance->updateBearings();

#if defined(ARDUINO)
        int rdist = (int)std::floor(s_appInstance->distance_ / 1000.0);
        Serial.printf("[SIM ROVER] Origin: %s (%.5f, %.5f) | Current: %s (%.5f, %.5f)\r\n",
                      s_appInstance->gridEditor_.getGrid(), s_appInstance->targetPosition_.latitude, s_appInstance->targetPosition_.longitude,
                      s_appInstance->myGrid_, s_appInstance->currentPosition_.latitude, s_appInstance->currentPosition_.longitude);
        Serial.printf("[SIM ROVER] Distance: %5.1f km (%d km) | Alert: %s\r\n",
                      s_appInstance->distance_ / 1000.0, rdist, (rdist < 16) ? "ACTIVE (RED on WHITE)" : "CLEAR (>= 16 km)");
#endif
    });

    geoPointToGrid(currentPosition_, myGrid_);
    displayUi_.renderMyGrid(myGrid_);

    gridEditor_.recordActivity(0);
    displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(), gridEditor_.isEditing());
    updateBearings();

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
                updateBearings();
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
                updateBearings();
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
            for (int i = 0; i < 6; i++) gridEditor_.getGridBuffer()[i] = myGrid_[i];
            gridEditor_.getGridBuffer()[6] = '\0';
            targetPosition_ = currentPosition_;
            isRoverAlertActive_ = true;
            displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(), gridEditor_.isEditing());
            updateBearings();
            break;

        case UserEventType::TouchGridCharacter:
            gridEditor_.selectCharacter(event.gridIndex, nowMs);
            displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(), gridEditor_.isEditing());
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
            pendingModalButtonReleaseMillis_ = nowMs + 120;
            break;

        case UserEventType::ResetOffsetTriggered:
            calModalOpenedMillis_ = nowMs;
            displayUi_.getCalModal().drawResetButton(true);
            calibration_.resetProposed();
            displayUi_.getCalModal().drawProposedOffsetRow(calibration_.getProposedOffset());
            pendingModalButtonRelease_ = ModalButtonId::Reset;
            pendingModalButtonReleaseMillis_ = nowMs + 120;
            break;

        case UserEventType::CancelModalTriggered:
            displayUi_.getCalModal().drawCancelButton(true);
            pendingModalButtonRelease_ = ModalButtonId::Cancel;
            pendingModalButtonReleaseMillis_ = nowMs + 120;
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

    // 4. Non-blocking modal button unpress timer
    if (pendingModalButtonRelease_ != ModalButtonId::None && nowMs >= pendingModalButtonReleaseMillis_) {
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
