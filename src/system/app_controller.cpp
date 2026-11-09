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
                             SerialCli &serialCli,
                             RoverLogManager *roverLogManager)
    : board_(board),
      displayUi_(displayUi),
      inputManager_(inputManager),
      gpsManager_(gpsManager),
      calibration_(calibration),
      gpsSimulator_(gpsSimulator),
      serialCli_(serialCli),
      roverLogManagerRef_(roverLogManager ? roverLogManager : &roverLogManagerOwned_),
      hasLocation_(false),
      hasTime_(false),
      hours_(0),
      minutes_(0),
      hasHeading_(false),
      currentBearingQuality_(BEARING_NONE),
      lastHeadingUpdate_(0),
      operationalMode_(OperationalMode::Bearing),
      lastRenderedMode_(OperationalMode::Bearing),
      lastValidDisplacementKm_(0.0),
      lastLocationUpdate_(0),
      lastRoverState_(RoverState::InTransitUnder16k),
      currentNowMs_(0),
      roverWarningUntilMs_(0),
      distance_(0.0),
      bearing_(0.0),
      revbearing_(0.0),
      currentUiMode_(UiMode::Normal),
      isSelfTesting_(false),
      calModalOpenedMillis_(0),
      pendingModalButtonRelease_(ModalButtonId::None),
      pendingModalButtonReleaseStartMillis_(0)
{
    std::strcpy(myGrid_, "JJ00AA00");
    std::strcpy(originGrid_, "JJ00AA00");
}

void AppController::setTargetGrid(const char *grid) {
    gridEditor_.setGrid(grid);
    updateNavigation();
}

void AppController::setMyGrid(const char *grid) {
    if (!grid) return;
    std::strncpy(myGrid_, grid, sizeof(myGrid_) - 1);
    myGrid_[sizeof(myGrid_) - 1] = '\0';
    updateNavigation();
}

void AppController::setCurrentPosition(const GeoPoint &pos) {
    hasLocation_ = true;
    currentPosition_ = pos;
    lastLocationUpdate_ = (currentNowMs_ > 0 ? currentNowMs_ : 1);
    char newGrid[9];
    geoPointToGrid(currentPosition_, newGrid);
    if (std::strcmp(myGrid_, newGrid) != 0) {
        std::strcpy(myGrid_, newGrid);
        if (currentUiMode_ == UiMode::Normal) {
            displayUi_.renderMyGrid(myGrid_);
        }
    }
    updateNavigation();
}

void AppController::setOperationalMode(OperationalMode mode) {
    if (operationalMode_ == OperationalMode::Rover && mode != OperationalMode::Rover) {
        displayUi_.clearRoverCard();
    }
    operationalMode_ = mode;
    updateNavigation();
}

void AppController::startRoverMode(unsigned long nowMs) {
#if defined(ARDUINO)
    if (nowMs == 0) nowMs = millis();
#endif
    if (nowMs > 0) currentNowMs_ = nowMs;

    // Guard: Prevent accidental restart if already in Rover Mode!
    if (operationalMode_ == OperationalMode::Rover) {
        roverWarningUntilMs_ = (nowMs > 0 ? nowMs : currentNowMs_) + ROVER_WARNING_DURATION_MS;
        displayUi_.renderRoverCardMessage("ROVER ACTIVE", COLOR_ROVER_TRANSIT_RED);
        return;
    }

    // Guard: Do not allow arming Rover Mode without a valid GPS location fix!
    if (!hasLocation_) {
        displayUi_.showStatusCard("WAIT GPS FIX", BLACK, RED);
        return;
    }

    // Initial Contest Departure Log (Stop #1): If contest history is empty,
    // record starting site as Stop #1 (0.0 km, ORIGIN).
    if (roverLogManagerRef_->isEmpty()) {
        uint32_t epoch = gpsManager_.getEpochTime();
        uint32_t ts = (epoch > 0) ? epoch : (hasTime_ ? (hours_ * 3600 + minutes_ * 60) : static_cast<uint32_t>(nowMs / 1000));
        RoverStop stop1(ts, currentPosition_.latitude, currentPosition_.longitude, myGrid_, 0.0f, false);
        roverLogManagerRef_->appendStop(stop1);
    }

    originPoint_ = currentPosition_;
    snprintf(originGrid_, sizeof(originGrid_), "%s", myGrid_);
    targetPosition_ = originPoint_;

    gridEditor_.setGrid(myGrid_);
    gridEditor_.setEditing(false); // Disarm edit mode on rover snapshot to protect origin & bezel

    operationalMode_ = OperationalMode::Rover;
    lastValidDisplacementKm_ = 0.0;
    lastRoverState_ = RoverState::InTransitUnder16k;

    displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(), gridEditor_.isEditing());
    updateNavigation();
}

void AppController::onTargetEdited(unsigned long nowMs) {
#if defined(ARDUINO)
    if (nowMs == 0) nowMs = millis();
#endif
    if (nowMs > 0) currentNowMs_ = nowMs;

    if (operationalMode_ == OperationalMode::Rover) {
        // Calculate final displacement from originPoint_
        double finalDistMeters = 0.0, finalBearing = 0.0;
        calculateWgs84(originPoint_, currentPosition_, finalDistMeters, finalBearing);
        float finalDistKm = static_cast<float>(finalDistMeters / 1000.0);
        bool qualified = (finalDistKm >= ROVER_ALERT_THRESHOLD_KM);

        uint32_t epoch = gpsManager_.getEpochTime();
        uint32_t ts = (epoch > 0) ? epoch : (hasTime_ ? (hours_ * 3600 + minutes_ * 60) : static_cast<uint32_t>(nowMs / 1000));
        RoverStop stop(ts, currentPosition_.latitude, currentPosition_.longitude, myGrid_, finalDistKm, qualified);
        roverLogManagerRef_->appendStop(stop);

        roverWarningUntilMs_ = 0;
        displayUi_.clearRoverCard();
        operationalMode_ = OperationalMode::Bearing;
    }
}

void AppController::stopRoverMode(unsigned long nowMs) {
#if defined(ARDUINO)
    if (nowMs == 0) nowMs = millis();
#endif
    if (nowMs > 0) currentNowMs_ = nowMs;

    if (operationalMode_ == OperationalMode::Rover) {
        roverWarningUntilMs_ = 0;
        onTargetEdited(nowMs);
        updateNavigation();
        redrawMainScreen();
    }
}

void AppController::discardLastStopAndRearm(unsigned long nowMs) {
#if defined(ARDUINO)
    if (nowMs == 0) nowMs = millis();
#endif
    if (nowMs > 0) currentNowMs_ = nowMs;
    roverWarningUntilMs_ = 0;

    // 1. Pop topmost stop from NVS history
    roverLogManagerRef_->popLastStop();

    // 2. Restore active origin baseline to prior stop (or current position if stack became empty)
    if (!roverLogManagerRef_->isEmpty()) {
        const RoverStop *restored = roverLogManagerRef_->getLastStop();
        originPoint_ = GeoPoint(restored->latitude, restored->longitude, true);
        snprintf(originGrid_, sizeof(originGrid_), "%s", restored->grid);
    } else {
        originPoint_ = currentPosition_;
        snprintf(originGrid_, sizeof(originGrid_), "%s", myGrid_);
    }
    targetPosition_ = originPoint_;
    gridEditor_.setGrid(originGrid_);

    // 3. Auto-close dialog and auto-rearm Rover Mode
    setUiMode(UiMode::Normal);
    operationalMode_ = OperationalMode::Rover;
    gridEditor_.setEditing(false);

    updateNavigation();
    redrawMainScreen();
}

void AppController::setUiMode(UiMode mode) {
    currentUiMode_ = mode;
    inputManager_.setUiMode(mode);
}

void AppController::setHeadingState(bool hasHeading, BearingQuality quality, unsigned long lastUpdate) {
    hasHeading_ = hasHeading;
    currentBearingQuality_ = quality;
    if (lastUpdate > 0) {
        lastHeadingUpdate_ = lastUpdate;
    }
}

void AppController::drawCurrentBearing() {
    BearingFilter &bf = gpsManager_.getBearingFilter();
    if (currentUiMode_ == UiMode::Calibrating) {
        displayUi_.getCalModal().drawRawHeading(bf.getRawHeading(), hasHeading_);
        return;
    }
    if (currentUiMode_ != UiMode::Normal) {
        return;
    }
    displayUi_.renderCurrentBearing(bf.getCalibratedHeading(calibration_.getOffset()),
                                  currentBearingQuality_, hasHeading_);
}

void AppController::recalculateGeodesy() {
    if (operationalMode_ == OperationalMode::Rover) {
        targetPosition_ = originPoint_;
        calculateWgs84(originPoint_, currentPosition_, distance_, bearing_);
        revbearing_ = calculateReverseBearing(currentPosition_, originPoint_);
    } else {
        gridToGeoPoint(gridEditor_.getGrid(), targetPosition_);
        calculateWgs84(currentPosition_, targetPosition_, distance_, bearing_);
#if !ENABLE_ARRL_DIST
        distance_ = calculateArrlContestDistance(myGrid_, gridEditor_.getGrid()) * 1000.0;
#endif
        revbearing_ = calculateReverseBearing(currentPosition_, targetPosition_);
    }
}

void AppController::renderNavigationRows() {
    if (currentUiMode_ == UiMode::Calibrating || currentUiMode_ == UiMode::RoverHistory) {
        return;
    }

    if (operationalMode_ == OperationalMode::Rover) {
        bool isFresh = hasLocation_ && (lastLocationUpdate_ > 0) &&
                       (currentNowMs_ == 0 || (currentNowMs_ >= lastLocationUpdate_ && (currentNowMs_ - lastLocationUpdate_ <= 3000)));
        double dispKm = distance_ / 1000.0;
        if (isFresh) {
            lastValidDisplacementKm_ = dispKm;
        } else {
            dispKm = lastValidDisplacementKm_;
        }
        bool isRevisit = roverLogManagerRef_->hasVisitedGrid(myGrid_);
        lastRoverState_ = evaluateRoverState(dispKm, isRevisit, isFresh);
        if (roverWarningUntilMs_ == 0 || (currentNowMs_ > 0 && currentNowMs_ >= roverWarningUntilMs_)) {
            displayUi_.renderRoverCard(lastRoverState_, dispKm, isRevisit);
        }
    } else {
        if (lastRenderedMode_ == OperationalMode::Rover) {
            displayUi_.clearRoverCard();
        }
        displayUi_.renderReverseBearingAndDistance(revbearing_, distance_);
    }
    lastRenderedMode_ = operationalMode_;

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
    lastRenderedMode_ = operationalMode_;
    BearingFilter &bf = gpsManager_.getBearingFilter();
    MainScreenState s;
    s.myGrid = myGrid_;
    s.targetGrid = gridEditor_.getGrid();
    s.cursorColumn = gridEditor_.getCursorColumn();
    s.isEditing = gridEditor_.isEditing();
    s.targetBearing = bearing_;
    s.revBearing = revbearing_;
    s.distanceMeters = distance_;
    s.isRoverMode = (operationalMode_ == OperationalMode::Rover);
    bool isFresh = hasLocation_ && (lastLocationUpdate_ > 0) &&
                   (currentNowMs_ == 0 || (currentNowMs_ >= lastLocationUpdate_ && (currentNowMs_ - lastLocationUpdate_ <= 3000)));
    if (s.isRoverMode) {
        if (!isFresh) {
            lastRoverState_ = RoverState::GpsLostStale;
        }
        s.roverState = lastRoverState_;
        s.roverDisplacementKm = isFresh ? (distance_ / 1000.0) : lastValidDisplacementKm_;
        s.isRoverRevisit = roverLogManagerRef_->hasVisitedGrid(myGrid_);
    } else {
        s.roverState = lastRoverState_;
        s.roverDisplacementKm = 0.0;
        s.isRoverRevisit = false;
    }
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
    roverLogManagerRef_->begin();

    gpsManager_.setSimulator(&gpsSimulator_);
#if defined(ARDUINO)
    gpsManager_.begin(GPS_BAUD, RXD2, TXD2);
#else
    gpsManager_.begin(GPS_BAUD);
#endif

    // Wire GPS Manager event callbacks (passing this as context)
    gpsManager_.setOnHeadingUpdated([](float /* heading */, BearingQuality quality, void *ctx) {
        auto *self = static_cast<AppController*>(ctx);
        if (!self || self->isSelfTesting_) return;
        self->hasHeading_ = self->gpsManager_.hasHeading();
        self->currentBearingQuality_ = quality;
        self->lastHeadingUpdate_ = self->gpsManager_.getLastHeadingUpdate();
        self->drawCurrentBearing();
    }, this);

    gpsManager_.setOnLocationUpdated([](const GeoPoint &pos, void *ctx) {
        auto *self = static_cast<AppController*>(ctx);
        if (!self || self->isSelfTesting_) return;
        self->setCurrentPosition(pos);
    }, this);

    gpsManager_.setOnTimeUpdated([](int h, int m, void *ctx) {
        auto *self = static_cast<AppController*>(ctx);
        if (!self || self->isSelfTesting_) return;
        self->hasTime_ = true;
        self->hours_ = h;
        self->minutes_ = m;
        if (self->currentUiMode_ == UiMode::Normal) {
            self->displayUi_.renderUtcTime(h, m, true);
        }
    }, this);

    // Wire Serial CLI callbacks (passing this as context)
    serialCli_.begin();
    serialCli_.setRoverLogManager(roverLogManagerRef_);
    serialCli_.setRoverCallbacks(
        [](void *ctx, unsigned long nowMs) {
            auto *self = static_cast<AppController*>(ctx);
            if (self) self->startRoverMode(nowMs);
        },
        [](void *ctx, unsigned long nowMs) {
            auto *self = static_cast<AppController*>(ctx);
            if (self) self->stopRoverMode(nowMs);
        },
        [](void *ctx, unsigned long nowMs) {
            auto *self = static_cast<AppController*>(ctx);
            if (self) self->discardLastStopAndRearm(nowMs);
        },
        this
    );

    serialCli_.setNavigationContextProvider([](void *ctx) -> CliNavContext {
        CliNavContext nctx;
        auto *self = static_cast<AppController*>(ctx);
        if (self) {
            nctx.targetBearing = self->bearing_;
            nctx.isRoverMode = self->isRoverMode();
            nctx.roverDistanceMeters = self->distance_;
            nctx.originLat = self->originPoint_.latitude;
            nctx.originLon = self->originPoint_.longitude;
            std::snprintf(nctx.originGrid, sizeof(nctx.originGrid), "%s", self->originGrid_);
            std::snprintf(nctx.currentGrid, sizeof(nctx.currentGrid), "%s", self->myGrid_);
            nctx.roverState = self->lastRoverState_;
            nctx.lastValidDisplacementKm = self->lastValidDisplacementKm_;
            unsigned long now = self->currentNowMs_;
#if defined(ARDUINO)
            if (now == 0) now = millis();
#endif
            nctx.isGpsFresh = self->hasLocation_ && (now >= self->lastLocationUpdate_) &&
                              ((now - self->lastLocationUpdate_) <= 3000);
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
            if (self->currentUiMode_ == UiMode::Normal) {
                self->displayUi_.renderMyGrid(self->myGrid_);
            }
        }
        if (self->gpsManager_.hasTime()) {
            self->hasTime_ = true;
            self->hours_ = self->gpsManager_.getHours();
            self->minutes_ = self->gpsManager_.getMinutes();
            if (self->currentUiMode_ == UiMode::Normal) {
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
        self->originPoint_ = originPoint;
        snprintf(self->originGrid_, sizeof(self->originGrid_), "%s", self->gridEditor_.getGrid());
        self->gridEditor_.setEditing(false); // Disarm edit mode on rover snapshot to protect origin & bezel
        self->targetPosition_ = originPoint;
        self->operationalMode_ = OperationalMode::Rover;

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
    if (nowMs > 0) currentNowMs_ = nowMs;

    switch (event.type) {
        case UserEventType::RotateClockwise:
            if (currentUiMode_ == UiMode::Calibrating) {
                calModalOpenedMillis_ = nowMs;
                calibration_.adjustProposedOffset(0.1f);
                displayUi_.getCalModal().drawProposedOffsetRow(calibration_.getProposedOffset());
            } else if (gridEditor_.isEditing()) {
                if (operationalMode_ == OperationalMode::Rover) {
                    onTargetEdited(nowMs);
                }
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
                if (operationalMode_ == OperationalMode::Rover) {
                    onTargetEdited(nowMs);
                }
                gridEditor_.rollChar(-1, nowMs);
                displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(), gridEditor_.isEditing());
                updateNavigation();
            }
            break;

        case UserEventType::EncoderButtonPressed:
            // Toggle edit mode on knob press in Bearing Mode only.
            // In Rover Mode, knob press is locked out to enforce 2-step natural exit
            // (touch character selection followed by physical rotary knob rotation).
            if (operationalMode_ == OperationalMode::Rover) {
                break;
            }
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
            startRoverMode(nowMs);
            break;

        case UserEventType::LongPressLocalGridTriggered:
            setUiMode(UiMode::RoverHistory);
            calModalOpenedMillis_ = nowMs;
            gridEditor_.setEditing(false);
            displayUi_.getRoverHistoryModal().drawModal(
                roverLogManagerRef_->getLastStop(),
                roverLogManagerRef_->getStopCount(),
                roverLogManagerRef_->getStopCount()
            );
            break;

        case UserEventType::TouchGridCharacter:
            // 2-step natural target exit protection: character touch arms editing and shows cursor,
            // but does NOT exit Rover Mode until the bezel is physically rotated.
            gridEditor_.selectCharacter(event.gridIndex, nowMs);
            displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(), gridEditor_.isEditing());
            updateNavigation();
            break;

        case UserEventType::TouchVersionFooter:
            runVisualSelfTest();
            break;

        case UserEventType::ModalButtonDown:
            if (currentUiMode_ == UiMode::Calibrating) {
                if (event.buttonId == ModalButtonId::Snap && hasHeading_) {
                    displayUi_.getCalModal().drawSnapButton(true, hasHeading_);
                } else if (event.buttonId == ModalButtonId::Reset) {
                    displayUi_.getCalModal().drawResetButton(true);
                } else if (event.buttonId == ModalButtonId::Cancel) {
                    displayUi_.getCalModal().drawCancelButton(true);
                }
            } else if (currentUiMode_ == UiMode::RoverHistory) {
                if (event.buttonId == ModalButtonId::Cancel) {
                    displayUi_.getRoverHistoryModal().drawCancelButton(true);
                }
            }
            break;

        case UserEventType::ModalButtonUp:
            if (currentUiMode_ == UiMode::Calibrating) {
                displayUi_.getCalModal().drawSnapButton(false, hasHeading_);
                displayUi_.getCalModal().drawResetButton(false);
                displayUi_.getCalModal().drawCancelButton(false);
            } else if (currentUiMode_ == UiMode::RoverHistory) {
                displayUi_.getRoverHistoryModal().drawDiscardButton(false, 0.0f);
                displayUi_.getRoverHistoryModal().drawCancelButton(false);
            }
            break;

        case UserEventType::HoldSaveButtonProgress:
            calModalOpenedMillis_ = nowMs;
            displayUi_.getCalModal().drawSaveButton(event.progress > 0.0f, event.progress);
            break;

        case UserEventType::HoldDiscardButtonProgress:
            calModalOpenedMillis_ = nowMs;
            displayUi_.getRoverHistoryModal().drawDiscardButton(event.progress > 0.0f, event.progress);
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

        case UserEventType::DiscardLastStopTriggered:
            displayUi_.getRoverHistoryModal().drawDiscardButton(true, 1.0f);
            pendingModalButtonRelease_ = ModalButtonId::Discard;
            pendingModalButtonReleaseStartMillis_ = nowMs;
            discardLastStopAndRearm(nowMs);
            break;

        case UserEventType::CancelModalTriggered:
            if (currentUiMode_ == UiMode::RoverHistory) {
                displayUi_.getRoverHistoryModal().drawCancelButton(true);
            } else {
                displayUi_.getCalModal().drawCancelButton(true);
            }
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
    if (nowMs > 0) currentNowMs_ = nowMs;

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
        } else if (pendingModalButtonRelease_ == ModalButtonId::Discard) {
            // Already handled by discardLastStopAndRearm
        }
        pendingModalButtonRelease_ = ModalButtonId::None;
    }

    // 5. Modal timeout check
    if (currentUiMode_ == UiMode::Calibrating || currentUiMode_ == UiMode::RoverHistory) {
        if ((nowMs - calModalOpenedMillis_) >= CAL_MODAL_TIMEOUT_MS) {
            setUiMode(UiMode::Normal);
            redrawMainScreen();
        }
    }

    // 6. In Rover Mode, evaluate GPS watchdog staleness and warning expiration
    if (operationalMode_ == OperationalMode::Rover && currentUiMode_ == UiMode::Normal) {
        bool isRevisit = roverLogManagerRef_->hasVisitedGrid(myGrid_);
        if (roverWarningUntilMs_ > 0 && nowMs >= roverWarningUntilMs_) {
            roverWarningUntilMs_ = 0;
            displayUi_.renderRoverCard(lastRoverState_, lastValidDisplacementKm_, isRevisit);
        }

        bool isFresh = hasLocation_ && (lastLocationUpdate_ > 0) && (nowMs >= lastLocationUpdate_) && ((nowMs - lastLocationUpdate_) <= 3000);
        if (!isFresh) {
            if (lastRoverState_ != RoverState::GpsLostStale) {
                lastRoverState_ = RoverState::GpsLostStale;
                if (roverWarningUntilMs_ == 0) {
                    displayUi_.renderRoverCard(RoverState::GpsLostStale, lastValidDisplacementKm_, isRevisit);
                }
            }
        }
    }

    // 7. Edit timeout check
    if (gridEditor_.checkTimeout(nowMs)) {
        displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(), gridEditor_.isEditing());
    }
}
