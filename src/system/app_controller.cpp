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
                             RoverLogManager *roverLogManager,
                             BleSerialManager *bleManager,
                             QsoLogManager *qsoLogManager,
                             TargetPresetManager *targetPresetManager)
    : board_(board),
      displayUi_(displayUi),
      inputManager_(inputManager),
      gpsManager_(gpsManager),
      calibration_(calibration),
      gpsSimulator_(gpsSimulator),
      serialCli_(serialCli),
      roverLogManagerRef_(roverLogManager ? roverLogManager : &roverLogManagerOwned_),
      qsoLogManagerRef_(qsoLogManager ? qsoLogManager : &qsoLogManagerOwned_),
      targetPresetManagerRef_(targetPresetManager ? targetPresetManager : &targetPresetManagerOwned_),
      bleManager_(bleManager),
      bleConnected_(false),
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
      qsoCardOpenedMillis_(0),
      targetModalOpenedMillis_(0),
      selectedPresetIndex_(0),
      presetScrollOffset_(0),
      pendingModalButtonRelease_(ModalButtonId::None),
      pendingModalButtonReleaseStartMillis_(0)
{
    std::strcpy(myGrid_, "JJ00AA00");
    std::strcpy(originGrid_, "JJ00AA00");
}

void AppController::setTargetGrid(const char *grid, unsigned long nowMs) {
#if defined(ARDUINO)
    if (nowMs == 0) nowMs = millis();
#endif
    if (nowMs > 0) currentNowMs_ = nowMs;

    if (operationalMode_ == OperationalMode::Rover) {
        onTargetEdited(currentNowMs_);
    }
    gridEditor_.setEditing(false);
    gridEditor_.setGrid(grid);
    if (currentUiMode_ == UiMode::Normal) {
        displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(),
                                    gridEditor_.isEditing(), resolveTargetLabel(gridEditor_.getGrid()));
    }
    updateNavigation();
    if (lastRenderedMode_ == OperationalMode::Rover) {
        redrawMainScreen();
    }
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

void AppController::showQsoCard(const QsoEntry &qso, size_t qsoIndex, unsigned long nowMs) {
    if (isSelfTesting_) return;
#if defined(ARDUINO)
    if (nowMs == 0) nowMs = millis();
#endif
    if (nowMs > 0) currentNowMs_ = nowMs;
    pendingModalButtonRelease_ = ModalButtonId::None;
    qsoCardOpenedMillis_ = nowMs;
    setUiMode(UiMode::QsoCard);
    displayUi_.getQsoCard().show(qso, qsoIndex, nowMs);
    displayUi_.getQsoCard().drawModal(qso, qsoIndex);
}

void AppController::dismissQsoCard() {
    displayUi_.getQsoCard().dismiss();
    setUiMode(UiMode::Normal);
    redrawMainScreen();
}

void AppController::setUiMode(UiMode mode) {
    currentUiMode_ = mode;
    inputManager_.setUiMode(mode);
    if (mode != UiMode::Normal) {
        gridEditor_.setEditing(false);
    }
}

const char* AppController::resolveTargetLabel(const char *grid) const {
    if (!grid || operationalMode_ == OperationalMode::Rover) {
        return nullptr;
    }
    if (targetPresetManagerRef_) {
        const TargetPreset *preset = targetPresetManagerRef_->findByGrid(grid);
        if (preset && preset->label[0] != '\0') {
            return preset->label;
        }
    }
    return TargetPresetManager::lookupRegionalSummit(grid);
}

uint32_t AppController::currentTimestampSec(unsigned long nowMs) const {
    uint32_t epoch = gpsManager_.getEpochTime();
    if (epoch > 0) return epoch;
    if (hasTime_) return static_cast<uint32_t>(hours_ * 3600 + minutes_ * 60);
    return static_cast<uint32_t>(nowMs / 1000);
}

void AppController::clampTargetModalSelection() {
    size_t count = targetPresetManagerRef_ ? targetPresetManagerRef_->getCount() : 0;
    if (count == 0) {
        selectedPresetIndex_ = 0;
        presetScrollOffset_ = 0;
        return;
    }
    if (selectedPresetIndex_ >= count) {
        selectedPresetIndex_ = count - 1;
    }
    if (selectedPresetIndex_ < presetScrollOffset_) {
        presetScrollOffset_ = selectedPresetIndex_;
    } else if (selectedPresetIndex_ >= presetScrollOffset_ + TARGET_LIST_VISIBLE_ROWS) {
        presetScrollOffset_ = selectedPresetIndex_ - TARGET_LIST_VISIBLE_ROWS + 1;
    }
    if (count <= TARGET_LIST_VISIBLE_ROWS) {
        presetScrollOffset_ = 0;
    } else if (presetScrollOffset_ > count - TARGET_LIST_VISIBLE_ROWS) {
        presetScrollOffset_ = count - TARGET_LIST_VISIBLE_ROWS;
    }
}

void AppController::drawTargetListModal() {
    if (!targetPresetManagerRef_) return;
    const auto &presets = targetPresetManagerRef_->getPresets();
    displayUi_.getTargetListModal().drawModal(
        presets.empty() ? nullptr : presets.data(),
        presets.size(),
        targetPresetManagerRef_->getMaxTargets(),
        selectedPresetIndex_,
        presetScrollOffset_,
        currentPosition_,
        hasLocation_);
}

void AppController::drawTargetListRows() {
    if (!targetPresetManagerRef_) return;
    const auto &presets = targetPresetManagerRef_->getPresets();
    displayUi_.getTargetListModal().drawPresetList(
        presets.empty() ? nullptr : presets.data(),
        presets.size(),
        selectedPresetIndex_,
        presetScrollOffset_,
        currentPosition_,
        hasLocation_);
}

void AppController::loadSelectedTargetPreset(unsigned long nowMs) {
    if (!targetPresetManagerRef_) return;
    const TargetPreset *preset = targetPresetManagerRef_->getPreset(selectedPresetIndex_);
    if (!preset) return;

    char loadedGrid[7];
    std::strncpy(loadedGrid, preset->grid, sizeof(loadedGrid) - 1);
    loadedGrid[sizeof(loadedGrid) - 1] = '\0';

    targetPresetManagerRef_->recordUse(selectedPresetIndex_, currentTimestampSec(nowMs));
    gridEditor_.setGrid(loadedGrid);
    gridEditor_.setEditing(false);
    if (operationalMode_ == OperationalMode::Rover) {
        stopRoverMode(nowMs);
    }
    setUiMode(UiMode::Normal);
    updateNavigation();
    redrawMainScreen();
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
    if (currentUiMode_ == UiMode::Calibrating || currentUiMode_ == UiMode::RoverHistory ||
        currentUiMode_ == UiMode::QsoCard || currentUiMode_ == UiMode::TargetList) {
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
    s.targetLabel = resolveTargetLabel(gridEditor_.getGrid());
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
    s.bleConnected = bleConnected_;

    displayUi_.redrawMainScreen(s);
}

void AppController::handleBleConnectionChanged(bool connected) {
    bleConnected_ = connected;
    serialCli_.handleBleConnectionChanged(connected);
    if (currentUiMode_ == UiMode::Normal) {
        displayUi_.renderBleStatus(connected);
    }
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
    qsoLogManagerRef_->begin();
    targetPresetManagerRef_->begin();

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
    if (bleManager_) {
        bleManager_->begin();
        bleManager_->setConnectionCallback([](bool connected, void *ctx) {
            auto *self = static_cast<AppController*>(ctx);
            if (self) self->handleBleConnectionChanged(connected);
        }, this);
        serialCli_.setBleManager(bleManager_);
    }
    serialCli_.setRoverLogManager(roverLogManagerRef_);
    serialCli_.setQsoLogManager(qsoLogManagerRef_);
    serialCli_.setTargetPresetManager(targetPresetManagerRef_);
    serialCli_.setQsoLoggedCallback([](const QsoEntry &entry, size_t qsoIndex, unsigned long nowMs, void *ctx) {
        auto *self = static_cast<AppController*>(ctx);
        if (self) self->showQsoCard(entry, qsoIndex, nowMs);
    }, this);
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

    serialCli_.setTargetGridCallback([](const char *grid, unsigned long nowMs, void *ctx) {
        auto *self = static_cast<AppController*>(ctx);
        if (self) self->setTargetGrid(grid, nowMs);
    }, this);

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
            std::snprintf(nctx.targetGrid, sizeof(nctx.targetGrid), "%s", self->gridEditor_.getGrid());
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
    serialCli_.setRestartCallback([](void *ctx) {
        auto *self = static_cast<AppController*>(ctx);
        if (self) self->getBoard().restart();
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
        self->serialCli_.broadcastFormat("[SIM ROVER] Origin: %s (%.5f, %.5f) | Current: %s (%.5f, %.5f)\r\n",
                      self->gridEditor_.getGrid(), self->targetPosition_.latitude, self->targetPosition_.longitude,
                      self->myGrid_, self->currentPosition_.latitude, self->currentPosition_.longitude);
        self->serialCli_.broadcastFormat("[SIM ROVER] Distance: %5.1f km (%d km) | Alert: %s\r\n",
                      self->distance_ / 1000.0, rdist, (rdist < 16) ? "ACTIVE (RED on WHITE)" : "CLEAR (>= 16 km)");
#endif
    }, this);

    geoPointToGrid(currentPosition_, myGrid_);
    displayUi_.renderMyGrid(myGrid_);

    gridEditor_.recordActivity(0);
    displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(),
                                gridEditor_.isEditing(), resolveTargetLabel(gridEditor_.getGrid()));
    updateNavigation();

    displayUi_.renderUtcTime(0, 0, false);
    displayUi_.showFooterText(FW_VERSION, DIM_GRAY_TEXT);
}

void AppController::handleUserEvent(const UserEvent &event, unsigned long nowMs) {
#if defined(ARDUINO)
    if (nowMs == 0) nowMs = millis();
#endif
    if (nowMs > 0) currentNowMs_ = nowMs;

    if (currentUiMode_ == UiMode::QsoCard) {
        if (event.type == UserEventType::EncoderButtonPressed ||
            event.type == UserEventType::DismissQsoCardTriggered ||
            event.type == UserEventType::RotateClockwise ||
            event.type == UserEventType::RotateCounterClockwise ||
            event.type == UserEventType::CancelModalTriggered ||
            event.type == UserEventType::TouchLocalGrid ||
            event.type == UserEventType::TouchGridCharacter ||
            event.type == UserEventType::TouchCalibrationPill ||
            event.type == UserEventType::TouchVersionFooter) {
            dismissQsoCard();
            return;
        }
    }

    switch (event.type) {
        case UserEventType::RotateClockwise:
            if (currentUiMode_ == UiMode::Calibrating) {
                calModalOpenedMillis_ = nowMs;
                calibration_.adjustProposedOffset(0.1f);
                displayUi_.getCalModal().drawProposedOffsetRow(calibration_.getProposedOffset());
            } else if (currentUiMode_ == UiMode::TargetList) {
                targetModalOpenedMillis_ = nowMs;
                size_t count = targetPresetManagerRef_ ? targetPresetManagerRef_->getCount() : 0;
                if (count > 0 && selectedPresetIndex_ + 1 < count) {
                    selectedPresetIndex_++;
                    clampTargetModalSelection();
                    drawTargetListRows();
                }
            } else if (currentUiMode_ == UiMode::Normal && gridEditor_.isEditing()) {
                if (operationalMode_ == OperationalMode::Rover) {
                    onTargetEdited(nowMs);
                }
                gridEditor_.rollChar(+1, nowMs);
                displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(),
                                            gridEditor_.isEditing(), resolveTargetLabel(gridEditor_.getGrid()));
                updateNavigation();
            }
            break;

        case UserEventType::RotateCounterClockwise:
            if (currentUiMode_ == UiMode::Calibrating) {
                calModalOpenedMillis_ = nowMs;
                calibration_.adjustProposedOffset(-0.1f);
                displayUi_.getCalModal().drawProposedOffsetRow(calibration_.getProposedOffset());
            } else if (currentUiMode_ == UiMode::TargetList) {
                targetModalOpenedMillis_ = nowMs;
                size_t count = targetPresetManagerRef_ ? targetPresetManagerRef_->getCount() : 0;
                if (count > 0 && selectedPresetIndex_ > 0) {
                    selectedPresetIndex_--;
                    clampTargetModalSelection();
                    drawTargetListRows();
                }
            } else if (currentUiMode_ == UiMode::Normal && gridEditor_.isEditing()) {
                if (operationalMode_ == OperationalMode::Rover) {
                    onTargetEdited(nowMs);
                }
                gridEditor_.rollChar(-1, nowMs);
                displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(),
                                            gridEditor_.isEditing(), resolveTargetLabel(gridEditor_.getGrid()));
                updateNavigation();
            }
            break;

        case UserEventType::EncoderButtonPressed:
            if (currentUiMode_ == UiMode::TargetList) {
                loadSelectedTargetPreset(nowMs);
                break;
            }
            // Toggle edit mode on knob press in Normal Bearing Mode only.
            // Disarmed in modals (Calibrating, RoverHistory) to prevent target grid poking through.
            // Locked out in Rover Mode to enforce 2-step natural exit.
            if (currentUiMode_ != UiMode::Normal || operationalMode_ == OperationalMode::Rover) {
                break;
            }
            gridEditor_.setEditing(!gridEditor_.isEditing());
            gridEditor_.recordActivity(nowMs);
            displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(),
                                        gridEditor_.isEditing(), resolveTargetLabel(gridEditor_.getGrid()));
            break;

        case UserEventType::TouchCalibrationPill:
            if (currentUiMode_ != UiMode::Normal) {
                break;
            }
            if (hasHeading_) {
                calibration_.snapProposed(bearing_, gpsManager_.getBearingFilter().getRawHeading());
            } else {
                calibration_.syncProposedFromActive();
            }
            setUiMode(UiMode::Calibrating);
            calModalOpenedMillis_ = nowMs;
            displayUi_.getCalModal().drawModal(bearing_, gpsManager_.getBearingFilter().getRawHeading(), hasHeading_,
                                              calibration_.getOffset(), calibration_.getProposedOffset());
            break;

        case UserEventType::TouchLocalGrid:
            if (currentUiMode_ != UiMode::Normal) {
                break;
            }
            startRoverMode(nowMs);
            break;

        case UserEventType::LongPressLocalGridTriggered:
            if (currentUiMode_ != UiMode::Normal) {
                break;
            }
            setUiMode(UiMode::RoverHistory);
            calModalOpenedMillis_ = nowMs;
            displayUi_.getRoverHistoryModal().drawModal(
                roverLogManagerRef_->getLastStop(),
                roverLogManagerRef_->getStopCount(),
                roverLogManagerRef_->getStopCount()
            );
            break;

        case UserEventType::TouchGridCharacter:
            if (currentUiMode_ != UiMode::Normal) {
                break;
            }
            // 2-step natural target exit protection: character touch arms editing and shows cursor,
            // but does NOT exit Rover Mode until the bezel is physically rotated.
            gridEditor_.selectCharacter(event.gridIndex, nowMs);
            displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(),
                                        gridEditor_.isEditing(), resolveTargetLabel(gridEditor_.getGrid()));
            updateNavigation();
            break;

        case UserEventType::LongPressTargetGridTriggered:
            if (currentUiMode_ != UiMode::Normal) {
                break;
            }
            setUiMode(UiMode::TargetList);
            targetModalOpenedMillis_ = nowMs;
            selectedPresetIndex_ = 0;
            presetScrollOffset_ = 0;
            drawTargetListModal();
            break;

        case UserEventType::TouchVersionFooter:
            if (currentUiMode_ != UiMode::Normal) {
                break;
            }
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
            } else if (currentUiMode_ == UiMode::QsoCard) {
                if (event.buttonId == ModalButtonId::Ok) {
                    displayUi_.getQsoCard().drawOkButton(true);
                }
            } else if (currentUiMode_ == UiMode::TargetList) {
                bool hasItems = targetPresetManagerRef_ && !targetPresetManagerRef_->isEmpty();
                if (event.buttonId == ModalButtonId::TargetLoad) {
                    displayUi_.getTargetListModal().drawLoadButton(true, hasItems);
                } else if (event.buttonId == ModalButtonId::TargetDelete) {
                    displayUi_.getTargetListModal().drawDeleteButton(true, hasItems);
                } else if (event.buttonId == ModalButtonId::TargetSave) {
                    displayUi_.getTargetListModal().drawSaveButton(true);
                } else if (event.buttonId == ModalButtonId::Cancel) {
                    displayUi_.getTargetListModal().drawCancelButton(true);
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
            } else if (currentUiMode_ == UiMode::QsoCard) {
                displayUi_.getQsoCard().drawOkButton(false);
            } else if (currentUiMode_ == UiMode::TargetList) {
                bool hasItems = targetPresetManagerRef_ && !targetPresetManagerRef_->isEmpty();
                displayUi_.getTargetListModal().drawLoadButton(false, hasItems);
                displayUi_.getTargetListModal().drawDeleteButton(false, hasItems);
                displayUi_.getTargetListModal().drawSaveButton(false);
                displayUi_.getTargetListModal().drawCancelButton(false);
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
            } else if (currentUiMode_ == UiMode::TargetList) {
                displayUi_.getTargetListModal().drawCancelButton(true);
            } else {
                displayUi_.getCalModal().drawCancelButton(true);
            }
            pendingModalButtonRelease_ = ModalButtonId::Cancel;
            pendingModalButtonReleaseStartMillis_ = nowMs;
            break;

        case UserEventType::TargetRowTouched:
            if (currentUiMode_ == UiMode::TargetList && targetPresetManagerRef_ && event.gridIndex >= 0) {
                targetModalOpenedMillis_ = nowMs;
                size_t tappedIdx = presetScrollOffset_ + static_cast<size_t>(event.gridIndex);
                if (tappedIdx < targetPresetManagerRef_->getCount()) {
                    selectedPresetIndex_ = tappedIdx;
                    clampTargetModalSelection();
                    drawTargetListRows();
                }
            }
            break;

        case UserEventType::LoadTargetTriggered:
            if (currentUiMode_ == UiMode::TargetList) {
                targetModalOpenedMillis_ = nowMs;
                loadSelectedTargetPreset(nowMs);
            }
            break;

        case UserEventType::DeleteTargetTriggered:
            if (currentUiMode_ == UiMode::TargetList && targetPresetManagerRef_) {
                targetModalOpenedMillis_ = nowMs;
                if (targetPresetManagerRef_->removePreset(selectedPresetIndex_)) {
                    clampTargetModalSelection();
                    drawTargetListModal();
                }
            }
            break;

        case UserEventType::SaveTargetTriggered:
            if (currentUiMode_ == UiMode::TargetList && targetPresetManagerRef_) {
                targetModalOpenedMillis_ = nowMs;
                const char *activeGrid = gridEditor_.getGrid();
                if (targetPresetManagerRef_->addOrUpdatePreset(activeGrid, nullptr, currentTimestampSec(nowMs))) {
                    int idx = targetPresetManagerRef_->findIndexByGrid(activeGrid);
                    if (idx >= 0) {
                        selectedPresetIndex_ = static_cast<size_t>(idx);
                    }
                    clampTargetModalSelection();
                    drawTargetListModal();
                }
            }
            break;

        case UserEventType::HoldClearButtonProgress:
            if (currentUiMode_ == UiMode::TargetList) {
                targetModalOpenedMillis_ = nowMs;
                bool hasItems = targetPresetManagerRef_ && !targetPresetManagerRef_->isEmpty();
                displayUi_.getTargetListModal().drawClearButton(event.progress > 0.0f, event.progress, hasItems);
            }
            break;

        case UserEventType::ClearButtonCommitted:
            if (currentUiMode_ == UiMode::TargetList && targetPresetManagerRef_) {
                targetModalOpenedMillis_ = nowMs;
                targetPresetManagerRef_->clearAll();
                selectedPresetIndex_ = 0;
                presetScrollOffset_ = 0;
                drawTargetListModal();
            }
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
    } else if (currentUiMode_ == UiMode::QsoCard) {
        unsigned long cardTimeoutMs = qsoLogManagerRef_ ? qsoLogManagerRef_->getCardTimeoutMs() : QSO_MODAL_TIMEOUT_MS;
        if (displayUi_.getQsoCard().checkTimeout(nowMs, cardTimeoutMs)) {
            dismissQsoCard();
        }
    } else if (currentUiMode_ == UiMode::TargetList) {
        if ((nowMs - targetModalOpenedMillis_) >= TARGET_MODAL_TIMEOUT_MS) {
            pendingModalButtonRelease_ = ModalButtonId::None;
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
        if (currentUiMode_ == UiMode::Normal) {
            displayUi_.renderTargetGrid(gridEditor_.getGrid(), gridEditor_.getCursorColumn(),
                                        gridEditor_.isEditing(), resolveTargetLabel(gridEditor_.getGrid()));
        }
    }
}
