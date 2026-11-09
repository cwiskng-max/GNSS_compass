#include "input_manager.h"
#include <cmath>

#if defined(ARDUINO)
volatile int InputManager::s_counter = 0;
volatile int InputManager::s_encState = 0;
volatile int InputManager::s_oldState = -1;
volatile bool InputManager::s_hasChanged = false;

void IRAM_ATTR InputManager::encoderIrq() {
    s_encState = digitalRead(ENCODER_CLK);
    if (s_encState != s_oldState) {
        s_counter += (digitalRead(ENCODER_DT) == s_encState) ? +1 : -1;
        s_oldState = s_encState;
        s_hasChanged = true;
    }
}
#endif

InputManager::InputManager() {
    clearEvents();
}

void InputManager::begin() {
#if defined(ARDUINO)
    pinMode(ENCODER_CLK, INPUT_PULLUP);
    pinMode(ENCODER_DT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoderIrq, CHANGE);

    if (!tsPanel_.begin(&Wire, I2C_TOUCH_ADDR)) {
        Serial.println(F("No touchscreen found"));
    }
#endif
}

void InputManager::clearEvents() {
    queueHead_ = 0;
    queueTail_ = 0;
    queueCount_ = 0;
}

bool InputManager::queueEvent(const UserEvent &evt) {
    if (queueCount_ >= EVENT_QUEUE_CAPACITY) {
        return false;
    }
    eventQueue_[queueTail_] = evt;
    queueTail_ = (queueTail_ + 1) % EVENT_QUEUE_CAPACITY;
    queueCount_++;
    return true;
}

bool InputManager::pollEvent(UserEvent &outEvent) {
    if (queueCount_ == 0) {
        return false;
    }
    outEvent = eventQueue_[queueHead_];
    queueHead_ = (queueHead_ + 1) % EVENT_QUEUE_CAPACITY;
    queueCount_--;
    return true;
}

void InputManager::injectTouch(bool touched, int16_t x, int16_t y, unsigned long nowMs) {
    handleTouchPoint(touched, x, y, nowMs);
}

void InputManager::injectEncoderStep(int delta) {
    if (delta > 0) {
        UserEvent ev;
        ev.type = UserEventType::RotateClockwise;
        queueEvent(ev);
    } else if (delta < 0) {
        UserEvent ev;
        ev.type = UserEventType::RotateCounterClockwise;
        queueEvent(ev);
    }
}

void InputManager::injectEncoderButton(bool pressed) {
    if (pressed) {
        UserEvent ev;
        ev.type = UserEventType::EncoderButtonPressed;
        queueEvent(ev);
    }
}

int InputManager::getTargetGridCharFromTouch(int16_t x) {
    int touched_char = (x - REMSTARTX) / CHAR_W(REMSIZE);
    if (touched_char < 0) touched_char = 0;
    if (touched_char > 5) touched_char = 5;
    return touched_char;
}

bool InputManager::isLocalGridTouched(int16_t x, int16_t y) {
    return (x >= TOUCH_LOCAL_GRID_X_MIN && x <= TOUCH_LOCAL_GRID_X_MAX &&
            y >= TOUCH_LOCAL_GRID_Y_MIN && y <= TOUCH_LOCAL_GRID_Y_MAX);
}

bool InputManager::isTargetGridTouched(int16_t x, int16_t y) {
    return (x >= TOUCH_X_MIN && x <= TOUCH_X_MAX &&
            y >= TOUCH_Y_MIN && y <= TOUCH_Y_MAX);
}

bool InputManager::isCalPillTouched(int16_t x, int16_t y) {
    return (x >= CAL_TOUCH_X_MIN && x <= CAL_TOUCH_X_MAX &&
            y >= CAL_TOUCH_Y_MIN && y <= CAL_TOUCH_Y_MAX);
}

bool InputManager::isFooterTouched(int16_t x, int16_t y) {
    return (x >= TOUCH_FOOTER_X_MIN && x <= TOUCH_FOOTER_X_MAX &&
            y >= TOUCH_FOOTER_Y_MIN && y <= TOUCH_FOOTER_Y_MAX);
}

ModalButtonId InputManager::getModalButtonFromTouch(int16_t x, int16_t y) {
    if (x >= CAL_TOUCH_SNAP_X_MIN && x <= CAL_TOUCH_SNAP_X_MAX &&
        y >= CAL_TOUCH_SNAP_Y_MIN && y <= CAL_TOUCH_SNAP_Y_MAX) {
        return ModalButtonId::Snap;
    }
    if (x >= CAL_TOUCH_RESET_X_MIN && x <= CAL_TOUCH_RESET_X_MAX &&
        y >= CAL_TOUCH_RESET_Y_MIN && y <= CAL_TOUCH_RESET_Y_MAX) {
        return ModalButtonId::Reset;
    }
    if (x >= CAL_TOUCH_SAVE_X_MIN && x <= CAL_TOUCH_SAVE_X_MAX &&
        y >= CAL_TOUCH_SAVE_Y_MIN && y <= CAL_TOUCH_SAVE_Y_MAX) {
        return ModalButtonId::Save;
    }
    if (x >= CAL_TOUCH_CANCEL_X_MIN && x <= CAL_TOUCH_CANCEL_X_MAX &&
        y >= CAL_TOUCH_CANCEL_Y_MIN && y <= CAL_TOUCH_CANCEL_Y_MAX) {
        return ModalButtonId::Cancel;
    }
    return ModalButtonId::None;
}

ModalButtonId InputManager::getRoverModalButtonFromTouch(int16_t x, int16_t y) {
    if (x >= ROVER_TOUCH_DISCARD_X_MIN && x <= ROVER_TOUCH_DISCARD_X_MAX &&
        y >= ROVER_TOUCH_DISCARD_Y_MIN && y <= ROVER_TOUCH_DISCARD_Y_MAX) {
        return ModalButtonId::Discard;
    }
    if (x >= ROVER_TOUCH_CANCEL_X_MIN && x <= ROVER_TOUCH_CANCEL_X_MAX &&
        y >= ROVER_TOUCH_CANCEL_Y_MIN && y <= ROVER_TOUCH_CANCEL_Y_MAX) {
        return ModalButtonId::Cancel;
    }
    return ModalButtonId::None;
}

void InputManager::handleTouchPoint(bool touched, int16_t x, int16_t y, unsigned long nowMs) {
    if (touched) {
        if (x != 0 || y != 0) {
            if (!isTouching_) {
                if (nowMs - lastTouchReleaseTime_ >= 100) {
                    isTouching_ = true;
                    touchStartTime_ = nowMs;
                    touchStartX_ = x;
                    touchStartY_ = y;
                    longPressTriggered_ = false;
                    holdingSaveBtn_ = false;

                    if (uiMode_ == UiMode::Calibrating) {
                        ModalButtonId btn = getModalButtonFromTouch(x, y);
                        if (btn == ModalButtonId::Save) {
                            holdingSaveBtn_ = true;
                        } else if (btn != ModalButtonId::None) {
                            UserEvent ev;
                            ev.type = UserEventType::ModalButtonDown;
                            ev.buttonId = btn;
                            queueEvent(ev);
                        }
                    } else if (uiMode_ == UiMode::RoverHistory) {
                        ModalButtonId btn = getRoverModalButtonFromTouch(x, y);
                        if (btn != ModalButtonId::None) {
                            UserEvent ev;
                            ev.type = UserEventType::ModalButtonDown;
                            ev.buttonId = btn;
                            queueEvent(ev);
                        }
                    }
                }
            } else {
                // Touch held
                unsigned long holdDuration = nowMs - touchStartTime_;
                if (std::abs(x - touchStartX_) > 35 || std::abs(y - touchStartY_) > 35) {
                    if (holdingSaveBtn_) {
                        holdingSaveBtn_ = false;
                        UserEvent ev;
                        ev.type = UserEventType::HoldSaveButtonProgress;
                        ev.progress = 0.0f;
                        queueEvent(ev);
                    }
                    if (uiMode_ == UiMode::Calibrating || uiMode_ == UiMode::RoverHistory) {
                        UserEvent ev;
                        ev.type = UserEventType::ModalButtonUp;
                        queueEvent(ev);
                    }
                }

                if (uiMode_ == UiMode::Normal && isLocalGridTouched(touchStartX_, touchStartY_)) {
                    if (!longPressTriggered_ && holdDuration >= LONG_PRESS_MS) {
                        longPressTriggered_ = true;
                        UserEvent ev;
                        ev.type = UserEventType::LongPressLocalGridTriggered;
                        queueEvent(ev);
                    }
                }

                if (holdingSaveBtn_ && !longPressTriggered_) {
                    float prog = static_cast<float>(holdDuration) / static_cast<float>(LONG_PRESS_MS);
                    if (prog > 1.0f) prog = 1.0f;
                    UserEvent ev;
                    ev.type = UserEventType::HoldSaveButtonProgress;
                    ev.progress = prog;
                    queueEvent(ev);

                    if (holdDuration >= LONG_PRESS_MS) {
                        longPressTriggered_ = true;
                        holdingSaveBtn_ = false;
                        UserEvent commitEv;
                        commitEv.type = UserEventType::SaveButtonCommitted;
                        queueEvent(commitEv);
                    }
                }
            }
        }
    } else {
        // Touch released
        if (isTouching_) {
            unsigned long pressDuration = nowMs - touchStartTime_;
            isTouching_ = false;
            lastTouchReleaseTime_ = nowMs;

            if (holdingSaveBtn_) {
                holdingSaveBtn_ = false;
                UserEvent ev;
                ev.type = UserEventType::HoldSaveButtonProgress;
                ev.progress = 0.0f;
                queueEvent(ev);
            } else if (uiMode_ == UiMode::Calibrating && pressDuration >= LONG_PRESS_MS) {
                UserEvent ev;
                ev.type = UserEventType::ModalButtonUp;
                queueEvent(ev);
            }

            if (!longPressTriggered_ && pressDuration < LONG_PRESS_MS) {
                int16_t px = touchStartX_;
                int16_t py = touchStartY_;

                if (uiMode_ == UiMode::Calibrating) {
                    ModalButtonId btn = getModalButtonFromTouch(px, py);
                    if (btn == ModalButtonId::Snap) {
                        UserEvent ev;
                        ev.type = UserEventType::SnapOffsetTriggered;
                        queueEvent(ev);
                    } else if (btn == ModalButtonId::Reset) {
                        UserEvent ev;
                        ev.type = UserEventType::ResetOffsetTriggered;
                        queueEvent(ev);
                    } else if (btn == ModalButtonId::Cancel) {
                        UserEvent ev;
                        ev.type = UserEventType::CancelModalTriggered;
                        queueEvent(ev);
                    } else {
                        UserEvent ev;
                        ev.type = UserEventType::ModalButtonUp;
                        queueEvent(ev);
                    }
                } else if (uiMode_ == UiMode::RoverHistory) {
                    ModalButtonId btn = getRoverModalButtonFromTouch(px, py);
                    if (btn == ModalButtonId::Discard) {
                        UserEvent ev;
                        ev.type = UserEventType::DiscardLastStopTriggered;
                        queueEvent(ev);
                    } else if (btn == ModalButtonId::Cancel) {
                        UserEvent ev;
                        ev.type = UserEventType::CancelModalTriggered;
                        queueEvent(ev);
                    } else {
                        UserEvent ev;
                        ev.type = UserEventType::ModalButtonUp;
                        queueEvent(ev);
                    }
                } else {
                    // Normal mode short taps
                    if (isCalPillTouched(px, py)) {
                        UserEvent ev;
                        ev.type = UserEventType::TouchCalibrationPill;
                        queueEvent(ev);
                    } else if (isLocalGridTouched(px, py)) {
                        UserEvent ev;
                        ev.type = UserEventType::TouchLocalGrid;
                        queueEvent(ev);
                    } else if (isTargetGridTouched(px, py)) {
                        UserEvent ev;
                        ev.type = UserEventType::TouchGridCharacter;
                        ev.gridIndex = getTargetGridCharFromTouch(px);
                        queueEvent(ev);
                    } else if (isFooterTouched(px, py)) {
                        UserEvent ev;
                        ev.type = UserEventType::TouchVersionFooter;
                        queueEvent(ev);
                    }
                }
            }
        }
    }
}

void InputManager::update(unsigned long nowMs) {
#if defined(ARDUINO)
    if (nowMs == 0) {
        nowMs = millis();
    }

    int delta = 0;
    noInterrupts();
    if (s_hasChanged) {
        delta = s_counter;
        s_counter = 0;
        s_hasChanged = false;
    }
    interrupts();

    while (delta > 0) {
        UserEvent ev;
        ev.type = UserEventType::RotateClockwise;
        queueEvent(ev);
        delta--;
    }
    while (delta < 0) {
        UserEvent ev;
        ev.type = UserEventType::RotateCounterClockwise;
        queueEvent(ev);
        delta++;
    }

    if (pcf8574_) {
        // Encoder switch is active-low (P5 with INPUT_PULLUP)
        uint8_t swVal = pcf8574_->digitalRead(P5);
        if (swVal == LOW && !encoderSwPressed_) {
            encoderSwPressed_ = true;
            UserEvent ev;
            ev.type = UserEventType::EncoderButtonPressed;
            queueEvent(ev);
        } else if (swVal == HIGH && encoderSwPressed_) {
            encoderSwPressed_ = false;
        }
    }

    if (tsPanel_.touched()) {
        CST_TS_Point p = tsPanel_.getPoint(0);
        handleTouchPoint(true, p.x, p.y, nowMs);
    } else {
        handleTouchPoint(false, 0, 0, nowMs);
    }
#else
    (void)nowMs;
#endif
}

