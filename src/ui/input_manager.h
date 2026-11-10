#pragma once

#include <cstdint>
#include <cstddef>
#include <cstdlib>
#include "../config.h"

#if defined(ARDUINO)
#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>
#include <Adafruit_CST8XX.h>
#endif

enum class UiMode : uint8_t {
    Normal = 0,
    Calibrating = 1,
    RoverHistory = 2
};

enum class ModalButtonId : uint8_t {
    None = 0,
    Snap = 1,
    Reset = 2,
    Save = 3,
    Cancel = 4,
    Discard = 5
};

enum class UserEventType : uint8_t {
    None = 0,
    RotateClockwise,
    RotateCounterClockwise,
    EncoderButtonPressed,
    TouchLocalGrid,
    LongPressLocalGridTriggered,
    TouchGridCharacter,
    TouchCalibrationPill,
    TouchVersionFooter,
    ModalButtonDown,
    ModalButtonUp,
    HoldSaveButtonProgress,
    SaveButtonCommitted,
    HoldDiscardButtonProgress,
    SnapOffsetTriggered,
    ResetOffsetTriggered,
    CancelModalTriggered,
    DiscardLastStopTriggered
};

struct UserEvent {
    UserEventType type = UserEventType::None;
    int gridIndex = 0;
    float progress = 0.0f;
    ModalButtonId buttonId = ModalButtonId::None;
};

class InputManager {
public:
    InputManager();

    void begin();
    void setUiMode(UiMode mode) { uiMode_ = mode; }
    UiMode getUiMode() const { return uiMode_; }

    void update(unsigned long nowMs = 0);
    bool pollEvent(UserEvent &outEvent);
    bool queueEvent(const UserEvent &evt);
    void clearEvents();

    // Event injection for host unit tests & simulation
    void injectTouch(bool touched, int16_t x, int16_t y, unsigned long nowMs);
    void injectEncoderStep(int delta);
    void injectEncoderButton(bool pressed);

    // Coordinate hit-testing helpers
    static int getTargetGridCharFromTouch(int16_t x);
    static bool isLocalGridTouched(int16_t x, int16_t y);
    static bool isTargetGridTouched(int16_t x, int16_t y);
    static bool isCalPillTouched(int16_t x, int16_t y);
    static bool isFooterTouched(int16_t x, int16_t y);
    static ModalButtonId getModalButtonFromTouch(int16_t x, int16_t y);
    static ModalButtonId getRoverModalButtonFromTouch(int16_t x, int16_t y);

    void handleTouchPoint(bool touched, int16_t x, int16_t y, unsigned long nowMs);

#if defined(ARDUINO)
    Adafruit_CST8XX &getTouchPanel() { return tsPanel_; }
    void setPcf8574(PCF8574 *pcf) { pcf8574_ = pcf; }
    PCF8574* getPcf8574() const { return pcf8574_; }
#endif

private:
#if defined(ARDUINO)
    static void IRAM_ATTR encoderIrq();
    static volatile int s_counter;
    static volatile int s_encState;
    static volatile int s_oldState;
    static volatile bool s_hasChanged;

    Adafruit_CST8XX tsPanel_;
    PCF8574 *pcf8574_ = nullptr;
    bool encoderSwPressed_ = false;
#endif

    UiMode uiMode_ = UiMode::Normal;

    // Touch state machine
    bool isTouching_ = false;
    unsigned long touchStartTime_ = 0;
    int16_t touchStartX_ = 0;
    int16_t touchStartY_ = 0;
    bool longPressTriggered_ = false;
    bool holdingSaveBtn_ = false;
    bool holdingDiscardBtn_ = false;
    unsigned long lastTouchReleaseTime_ = 0;

    // Event FIFO queue
    static constexpr size_t EVENT_QUEUE_CAPACITY = 16;
    UserEvent eventQueue_[EVENT_QUEUE_CAPACITY];
    size_t queueHead_ = 0;
    size_t queueTail_ = 0;
    size_t queueCount_ = 0;
};

