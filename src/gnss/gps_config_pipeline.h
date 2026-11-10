#pragma once

#include <cstdint>
#include <cstddef>
#include "../types.h"
#include "../config.h"

class GpsManager;

enum class ConfigPipelineState : uint8_t {
    Idle,
    Running,
    Success,
    Failed
};

enum class ConfigStep : uint8_t {
    None,
    ModeRoverUav,
    ConfigSignalGroup,
    UnlogCom1,
    KsxtOutput,
    HeadingFixLength,
    HeadingLength,
    SaveConfig,
    VerifyStream,
    Done
};

enum class ConfigFailureReason : uint8_t {
    None,
    TxNotAvailable,
    StepTimeout,
    CommandRejected,
    StreamVerifyTimeout
};

typedef void (*ConfigProgressCallback)(ConfigStep step, ConfigPipelineState state, const char *message, void *context);

class GpsConfigPipeline {
public:
    explicit GpsConfigPipeline(GpsManager &gpsManager);

    bool start(float baselineCm = 0.0f, float tolCm = 2.5f, bool verifyStream = true, unsigned long nowMs = 0);
    void cancel();
    void reset();

    void update(unsigned long nowMs = 0);
    void handleResponse(const char *response, unsigned long nowMs = 0);
    void handleKsxtReceived();

    ConfigPipelineState getState() const { return state_; }
    bool isIdle() const { return state_ == ConfigPipelineState::Idle; }
    bool isRunning() const { return state_ == ConfigPipelineState::Running; }
    bool isSuccess() const { return state_ == ConfigPipelineState::Success; }
    bool isFailed() const { return state_ == ConfigPipelineState::Failed; }
    bool isWaitingForStream() const { return state_ == ConfigPipelineState::Running && currentStep_ == ConfigStep::VerifyStream; }

    ConfigStep getCurrentStep() const { return currentStep_; }
    ConfigStep getFailedStep() const { return failedStep_; }
    ConfigFailureReason getFailureReason() const { return failureReason_; }

    const char* getStateString() const;
    const char* getFailureReasonString() const;
    static const char* stepToString(ConfigStep step);

    float getBaselineCm() const { return baselineCm_; }
    float getToleranceCm() const { return toleranceCm_; }
    unsigned long getStepStartTime() const { return stepStartTime_; }

    void setOnProgress(ConfigProgressCallback cb, void *context = nullptr) {
        onProgress_ = cb;
        progressContext_ = context;
    }

private:
    void sendCurrentStepCommand();
    void advanceStep(unsigned long nowMs);
    void notifyProgress(const char *message);

    GpsManager &gpsManager_;
    ConfigPipelineState state_;
    ConfigStep currentStep_;
    ConfigStep failedStep_;
    ConfigFailureReason failureReason_;

    float baselineCm_;
    float toleranceCm_;
    bool verifyStream_;
    unsigned long stepStartTime_;

    ConfigProgressCallback onProgress_;
    void *progressContext_;
};

