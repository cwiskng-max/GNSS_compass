#include "gps_config_pipeline.h"
#include "gps_manager.h"
#include <cstdio>
#include <cstring>
#include <strings.h>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

GpsConfigPipeline::GpsConfigPipeline(GpsManager &gpsManager)
    : gpsManager_(gpsManager),
      state_(ConfigPipelineState::Idle),
      currentStep_(ConfigStep::None),
      failedStep_(ConfigStep::None),
      failureReason_(ConfigFailureReason::None),
      baselineCm_(0.0f),
      toleranceCm_(2.5f),
      verifyStream_(true),
      stepStartTime_(0),
      onProgress_(nullptr),
      progressContext_(nullptr) {
}

bool GpsConfigPipeline::start(float baselineCm, float tolCm, bool verifyStream, unsigned long nowMs) {
#if defined(ARDUINO)
    if (!nowMs) nowMs = millis();
#endif

    if (!gpsManager_.isSendAvailable()) {
        state_ = ConfigPipelineState::Failed;
        currentStep_ = ConfigStep::None;
        failedStep_ = ConfigStep::None;
        failureReason_ = ConfigFailureReason::TxNotAvailable;
        notifyProgress("[GPS CONFIG] Error: TX wire not connected. Cannot configure receiver.");
        return false;
    }

    if (state_ == ConfigPipelineState::Running) {
        return false;
    }

    baselineCm_ = baselineCm;
    toleranceCm_ = (tolCm > 0.0f) ? tolCm : 2.5f;
    verifyStream_ = verifyStream;
    state_ = ConfigPipelineState::Running;
    failureReason_ = ConfigFailureReason::None;
    failedStep_ = ConfigStep::None;
    currentStep_ = ConfigStep::ModeRoverUav;
    stepStartTime_ = nowMs;

    notifyProgress("[GPS CONFIG] Starting receiver configuration...");
    sendCurrentStepCommand();
    return true;
}

void GpsConfigPipeline::cancel() {
    if (state_ == ConfigPipelineState::Running) {
        state_ = ConfigPipelineState::Failed;
        failureReason_ = ConfigFailureReason::None;
        failedStep_ = currentStep_;
        currentStep_ = ConfigStep::None;
        notifyProgress("[GPS CONFIG] Configuration cancelled.");
    }
}

void GpsConfigPipeline::reset() {
    state_ = ConfigPipelineState::Idle;
    currentStep_ = ConfigStep::None;
    failedStep_ = ConfigStep::None;
    failureReason_ = ConfigFailureReason::None;
    stepStartTime_ = 0;
}

void GpsConfigPipeline::sendCurrentStepCommand() {
    char cmdBuf[64];
    switch (currentStep_) {
        case ConfigStep::ModeRoverUav:
            gpsManager_.sendCommand("MODE ROVER UAV");
            break;
        case ConfigStep::ConfigSignalGroup:
            gpsManager_.sendCommand("CONFIG SIGNALGROUP 3 6");
            break;
        case ConfigStep::UnlogCom1:
            gpsManager_.sendCommand("UNLOG COM1");
            break;
        case ConfigStep::KsxtOutput:
            gpsManager_.sendCommand("KSXT com1 0.2");
            break;
        case ConfigStep::HeadingFixLength:
            gpsManager_.sendCommand("CONFIG HEADING FIXLENGTH");
            break;
        case ConfigStep::HeadingLength:
            std::snprintf(cmdBuf, sizeof(cmdBuf), "CONFIG HEADING LENGTH %.1f %.1f", baselineCm_, toleranceCm_);
            gpsManager_.sendCommand(cmdBuf);
            break;
        case ConfigStep::SaveConfig:
            gpsManager_.sendCommand("SAVECONFIG");
            break;
        case ConfigStep::VerifyStream:
            // No command to transmit; waiting for first KSXT sentence
            break;
        default:
            break;
    }
}

void GpsConfigPipeline::advanceStep(unsigned long nowMs) {
    switch (currentStep_) {
        case ConfigStep::ModeRoverUav:
            currentStep_ = ConfigStep::ConfigSignalGroup;
            break;
        case ConfigStep::ConfigSignalGroup:
            currentStep_ = ConfigStep::UnlogCom1;
            break;
        case ConfigStep::UnlogCom1:
            currentStep_ = ConfigStep::KsxtOutput;
            break;
        case ConfigStep::KsxtOutput:
            currentStep_ = ConfigStep::HeadingFixLength;
            break;
        case ConfigStep::HeadingFixLength:
            if (baselineCm_ > 0.0f) {
                currentStep_ = ConfigStep::HeadingLength;
            } else {
                currentStep_ = ConfigStep::SaveConfig;
            }
            break;
        case ConfigStep::HeadingLength:
            currentStep_ = ConfigStep::SaveConfig;
            break;
        case ConfigStep::SaveConfig:
            if (verifyStream_) {
                currentStep_ = ConfigStep::VerifyStream;
                stepStartTime_ = nowMs;
                notifyProgress("[GPS CONFIG] Settings saved. Verifying KSXT 5 Hz stream...");
                return;
            } else {
                currentStep_ = ConfigStep::Done;
                state_ = ConfigPipelineState::Success;
                notifyProgress("[GPS CONFIG] Complete! Receiver configured and saved to flash.");
                return;
            }
        case ConfigStep::VerifyStream:
            currentStep_ = ConfigStep::Done;
            state_ = ConfigPipelineState::Success;
            notifyProgress("[GPS CONFIG] Complete! Receiver configured at 5 Hz KSXT.");
            return;
        default:
            return;
    }

    stepStartTime_ = nowMs;
    sendCurrentStepCommand();
}

void GpsConfigPipeline::handleResponse(const char *response, unsigned long nowMs) {
#if defined(ARDUINO)
    if (!nowMs) nowMs = millis();
#endif

    if (state_ != ConfigPipelineState::Running || currentStep_ == ConfigStep::VerifyStream) {
        return;
    }

    if (!response) return;

    const char *p = response;
    while (*p == ' ' || *p == '\t') p++;

    if (strncasecmp(p, "<ERROR", 6) == 0 || strncasecmp(p, "< ERROR", 7) == 0) {
        state_ = ConfigPipelineState::Failed;
        failureReason_ = ConfigFailureReason::CommandRejected;
        failedStep_ = currentStep_;
        notifyProgress("[GPS CONFIG] Error: Receiver rejected command. Aborting without saving flash.");
        return;
    }

    if (strncasecmp(p, "<OK", 3) == 0 || strncasecmp(p, "< OK", 4) == 0 || strncasecmp(p, "$command", 8) == 0) {
        advanceStep(nowMs);
    }
}

void GpsConfigPipeline::handleKsxtReceived() {
    if (state_ == ConfigPipelineState::Running && currentStep_ == ConfigStep::VerifyStream) {
        currentStep_ = ConfigStep::Done;
        state_ = ConfigPipelineState::Success;
        notifyProgress("[GPS CONFIG] Complete! Receiver configured at 5 Hz KSXT.");
    }
}

void GpsConfigPipeline::update(unsigned long nowMs) {
#if defined(ARDUINO)
    if (!nowMs) nowMs = millis();
#endif

    if (state_ != ConfigPipelineState::Running) {
        return;
    }

    if (currentStep_ == ConfigStep::VerifyStream) {
        if ((nowMs - stepStartTime_) >= GPS_CONFIG_VERIFY_TIMEOUT_MS) {
            state_ = ConfigPipelineState::Failed;
            failureReason_ = ConfigFailureReason::StreamVerifyTimeout;
            failedStep_ = ConfigStep::VerifyStream;
            notifyProgress("[GPS CONFIG] Error: Stream verification timed out waiting for KSXT.");
        }
    } else {
        if ((nowMs - stepStartTime_) >= GPS_CONFIG_STEP_TIMEOUT_MS) {
            state_ = ConfigPipelineState::Failed;
            failureReason_ = ConfigFailureReason::StepTimeout;
            failedStep_ = currentStep_;
            notifyProgress("[GPS CONFIG] Error: Command acknowledgment timed out. Aborting without saving flash.");
        }
    }
}

void GpsConfigPipeline::notifyProgress(const char *message) {
    if (onProgress_) {
        onProgress_(currentStep_, state_, message, progressContext_);
    }
}

const char* GpsConfigPipeline::getStateString() const {
    switch (state_) {
        case ConfigPipelineState::Idle: return "IDLE";
        case ConfigPipelineState::Running: return "RUNNING";
        case ConfigPipelineState::Success: return "SUCCESS";
        case ConfigPipelineState::Failed: return "FAILED";
        default: return "UNKNOWN";
    }
}

const char* GpsConfigPipeline::getFailureReasonString() const {
    switch (failureReason_) {
        case ConfigFailureReason::None: return "NONE";
        case ConfigFailureReason::TxNotAvailable: return "TX_NOT_AVAILABLE";
        case ConfigFailureReason::StepTimeout: return "STEP_TIMEOUT";
        case ConfigFailureReason::CommandRejected: return "COMMAND_REJECTED";
        case ConfigFailureReason::StreamVerifyTimeout: return "STREAM_VERIFY_TIMEOUT";
        default: return "UNKNOWN";
    }
}

const char* GpsConfigPipeline::stepToString(ConfigStep step) {
    switch (step) {
        case ConfigStep::None: return "NONE";
        case ConfigStep::ModeRoverUav: return "MODE ROVER UAV";
        case ConfigStep::ConfigSignalGroup: return "CONFIG SIGNALGROUP 3 6";
        case ConfigStep::UnlogCom1: return "UNLOG COM1";
        case ConfigStep::KsxtOutput: return "KSXT com1 0.2";
        case ConfigStep::HeadingFixLength: return "CONFIG HEADING FIXLENGTH";
        case ConfigStep::HeadingLength: return "CONFIG HEADING LENGTH";
        case ConfigStep::SaveConfig: return "SAVECONFIG";
        case ConfigStep::VerifyStream: return "VERIFY STREAM";
        case ConfigStep::Done: return "DONE";
        default: return "UNKNOWN";
    }
}

