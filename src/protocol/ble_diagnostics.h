#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

/**
 * @brief BLE link diagnostics, printed to USB serial.
 *
 * Bluetooth-task code records events with push(), which never blocks. The main loop prints
 * them from update(), so slow USB-CDC writes never stall the Bluetooth stack.
 *
 * Logging policy:
 *   - Always printed: connect, disconnect with reason code, and advertising start failures.
 *   - Only when enabled (`ble diag on`): RSSI every 3 s, main-loop stall reports, TX power,
 *     MTU, connection-parameter changes, congestion changes, and routine advertising events.
 */
class BleDiagnostics {
public:
    enum EventType : uint8_t {
        EVT_CONNECT = 1,  // v1=conn_id v2=interval(1.25ms) v3=latency v4=timeout(10ms), bda
        EVT_DISCONNECT,   // v1=conn_id v2=reason, bda
        EVT_MTU,          // v1=conn_id v2=mtu
        EVT_CONN_PARAMS,  // v1=interval(1.25ms) v2=latency v3=timeout(10ms)
        EVT_ADV_START,    // status = controller status
        EVT_ADV_STOP,     // status = controller status
        EVT_ADV_REQUEST,
        EVT_RSSI,         // v1=(int16)rssi
        EVT_CONGEST,      // v1=1 congested, 0 cleared
    };

    // Output sink for printed lines. Defaults to USB serial (stdout on host builds).
    typedef void (*LogFn)(const char *line, void *ctx);

    BleDiagnostics();

    void setEnabled(bool enabled);
    bool isEnabled() const { return enabled_; }

    void setLogSink(LogFn fn, void *ctx) { logFn_ = fn; logCtx_ = ctx; }

    // Bluetooth task. Records an event; counts and drops it if the queue is full.
    void push(EventType type, uint8_t status, uint16_t v1 = 0, uint16_t v2 = 0,
              uint16_t v3 = 0, uint16_t v4 = 0, const uint8_t *bda = nullptr);

    // Main loop. Runs stall detection and RSSI polling (when enabled) and prints queued events.
    // peerBda is the connected peer's address, or nullptr when there is none.
    void update(unsigned long nowMs, const uint8_t *peerBda);

    static const char *disconnectReasonName(uint16_t reason);

private:
    struct Event {
        uint32_t ms;
        uint8_t type;
        uint8_t status;
        uint16_t v1, v2, v3, v4;
        uint8_t bda[6];
    };

    void log(const char *fmt, ...);
    void printEvent(const Event &e);

    static const size_t kQueueLen = 24;
    Event queue_[kQueueLen];
    std::atomic<uint32_t> head_;     // written by the Bluetooth task
    std::atomic<uint32_t> tail_;     // written by the main loop
    std::atomic<uint32_t> dropped_;

    bool enabled_;
    unsigned long lastUpdateMs_;
    unsigned long nextRssiMs_;
    unsigned long maxLoopGapMs_;
    bool txPowerReported_;

    LogFn logFn_;
    void *logCtx_;
};

