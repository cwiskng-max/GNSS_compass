#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>
#include <string>

#include "ble_platform.h"
#include "ble_diagnostics.h"

// Nordic UART Service (NUS) Standard 128-bit UUIDs
#define BLE_NUS_SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_NUS_RX_CHARACTERISTIC_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_NUS_TX_CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Callback type signatures
typedef void (*BleDataReceivedCallback)(const char *data, size_t len, void *context);
typedef void (*BleConnectionCallback)(bool connected, void *context);

#if BLE_SERIAL_HW
class BLEServer;
class BLECharacteristic;
class BLE2902;
#endif

/**
 * @brief Bluetooth Low Energy (BLE) Serial Manager using the Nordic UART Service (NUS).
 *
 * Provides a wireless transparent serial port for smartphones (iOS/Android) and laptops (Web Bluetooth).
 * Decouples wireless transport from command processing.
 *
 * Source layout:
 *   - ble_serial_manager.cpp:       portable core (RX/TX queues, session handling, update loop)
 *   - ble_serial_manager_esp32.cpp: Bluedroid backend (GATT service, advertising, notifications)
 *   - ble_serial_manager_host.cpp:  simulated backend used by the host unit tests
 *   - ble_diagnostics.cpp:          optional link diagnostics printed to USB serial
 *
 * Threading:
 *   - The handle*() hooks run on the Bluetooth (Bluedroid BTC) task on hardware.
 *   - Everything else, including all output calls, must run on the main loop.
 *   - State shared between the two is atomic. The RX ring is single-producer (Bluetooth task)
 *     / single-consumer (main loop). The TX queue is touched only by the main loop.
 *
 * Output:
 *   write()/print() append to a TX queue and return immediately; update() drains it as
 *   MTU-sized notifications, pausing while the stack reports congestion or a send fails.
 *   Output is discarded while no client has subscribed to notifications, and any queued
 *   output is discarded when the connection changes.
 *   If a write does not fit in the queue, it and every later write are dropped until the
 *   queue has fully drained. A single "[BLE] N bytes of output dropped" notice is then sent
 *   and the count is kept in getTxDroppedBytes().
 */
class BleSerialManager {
public:
    // Big enough for a full rover-log or target-list dump (~100 lines). Allocated from PSRAM.
    static const size_t kTxQueueSize = 12 * 1024;

    BleSerialManager();
    ~BleSerialManager();

    // Initializes BLE stack, configures NUS GATT service, and starts advertising
    void begin(const char *customName = nullptr);

    // Main-loop service: connection callbacks, RX dispatch, TX drain, advertising watchdog
    void update(unsigned long nowMs = 0);

    // Connection state & MTU
    bool isConnected() const { return connected_.load(); }
    bool isSubscribed() const { return subscribed_.load(); }
    bool isAdvertising() const;
    uint16_t getMtu() const { return negotiatedMtu_.load(); }
    const char* getDeviceName() const { return deviceName_; }

    // Output (main loop only); see class comment for queueing and drop policy
    void write(const uint8_t *data, size_t length);
    void print(const char *str);
    void println(const char *str = "");
    void printFormat(const char *fmt, ...);

    // TX queue statistics
    size_t getTxQueued() const { return txCount_; }
    size_t getTxCapacity() const { return kTxQueueSize; }
    uint32_t getTxDroppedBytes() const { return txDroppedTotal_; }
    uint32_t getTxRetries() const { return txRetries_; }

    // Ends the connection once queued output has been sent (or after a timeout)
    void disconnect();

    // Optional RF diagnostics on USB serial (see BleDiagnostics for what is always logged)
    void setDiagEnabled(bool enabled) { diag_.setEnabled(enabled); }
    bool isDiagEnabled() const { return diag_.isEnabled(); }
    BleDiagnostics &diagnostics() { return diag_; }

    // Inbound data callback hook (called from update() with contiguous runs of received bytes)
    void setDataReceivedCallback(BleDataReceivedCallback cb, void *context = nullptr) {
        dataCb_ = cb;
        dataCbContext_ = context;
    }

    // Connection lifecycle callback (for UI icon / LED updates), called from update().
    // It is state-based and intentionally lossy: if the link drops and reconnects between two
    // update() calls, it fires once with the current state.
    void setConnectionCallback(BleConnectionCallback cb, void *context = nullptr) {
        connCb_ = cb;
        connCbContext_ = context;
    }

    // Transport event hooks, called by the backend (Bluetooth task on hardware, test
    // helpers on host). Not for application code.
    void handleConnect(uint16_t connId);
    void handleDisconnect();
    void handleSubscribe(bool enabled);
    void handleMtu(uint16_t mtu);
    void handleCongestion(bool congested);
    void handleRxData(const char *data, size_t len);

#if BLE_SERIAL_HW
    // Bluedroid hooks (Bluetooth task). Do not call from application code.
    void onBtConnect(uint16_t connId, const uint8_t *bda);
    void onBtDisconnect();
    void onGapEvent(int event, const void *param);
    void onGattsEvent(int event, uint8_t gattsIf, const void *param);
#else
    // Host / unit-test simulation helpers
    void setConnected(bool connected);
    void setSubscribed(bool subscribed);
    void setMtu(uint16_t mtu) { handleMtu(mtu); }
    void setCongested(bool congested) { handleCongestion(congested); }
    void failNextSends(int count) { testFailSends_ = count; }
    void simulateRx(const char *data, size_t len);  // also flushes the reply
    void flushTx();  // runs update() until the TX queue is empty or stops draining
    const std::string& getTxBuffer() const { return testTxBuffer_; }
    void clearTxBuffer() { testTxBuffer_.clear(); }
    size_t getNotifyCount() const { return testNotifyCount_; }
    size_t getMaxNotifySize() const { return testMaxNotify_; }
#endif

private:
    enum class SendResult { Sent, Retry };

    // Backend interface (ble_serial_manager_esp32.cpp / ble_serial_manager_host.cpp)
    // Sets up the stack and advertising. Derives deviceName_ unless a custom name was given.
    bool platformBegin(bool haveCustomName);
    void platformUpdate(unsigned long nowMs);
    SendResult platformSend(const uint8_t *data, size_t len);
    void platformDisconnect();
    static unsigned long platformMillis();
    static uint8_t *platformAllocTx(size_t size);

    // Portable core
    void serviceSessionChange();
    void drainRx();
    void pumpTx(unsigned long nowMs);
    void serviceDisconnect(unsigned long nowMs);
    bool txEnqueue(const uint8_t *data, size_t len);
    void txClear();
    size_t maxNotifyPayload() const;
    static void printSink(const char *str, void *ctx);

    static const uint16_t kDefaultMtu = 23;
    static const size_t kRxBufSize = 512;  // power of two (indices are free-running counters)

    bool initialized_ = false;
    char deviceName_[32];

    // Shared with the Bluetooth task
    std::atomic<bool> connected_{false};
    std::atomic<bool> subscribed_{false};
    std::atomic<bool> congested_{false};
    std::atomic<uint16_t> negotiatedMtu_{kDefaultMtu};
    std::atomic<uint16_t> connId_{0};
    std::atomic<uint32_t> sessionGen_{0};  // bumped on every connect/disconnect

    // RX ring. Free-running counters; slot = counter % kRxBufSize.
    char rxRingBuf_[kRxBufSize];
    std::atomic<uint32_t> rxHead_{0};          // written by the Bluetooth task
    std::atomic<uint32_t> rxTail_{0};          // written by the main loop
    std::atomic<uint32_t> rxSessionStart_{0};  // rxHead_ at the last connect (Bluetooth task)

    // Main loop only
    uint32_t seenSessionGen_ = 0;
    uint8_t *txBuf_ = nullptr;
    size_t txTail_ = 0;
    size_t txCount_ = 0;
    bool txOverflow_ = false;            // dropping until the queue drains
    uint32_t txDroppedPending_ = 0;  // bytes dropped since the last notice
    uint32_t txDroppedTotal_ = 0;
    uint32_t txRetries_ = 0;
    bool txBackoff_ = false;
    unsigned long txRetryAtMs_ = 0;
    unsigned long lastTxMs_ = 0;
    bool pendingDisconnect_ = false;
    bool disconnectTimerStarted_ = false;
    unsigned long disconnectStartMs_ = 0;

    BleDataReceivedCallback dataCb_ = nullptr;
    void *dataCbContext_ = nullptr;
    BleConnectionCallback connCb_ = nullptr;
    void *connCbContext_ = nullptr;

    BleDiagnostics diag_;

#if BLE_SERIAL_HW
    void requestAdvertisingStart();

    BLEServer *pServer_ = nullptr;
    BLECharacteristic *pTxChar_ = nullptr;
    BLE2902 *p2902_ = nullptr;
    std::atomic<uint8_t> gattsIf_{0xFF};  // ESP_GATT_IF_NONE until the first GATTS event
    std::atomic<uint16_t> txHandle_{0};

    // Peer address of the current link (used for a real link-layer disconnect and RSSI reads)
    uint8_t remoteBda_[6] = {0};
    std::atomic<bool> haveRemoteBda_{false};

    // Advertising supervision: advertising_ reflects controller state as reported by GAP events.
    // update() restarts advertising if we are disconnected and the controller is not advertising.
    std::atomic<bool> advertising_{false};
    std::atomic<uint32_t> lastAdvStartRequestMs_{0};
#else
    std::string testTxBuffer_;
    int testFailSends_ = 0;
    size_t testNotifyCount_ = 0;
    size_t testMaxNotify_ = 0;
#endif
};
