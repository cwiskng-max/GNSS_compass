#pragma once

#include <cstdint>
#include <cstddef>
#include <string>

// Nordic UART Service (NUS) Standard 128-bit UUIDs
#define BLE_NUS_SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_NUS_RX_CHARACTERISTIC_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_NUS_TX_CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Callback type signatures
typedef void (*BleDataReceivedCallback)(const char *data, size_t len, void *context);
typedef void (*BleConnectionCallback)(bool connected, void *context);

#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
class BLEServer;
class BLECharacteristic;
class BLE2902;
#endif

/**
 * @brief Bluetooth Low Energy (BLE) Serial Manager using the Nordic UART Service (NUS).
 *
 * Provides a wireless transparent serial port for smartphones (iOS/Android) and laptops (Web Bluetooth).
 * Decouples wireless transport from command processing.
 */
class BleSerialManager {
public:
    BleSerialManager();
    ~BleSerialManager();

    // Initializes BLE stack, configures NUS GATT service, and starts advertising
    void begin(const char *customName = nullptr);

    // Non-blocking update loop (cooperative yields / watchdog upkeep)
    void update(unsigned long nowMs = 0);

    // Connection state & MTU
    bool isConnected() const { return connected_; }
    bool isSubscribed() const;
    uint16_t getMtu() const { return negotiatedMtu_; }
    const char* getDeviceName() const { return deviceName_; }

    // Stream write operations (chunked notification delivery)
    void write(const uint8_t *data, size_t length);
    void print(const char *str);
    void println(const char *str = "");
    void printFormat(const char *fmt, ...);
    void disconnect();

    // Inbound data callback hook (called when phone/laptop writes to RX characteristic)
    void setDataReceivedCallback(BleDataReceivedCallback cb, void *context = nullptr) {
        dataCb_ = cb;
        dataCbContext_ = context;
    }

    // Connection lifecycle callback (for UI icon / LED updates)
    void setConnectionCallback(BleConnectionCallback cb, void *context = nullptr) {
        connCb_ = cb;
        connCbContext_ = context;
    }

    // Internal notification hooks for BLE callbacks
    void handleConnect(uint16_t connId = 0);
    void handleDisconnect();
    void handleRxData(const char *data, size_t len);

#if !defined(ARDUINO) || defined(ARDUINO_TEST_BUILD)
    // Host / Unit-test simulation helpers
    void setConnected(bool connected);
    void setSubscribed(bool subscribed) { subscribed_ = subscribed; }
    void simulateRx(const char *data, size_t len);
    const std::string& getTxBuffer() const { return testTxBuffer_; }
    void clearTxBuffer() { testTxBuffer_.clear(); }
#endif

private:
    bool initialized_;
    bool connected_;
    bool subscribed_;
    bool pendingConnCb_;
    bool restartAdvertisingNeeded_;
    uint16_t negotiatedMtu_;
    uint16_t connId_;
    unsigned long restartAdvertisingAfterMs_;
    char deviceName_[32];

    char rxRingBuf_[512];
    volatile size_t rxHead_;
    volatile size_t rxTail_;

    BleDataReceivedCallback dataCb_;
    void *dataCbContext_;
    BleConnectionCallback connCb_;
    void *connCbContext_;

#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    BLEServer *pServer_;
    BLECharacteristic *pTxChar_;
    BLECharacteristic *pRxChar_;
    BLE2902 *p2902_;
#endif

#if !defined(ARDUINO) || defined(ARDUINO_TEST_BUILD)
    std::string testTxBuffer_;
#endif
};

