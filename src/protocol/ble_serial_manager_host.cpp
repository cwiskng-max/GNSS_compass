// Simulated backend for BleSerialManager, used by the host unit tests. Notifications are
// appended to a string the tests can inspect; connection events are driven by test helpers.
#include "ble_serial_manager.h"

#if !BLE_SERIAL_HW

#include <cstdlib>
#include <cstring>

bool BleSerialManager::platformBegin(bool haveCustomName) {
    if (!haveCustomName) {
        std::strncpy(deviceName_, "GNSS-Compass-test", sizeof(deviceName_) - 1);
    }
    return true;
}

void BleSerialManager::platformUpdate(unsigned long nowMs) {
    diag_.update(nowMs, nullptr);
}

BleSerialManager::SendResult BleSerialManager::platformSend(const uint8_t *data, size_t len) {
    if (testFailSends_ > 0) {
        --testFailSends_;
        return SendResult::Retry;
    }
    testTxBuffer_.append(reinterpret_cast<const char *>(data), len);
    ++testNotifyCount_;
    if (len > testMaxNotify_) testMaxNotify_ = len;
    return SendResult::Sent;
}

void BleSerialManager::platformDisconnect() {
    handleDisconnect();
}

unsigned long BleSerialManager::platformMillis() {
    return 0;
}

uint8_t *BleSerialManager::platformAllocTx(size_t size) {
    return static_cast<uint8_t *>(std::malloc(size));
}

bool BleSerialManager::isAdvertising() const {
    return initialized_ && !connected_.load();
}

void BleSerialManager::setConnected(bool connected) {
    if (connected_.load() == connected) return;
    if (connected) {
        handleConnect(0);
    } else {
        handleDisconnect();
    }
    update(0);  // deliver the connection callback synchronously
}

void BleSerialManager::setSubscribed(bool subscribed) {
    handleSubscribe(subscribed);
}

void BleSerialManager::simulateRx(const char *data, size_t len) {
    handleRxData(data, len);
    update(0);  // dispatch RX
    flushTx();  // and send the complete reply
}

void BleSerialManager::flushTx() {
    // Each update() sends a bounded number of notifications; keep going while it makes progress
    for (int i = 0; i < 1000 && txCount_ > 0; ++i) {
        size_t before = txCount_;
        update(0);
        if (txCount_ >= before) break;  // congested or backing off
    }
}

#endif // !BLE_SERIAL_HW
