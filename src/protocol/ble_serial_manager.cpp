#include "ble_serial_manager.h"
#include <cstdio>
#include <cstring>
#include <cstdarg>

#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_mac.h>

class BleServerCallbackHandler : public BLEServerCallbacks {
public:
    explicit BleServerCallbackHandler(BleSerialManager *mgr) : mgr_(mgr) {}

    void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) override {
        (void)pServer;
        uint16_t connId = param ? param->connect.conn_id : 0;
        if (mgr_) mgr_->handleConnect(connId);
    }

    void onDisconnect(BLEServer *pServer) override {
        if (mgr_) mgr_->handleDisconnect();
        if (pServer) {
            pServer->startAdvertising();
        }
    }

private:
    BleSerialManager *mgr_;
};

class BleRxCallbackHandler : public BLECharacteristicCallbacks {
public:
    explicit BleRxCallbackHandler(BleSerialManager *mgr) : mgr_(mgr) {}

    void onWrite(BLECharacteristic *pChar) override {
        if (!mgr_ || !pChar) return;
        std::string rxVal = pChar->getValue();
        if (!rxVal.empty()) {
            mgr_->handleRxData(rxVal.data(), rxVal.length());
        }
    }

private:
    BleSerialManager *mgr_;
};
#endif

BleSerialManager::BleSerialManager()
    : initialized_(false),
      connected_(false),
      subscribed_(false),
      pendingConnCb_(false),
      restartAdvertisingNeeded_(false),
      negotiatedMtu_(23),
      connId_(0),
      restartAdvertisingAfterMs_(0),
      rxHead_(0),
      rxTail_(0),
      dataCb_(nullptr),
      dataCbContext_(nullptr),
      connCb_(nullptr),
      connCbContext_(nullptr)
#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    , pServer_(nullptr),
      pTxChar_(nullptr),
      pRxChar_(nullptr),
      p2902_(nullptr)
#endif
{
    std::memset(deviceName_, 0, sizeof(deviceName_));
    std::strncpy(deviceName_, "GNSS-Compass", sizeof(deviceName_) - 1);
}

BleSerialManager::~BleSerialManager() = default;

void BleSerialManager::begin(const char *customName) {
    if (initialized_) return;

    if (customName && customName[0] != '\0') {
        std::strncpy(deviceName_, customName, sizeof(deviceName_) - 1);
        deviceName_[sizeof(deviceName_) - 1] = '\0';
    } else {
#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
        uint8_t mac[6] = {0};
        if (esp_read_mac(mac, ESP_MAC_BT) == ESP_OK) {
            std::snprintf(deviceName_, sizeof(deviceName_), "GNSS-Compass-%02x%02x%02x", mac[3], mac[4], mac[5]);
        } else {
            std::strncpy(deviceName_, "GNSS-Compass", sizeof(deviceName_) - 1);
        }
#else
        std::strncpy(deviceName_, "GNSS-Compass-test", sizeof(deviceName_) - 1);
#endif
    }

#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    BLEDevice::init(deviceName_);
    BLEDevice::setMTU(512);

    pServer_ = BLEDevice::createServer();
    pServer_->setCallbacks(new BleServerCallbackHandler(this));

    BLEService *pService = pServer_->createService(BLE_NUS_SERVICE_UUID);

    // TX Characteristic (Notify per Nordic NUS standard - NOT readable)
    pTxChar_ = pService->createCharacteristic(
        BLE_NUS_TX_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    p2902_ = new BLE2902();
    pTxChar_->addDescriptor(p2902_);

    // RX Characteristic (Write / Write Without Response from Client)
    pRxChar_ = pService->createCharacteristic(
        BLE_NUS_RX_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pRxChar_->setCallbacks(new BleRxCallbackHandler(this));

    pService->start();

    // Main Advertisement Packet (Flags + Complete NUS 128-bit Service UUID + Preferred Conn Interval)
    BLEAdvertisementData advData;
    advData.setFlags(0x06); // General Discoverable + BR/EDR not supported
    advData.setCompleteServices(BLEUUID(BLE_NUS_SERVICE_UUID));

    // Preferred connection interval hints (15.0 ms - 30.0 ms) per BLE core spec AD type 0x12
    // 0x000C (12 * 1.25 ms = 15.0 ms) satisfies Windows 10/11 non-HID floor requirements
    // 0x0018 (24 * 1.25 ms = 30.0 ms) ensures low latency across Android, iOS, and Windows
    // Total advData payload: 3 (flags) + 18 (128-bit UUID) + 6 (conn interval) = 27 bytes (<= 31 max)
    const char connIntervalAd[] = { 0x05, 0x12, 0x0C, 0x00, 0x18, 0x00 };
    advData.addData(std::string(connIntervalAd, sizeof(connIntervalAd)));

    // Scan Response Packet (Complete Local Name)
    // Keeps name isolated in scan response to avoid exceeding the 31-byte legacy BLE advertisement limit
    BLEAdvertisementData scanResponseData;
    scanResponseData.setName(deviceName_);

    // Fast, reliable advertising configuration
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setAdvertisementData(advData);
    pAdvertising->setScanResponseData(scanResponseData);
    pAdvertising->setMinInterval(0x50); // 50 ms (80 * 0.625 ms)
    pAdvertising->setMaxInterval(0xA0); // 100 ms (160 * 0.625 ms)

    BLEDevice::startAdvertising();
#endif

    initialized_ = true;
}

void BleSerialManager::update(unsigned long nowMs) {
    (void)nowMs;
#if defined(ARDUINO)
    if (nowMs == 0) nowMs = millis();
#endif

#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    // Non-blocking advertising restart fallback when disconnected
    if (restartAdvertisingNeeded_) {
        if (restartAdvertisingAfterMs_ == 0) {
            restartAdvertisingAfterMs_ = nowMs + 100;
        } else if (nowMs >= restartAdvertisingAfterMs_) {
            restartAdvertisingNeeded_ = false;
            restartAdvertisingAfterMs_ = 0;
            if (pServer_) {
                pServer_->startAdvertising();
            } else {
                BLEDevice::startAdvertising();
            }
        }
    }
#endif

    // Dispatch connection callback on main thread (safe for SPI LCD UI drawing)
    if (pendingConnCb_) {
        pendingConnCb_ = false;
        if (connCb_) {
            connCb_(connected_, connCbContext_);
        }
    }

    // Drain incoming RX ring buffer on main thread
    if (dataCb_) {
        while (rxTail_ != rxHead_) {
            char c = rxRingBuf_[rxTail_];
            rxTail_ = (rxTail_ + 1) % sizeof(rxRingBuf_);
            dataCb_(&c, 1, dataCbContext_);
        }
    }
}

void BleSerialManager::write(const uint8_t *data, size_t length) {
    if (!data || length == 0) return;

#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    // Suppress notifications if not connected or if client has not enabled notifications on CCCD yet.
    // This prevents notification collisions during client GATT service discovery.
    if (!connected_ || !pTxChar_) return;
    if (p2902_ && !p2902_->getNotifications()) return;

    // Dynamically query negotiated peer MTU in case it was negotiated after connection
    if (pServer_) {
        uint16_t peerMtu = pServer_->getPeerMTU(connId_);
        if (peerMtu > negotiatedMtu_) {
            negotiatedMtu_ = peerMtu;
        }
    }

    // Chunk notification payload to fit safely within MTU
    size_t chunkSize = (negotiatedMtu_ > 3) ? (negotiatedMtu_ - 3) : 20;
    if (chunkSize < 20) chunkSize = 20;
    if (chunkSize > 240) chunkSize = 240; // Max BLE notification chunk size for high throughput

    size_t offset = 0;
    while (offset < length) {
        size_t toSend = (length - offset > chunkSize) ? chunkSize : (length - offset);
        pTxChar_->setValue(const_cast<uint8_t*>(data + offset), toSend);
        pTxChar_->notify();
        offset += toSend;
        if (offset < length) {
            delay(1); // Brief 1ms yield only between multi-packet bursts
        }
    }
#else
    if (connected_) {
        testTxBuffer_.append(reinterpret_cast<const char*>(data), length);
    }
#endif
}

bool BleSerialManager::isSubscribed() const {
#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    return p2902_ ? p2902_->getNotifications() : false;
#else
    return subscribed_;
#endif
}

void BleSerialManager::print(const char *str) {
    if (!str) return;
    write(reinterpret_cast<const uint8_t*>(str), std::strlen(str));
}

void BleSerialManager::println(const char *str) {
    if (!str || str[0] == '\0') {
        print("\r\n");
        return;
    }
    char buf[384];
    size_t len = std::strlen(str);
    if (len + 3 <= sizeof(buf)) {
        std::memcpy(buf, str, len);
        buf[len] = '\r';
        buf[len + 1] = '\n';
        buf[len + 2] = '\0';
        print(buf);
    } else {
        print(str);
        print("\r\n");
    }
}

void BleSerialManager::printFormat(const char *fmt, ...) {
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    print(buf);
}

void BleSerialManager::disconnect() {
#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    if (connected_ && pServer_) {
        pServer_->disconnect(connId_);
    }
#else
    if (connected_) {
        handleDisconnect();
    }
#endif
}

void BleSerialManager::handleConnect(uint16_t connId) {
    connected_ = true;
    subscribed_ = false;
    connId_ = connId;
    negotiatedMtu_ = 23;
    rxHead_ = 0;
    rxTail_ = 0;
    pendingConnCb_ = true;
    restartAdvertisingNeeded_ = false;
    restartAdvertisingAfterMs_ = 0;

#if !defined(ARDUINO) || defined(ARDUINO_TEST_BUILD)
    subscribed_ = true;
    if (connCb_) {
        connCb_(true, connCbContext_);
    }
    pendingConnCb_ = false;
#endif
}

void BleSerialManager::handleDisconnect() {
    connected_ = false;
    subscribed_ = false;
    connId_ = 0;
    negotiatedMtu_ = 23;
    pendingConnCb_ = true;

#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    restartAdvertisingNeeded_ = true;
    restartAdvertisingAfterMs_ = 0;
    if (p2902_) {
        p2902_->setNotifications(false);
    }
#else
    if (connCb_) {
        connCb_(false, connCbContext_);
    }
    pendingConnCb_ = false;
#endif
}

void BleSerialManager::handleRxData(const char *data, size_t len) {
    if (!data || len == 0) return;
    for (size_t i = 0; i < len; ++i) {
        size_t nextHead = (rxHead_ + 1) % sizeof(rxRingBuf_);
        if (nextHead != rxTail_) {
            rxRingBuf_[rxHead_] = data[i];
            rxHead_ = nextHead;
        }
    }
}

#if !defined(ARDUINO) || defined(ARDUINO_TEST_BUILD)
void BleSerialManager::setConnected(bool connected) {
    if (connected_ != connected) {
        if (connected) {
            handleConnect(0);
        } else {
            handleDisconnect();
        }
    }
}

void BleSerialManager::simulateRx(const char *data, size_t len) {
    handleRxData(data, len);
    update(0); // Synchronously drain ring buffer for unit testing
}
#endif
