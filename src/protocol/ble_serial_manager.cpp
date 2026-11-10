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

    void onConnect(BLEServer *pServer) override {
        (void)pServer;
        if (mgr_) mgr_->handleConnect(0);
    }

    void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) override {
        uint16_t connId = 0;
        if (pServer && param) {
            connId = param->connect.conn_id;
            // Request low-latency, high-throughput BLE connection parameters universally
            // accepted across Windows, Android, iOS, and macOS:
            // minInterval: 12 (15.0ms), maxInterval: 24 (30.0ms), latency: 0, timeout: 400 (4.0s)
            pServer->updateConnParams(param->connect.remote_bda, 12, 24, 0, 400);
        }
        if (mgr_) mgr_->handleConnect(connId);
    }

    void onDisconnect(BLEServer *pServer) override {
        (void)pServer;
        if (mgr_) mgr_->handleDisconnect();
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
      restartAdvertisingNeeded_(false),
      negotiatedMtu_(23),
      connId_(0),
      rxHead_(0),
      rxTail_(0),
      dataCb_(nullptr),
      dataCbContext_(nullptr),
      connCb_(nullptr),
      connCbContext_(nullptr)
#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    , pServer_(nullptr),
      pTxChar_(nullptr),
      pRxChar_(nullptr)
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

    // TX Characteristic (Notify & Read to Client)
    pTxChar_ = pService->createCharacteristic(
        BLE_NUS_TX_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
    );
    pTxChar_->addDescriptor(new BLE2902());

    // RX Characteristic (Write from Client)
    pRxChar_ = pService->createCharacteristic(
        BLE_NUS_RX_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pRxChar_->setCallbacks(new BleRxCallbackHandler(this));

    pService->start();

    BLEAdvertisementData advData;
    advData.setFlags(0x06); // General Discoverable + BR/EDR not supported
    advData.setCompleteServices(BLEUUID(BLE_NUS_SERVICE_UUID));

    BLEAdvertisementData scanResponseData;
    scanResponseData.setName(deviceName_);

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setAdvertisementData(advData);
    pAdvertising->setScanResponseData(scanResponseData);
    // Fast advertising interval: 20ms min, 40ms max for immediate discovery
    pAdvertising->setMinInterval(0x20); // 20 ms (32 * 0.625ms)
    pAdvertising->setMaxInterval(0x40); // 40 ms (64 * 0.625ms)
    BLEDevice::startAdvertising();
#endif

    initialized_ = true;
}

void BleSerialManager::update(unsigned long nowMs) {
    (void)nowMs;
#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    if (restartAdvertisingNeeded_) {
        restartAdvertisingNeeded_ = false;
        delay(100);
        if (pServer_) {
            pServer_->startAdvertising();
        } else {
            BLEDevice::startAdvertising();
        }
    }
#endif

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
    if (!connected_ || !pTxChar_) return;

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
    connId_ = connId;
    negotiatedMtu_ = 23; // Will dynamically expand to peer MTU on first write
    rxHead_ = 0;
    rxTail_ = 0;
    if (connCb_) {
        connCb_(true, connCbContext_);
    }
}

void BleSerialManager::handleDisconnect() {
    connected_ = false;
    connId_ = 0;
    negotiatedMtu_ = 23;
    restartAdvertisingNeeded_ = true;
    if (connCb_) {
        connCb_(false, connCbContext_);
    }
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
            handleConnect();
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

