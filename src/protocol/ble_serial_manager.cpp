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
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>

namespace {
// Single instance used by the plain-function-pointer Bluedroid custom handlers
BleSerialManager *s_bleInstance = nullptr;

// Restart advertising if disconnected and the controller has not confirmed advertising
constexpr unsigned long kAdvWatchdogMs = 2000;

enum DiagType : uint8_t {
    DIAG_CONNECT = 1,
    DIAG_DISCONNECT,
    DIAG_MTU,
    DIAG_CONN_PARAMS,
    DIAG_ADV_START,
    DIAG_ADV_STOP,
    DIAG_ADV_REQUEST,
    DIAG_RSSI,
};

// Periodic RSSI read interval while connected
constexpr unsigned long kRssiIntervalMs = 3000;
// Report main-loop gaps longer than this
constexpr unsigned long kLoopStallReportMs = 500;

const char *disconnectReasonName(uint16_t reason) {
    switch (reason) {
        case 0x08: return "supervision timeout (link lost / RF)";
        case 0x13: return "remote user terminated (clean)";
        case 0x16: return "local host terminated";
        case 0x22: return "LL response timeout";
        case 0x3B: return "unacceptable conn params";
        case 0x3D: return "MIC failure";
        case 0x3E: return "connection failed to be established";
        case 0x100: return "L2CAP failure";
        default: return "other";
    }
}

void gapTrampoline(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (s_bleInstance) s_bleInstance->onGapEvent(static_cast<int>(event), param);
}

void gattsTrampoline(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    (void)gatts_if;
    if (s_bleInstance) s_bleInstance->onGattsEvent(static_cast<int>(event), param);
}
} // namespace

class BleServerCallbackHandler : public BLEServerCallbacks {
public:
    explicit BleServerCallbackHandler(BleSerialManager *mgr) : mgr_(mgr) {}

    void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) override {
        (void)pServer;
        uint16_t connId = param ? param->connect.conn_id : 0;
        if (mgr_) {
            if (param) mgr_->setRemoteAddress(param->connect.remote_bda);
            mgr_->handleConnect(connId);
        }
    }

    void onDisconnect(BLEServer *pServer) override {
        (void)pServer;
        if (mgr_) {
            mgr_->handleDisconnect();
            // Restart advertising exactly once, right away. The update() watchdog covers
            // the case where this start is rejected by the controller.
            mgr_->requestAdvertisingStart();
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
      diagEnabled_(false),
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
      pRxChar_(nullptr),
      p2902_(nullptr),
      haveRemoteBda_(false),
      advertising_(false),
      lastAdvStartRequestMs_(0),
      lastUpdateMs_(0),
      nextRssiMs_(0),
      maxLoopGapMs_(0),
      txPowerReported_(false),
      diagHead_(0),
      diagTail_(0),
      diagDropped_(0)
#endif
{
    std::memset(deviceName_, 0, sizeof(deviceName_));
    std::strncpy(deviceName_, "GNSS-Compass", sizeof(deviceName_) - 1);
#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    std::memset(remoteBda_, 0, sizeof(remoteBda_));
#endif
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
    s_bleInstance = this;
    BLEDevice::init(deviceName_);
    BLEDevice::setMTU(512);
    // Diagnostic / supervision hooks (invoked after the library's own handling)
    BLEDevice::setCustomGapHandler(gapTrampoline);
    BLEDevice::setCustomGattsHandler(gattsTrampoline);

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

    requestAdvertisingStart();
#endif

    initialized_ = true;
}

void BleSerialManager::update(unsigned long nowMs) {
    (void)nowMs;
#if defined(ARDUINO)
    if (nowMs == 0) nowMs = millis();
#endif

#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    if (diagEnabled_) {
        // Main-loop stall detection (a stalled loop delays UI, CLI and these diagnostics)
        if (lastUpdateMs_ != 0) {
            unsigned long gap = nowMs - lastUpdateMs_;
            if (gap > maxLoopGapMs_) maxLoopGapMs_ = gap;
            if (gap >= kLoopStallReportMs) {
                Serial.printf("[BLE %lu] main loop stalled for %lu ms\r\n", nowMs, gap);
            }
        }
        lastUpdateMs_ = nowMs;

        if (!txPowerReported_ && initialized_) {
            txPowerReported_ = true;
            // ESP32-S3/C3 table: level index 0 = -24 dBm, 3 dB steps
            int adv = (int)esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);
            int def = (int)esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT);
            Serial.printf("[BLE] TX power level: adv=%d (~%d dBm) default=%d (~%d dBm)\r\n",
                          adv, -24 + 3 * adv, def, -24 + 3 * def);
        }

        // Periodic RSSI read of the connected peer
        if (connected_ && haveRemoteBda_ && (long)(nowMs - nextRssiMs_) >= 0) {
            nextRssiMs_ = nowMs + kRssiIntervalMs;
            uint8_t bda[6];
            std::memcpy(bda, remoteBda_, sizeof(bda));
            esp_ble_gap_read_rssi(bda);
        }
    }

    drainDiag();

    // Advertising watchdog: if we are disconnected and the controller has not confirmed
    // that advertising is running, (re)start it. Normal path is the immediate restart in
    // onDisconnect(); this only fires if that start was lost or rejected.
    if (initialized_ && !connected_ && !advertising_ &&
        (nowMs - lastAdvStartRequestMs_) >= kAdvWatchdogMs) {
        Serial.println("[BLE] watchdog: not advertising while disconnected -> restarting advertising");
        requestAdvertisingStart();
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

void BleSerialManager::setDiagEnabled(bool enabled) {
    diagEnabled_ = enabled;
#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    // Restart stall tracking so the gap since the last diag session isn't reported as a stall,
    // re-report TX power, and take an RSSI reading right away if connected.
    lastUpdateMs_ = 0;
    maxLoopGapMs_ = 0;
    txPowerReported_ = false;
    nextRssiMs_ = millis();
#endif
}

void BleSerialManager::disconnect() {
#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    if (!connected_) return;
    // Terminate the link-layer connection directly (HCI Disconnect, reason 0x13).
    // BLEServer::disconnect() only calls esp_ble_gatts_close(), which closes the GATT
    // virtual connection and leaves the ACL link to be torn down later by an idle timer.
    if (haveRemoteBda_) {
        uint8_t bda[6];
        std::memcpy(bda, remoteBda_, sizeof(bda));
        esp_err_t rc = esp_ble_gap_disconnect(bda);
        if (diagEnabled_ || rc != ESP_OK) {
            Serial.printf("[BLE] disconnect requested (gap_disconnect rc=%d)\r\n", (int)rc);
        }
        if (rc == ESP_OK) return;
    }
    if (pServer_) {
        Serial.println("[BLE] disconnect requested (gatts_close fallback)");
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

#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    // Controller stops legacy advertising automatically when a connection is created
    advertising_ = false;
#else
    subscribed_ = true;
    if (connCb_) {
        connCb_(true, connCbContext_);
    }
    pendingConnCb_ = false;
#endif
}

void BleSerialManager::handleDisconnect() {
#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    // Stamp before clearing connected_/advertising_ so the update() watchdog (main task)
    // cannot see "disconnected + not advertising + stale timestamp" in the window before
    // onDisconnect() issues its own advertising restart.
    lastAdvStartRequestMs_ = millis();
#endif
    connected_ = false;
    subscribed_ = false;
    connId_ = 0;
    negotiatedMtu_ = 23;
    pendingConnCb_ = true;

#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
    haveRemoteBda_ = false;
    advertising_ = false;
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

#if defined(ARDUINO) && !defined(ARDUINO_TEST_BUILD)
void BleSerialManager::setRemoteAddress(const uint8_t *bda) {
    if (!bda) return;
    std::memcpy(remoteBda_, bda, sizeof(remoteBda_));
    haveRemoteBda_ = true;
}

void BleSerialManager::requestAdvertisingStart() {
    lastAdvStartRequestMs_ = millis();
    pushDiag(DIAG_ADV_REQUEST, 0);
    // Custom raw adv/scan-response data is already configured, so this only issues
    // esp_ble_gap_start_advertising(). Calling it while already advertising simply
    // re-applies the advertising parameters.
    BLEDevice::startAdvertising();
}

void BleSerialManager::onGapEvent(int event, const void *p) {
    const esp_ble_gap_cb_param_t *param = static_cast<const esp_ble_gap_cb_param_t *>(p);
    if (!param) return;
    switch (static_cast<esp_gap_ble_cb_event_t>(event)) {
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                // Ignore a late completion that lands after a connection was already made
                if (!connected_) advertising_ = true;
            }
            pushDiag(DIAG_ADV_START, (uint8_t)param->adv_start_cmpl.status);
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                advertising_ = false;
            }
            pushDiag(DIAG_ADV_STOP, (uint8_t)param->adv_stop_cmpl.status);
            break;
        case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT:
            pushDiag(DIAG_RSSI, (uint8_t)param->read_rssi_cmpl.status,
                     (uint16_t)(int16_t)param->read_rssi_cmpl.rssi);
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            pushDiag(DIAG_CONN_PARAMS, (uint8_t)param->update_conn_params.status,
                     param->update_conn_params.conn_int,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout, 0,
                     param->update_conn_params.bda);
            break;
        default:
            break;
    }
}

void BleSerialManager::onGattsEvent(int event, const void *p) {
    const esp_ble_gatts_cb_param_t *param = static_cast<const esp_ble_gatts_cb_param_t *>(p);
    if (!param) return;
    switch (static_cast<esp_gatts_cb_event_t>(event)) {
        case ESP_GATTS_CONNECT_EVT:
            pushDiag(DIAG_CONNECT, 0, param->connect.conn_id,
                     param->connect.conn_params.interval,
                     param->connect.conn_params.latency,
                     param->connect.conn_params.timeout,
                     param->connect.remote_bda);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            pushDiag(DIAG_DISCONNECT, 0, param->disconnect.conn_id,
                     (uint16_t)param->disconnect.reason, 0, 0,
                     param->disconnect.remote_bda);
            break;
        case ESP_GATTS_MTU_EVT:
            pushDiag(DIAG_MTU, 0, param->mtu.conn_id, param->mtu.mtu);
            break;
        default:
            break;
    }
}

void BleSerialManager::pushDiag(uint8_t type, uint8_t status, uint16_t v1, uint16_t v2,
                                uint16_t v3, uint16_t v4, const uint8_t *bda) {
    size_t head = diagHead_;
    size_t next = (head + 1) % kDiagQueueLen;
    if (next == diagTail_) {
        diagDropped_ = diagDropped_ + 1;
        return;
    }
    DiagEvent &e = diagQueue_[head];
    e.ms = millis();
    e.type = type;
    e.status = status;
    e.v1 = v1;
    e.v2 = v2;
    e.v3 = v3;
    e.v4 = v4;
    if (bda) {
        std::memcpy(e.bda, bda, sizeof(e.bda));
    } else {
        std::memset(e.bda, 0, sizeof(e.bda));
    }
    diagHead_ = next;
}

void BleSerialManager::drainDiag() {
    while (diagTail_ != diagHead_) {
        const DiagEvent &e = diagQueue_[diagTail_];
        const uint8_t *b = e.bda;
        switch (e.type) {
            case DIAG_CONNECT:
                // interval in 1.25 ms units, timeout in 10 ms units
                Serial.printf("[BLE %lu] CONNECT conn_id=%u peer=%02x:%02x:%02x:%02x:%02x:%02x "
                              "interval=%.2fms latency=%u timeout=%ums\r\n",
                              (unsigned long)e.ms, e.v1, b[0], b[1], b[2], b[3], b[4], b[5],
                              e.v2 * 1.25f, e.v3, (unsigned)e.v4 * 10u);
                break;
            case DIAG_DISCONNECT:
                Serial.printf("[BLE %lu] DISCONNECT conn_id=%u peer=%02x:%02x:%02x:%02x:%02x:%02x "
                              "reason=0x%02x (%s)\r\n",
                              (unsigned long)e.ms, e.v1, b[0], b[1], b[2], b[3], b[4], b[5],
                              e.v2, disconnectReasonName(e.v2));
                break;
            case DIAG_MTU:
                if (diagEnabled_) {
                    Serial.printf("[BLE %lu] MTU conn_id=%u mtu=%u\r\n", (unsigned long)e.ms, e.v1, e.v2);
                }
                break;
            case DIAG_CONN_PARAMS:
                if (diagEnabled_) {
                    Serial.printf("[BLE %lu] CONN_PARAMS status=%u interval=%.2fms latency=%u timeout=%ums\r\n",
                                  (unsigned long)e.ms, e.status, e.v1 * 1.25f, e.v2, (unsigned)e.v3 * 10u);
                }
                break;
            case DIAG_ADV_REQUEST:
                if (diagEnabled_) {
                    Serial.printf("[BLE %lu] advertising start requested\r\n", (unsigned long)e.ms);
                }
                break;
            case DIAG_ADV_START:
                // Failures are always reported; routine successes only with diagnostics on
                if (diagEnabled_ || e.status != 0) {
                    Serial.printf("[BLE %lu] ADV_START status=%u%s\r\n", (unsigned long)e.ms, e.status,
                                  e.status == 0 ? " (advertising)" : " (FAILED)");
                }
                break;
            case DIAG_RSSI:
                if (diagEnabled_) {
                    Serial.printf("[BLE %lu] RSSI %d dBm (status=%u, max loop gap %lu ms)\r\n",
                                  (unsigned long)e.ms, (int)(int16_t)e.v1, e.status, maxLoopGapMs_);
                }
                maxLoopGapMs_ = 0;
                break;
            case DIAG_ADV_STOP:
                if (diagEnabled_) {
                    Serial.printf("[BLE %lu] ADV_STOP status=%u\r\n", (unsigned long)e.ms, e.status);
                }
                break;
            default:
                break;
        }
        diagTail_ = (diagTail_ + 1) % kDiagQueueLen;
    }
    if (diagDropped_) {
        Serial.printf("[BLE] %u diagnostic events dropped\r\n", (unsigned)diagDropped_);
        diagDropped_ = 0;
    }
}
#endif

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
