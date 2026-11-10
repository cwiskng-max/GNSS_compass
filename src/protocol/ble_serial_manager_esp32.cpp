// Bluedroid (ESP32 Arduino core 2.0.x) backend for BleSerialManager.
#include "ble_serial_manager.h"

#if BLE_SERIAL_HW

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_mac.h>
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_heap_caps.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
// Bluedroid's custom handlers are plain function pointers, so they reach the manager through
// this pointer. Only one BleSerialManager may be started (enforced in platformBegin()).
BleSerialManager *s_bleInstance = nullptr;

// Restart advertising if disconnected and the controller has not confirmed advertising
constexpr unsigned long kAdvWatchdogMs = 2000;

void gapTrampoline(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (s_bleInstance) s_bleInstance->onGapEvent(static_cast<int>(event), param);
}

void gattsTrampoline(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (s_bleInstance) s_bleInstance->onGattsEvent(static_cast<int>(event), gatts_if, param);
}
} // namespace

// The callback objects below are allocated once in platformBegin() and live for the life of
// the program, like the BLE stack objects they are attached to.

class BleServerCallbackHandler : public BLEServerCallbacks {
public:
    explicit BleServerCallbackHandler(BleSerialManager *mgr) : mgr_(mgr) {}

    void onConnect(BLEServer *, esp_ble_gatts_cb_param_t *param) override {
        mgr_->onBtConnect(param ? param->connect.conn_id : 0,
                          param ? param->connect.remote_bda : nullptr);
    }

    void onDisconnect(BLEServer *) override {
        mgr_->onBtDisconnect();
    }

private:
    BleSerialManager *mgr_;
};

class BleRxCallbackHandler : public BLECharacteristicCallbacks {
public:
    explicit BleRxCallbackHandler(BleSerialManager *mgr) : mgr_(mgr) {}

    void onWrite(BLECharacteristic *pChar) override {
        if (!pChar) return;
        std::string rxVal = pChar->getValue();
        if (!rxVal.empty()) {
            mgr_->handleRxData(rxVal.data(), rxVal.length());
        }
    }

private:
    BleSerialManager *mgr_;
};

// Tracks the client enabling/disabling notifications on the TX characteristic's CCCD
class BleCccdCallbackHandler : public BLEDescriptorCallbacks {
public:
    explicit BleCccdCallbackHandler(BleSerialManager *mgr) : mgr_(mgr) {}

    void onWrite(BLEDescriptor *pDesc) override {
        if (!pDesc) return;
        mgr_->handleSubscribe(static_cast<BLE2902 *>(pDesc)->getNotifications());
    }

private:
    BleSerialManager *mgr_;
};

bool BleSerialManager::platformBegin(bool haveCustomName) {
    if (s_bleInstance && s_bleInstance != this) {
        Serial.println("[BLE] ERROR: a BleSerialManager is already running; only one is supported");
        return false;
    }

    if (!haveCustomName) {
        uint8_t mac[6] = {0};
        if (esp_read_mac(mac, ESP_MAC_BT) == ESP_OK) {
            std::snprintf(deviceName_, sizeof(deviceName_), "GNSS-Compass-%02x%02x%02x", mac[3], mac[4], mac[5]);
        }
    }

    s_bleInstance = this;
    BLEDevice::init(deviceName_);
    // Largest MTU we accept. The client decides whether to start an MTU exchange; the
    // result arrives as ESP_GATTS_MTU_EVT.
    BLEDevice::setMTU(512);
    // Supervision / diagnostic hooks (invoked after the library's own handling)
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
    p2902_->setCallbacks(new BleCccdCallbackHandler(this));
    pTxChar_->addDescriptor(p2902_);

    // RX Characteristic (Write / Write Without Response from Client)
    BLECharacteristic *pRxChar = pService->createCharacteristic(
        BLE_NUS_RX_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pRxChar->setCallbacks(new BleRxCallbackHandler(this));

    pService->start();
    // Attribute handles are assigned while the service is created/started
    txHandle_ = pTxChar_->getHandle();

    // Advertising packet: flags + complete NUS 128-bit service UUID + preferred conn interval
    BLEAdvertisementData advData;
    advData.setFlags(0x06); // General Discoverable + BR/EDR not supported
    advData.setCompleteServices(BLEUUID(BLE_NUS_SERVICE_UUID));

    // Slave Connection Interval Range (AD type 0x12): 15 ms (0x000C) to 30 ms (0x0018), in
    // 1.25 ms units. This is only a hint; centrals are free to ignore it. The range was chosen
    // as a low-latency interval that common phone and PC stacks accept.
    // Payload: 3 (flags) + 18 (128-bit UUID) + 6 (conn interval) = 27 of 31 bytes.
    const char connIntervalAd[] = { 0x05, 0x12, 0x0C, 0x00, 0x18, 0x00 };
    advData.addData(std::string(connIntervalAd, sizeof(connIntervalAd)));

    // Scan response: complete local name (it does not fit in the 31-byte advertising packet)
    BLEAdvertisementData scanResponseData;
    scanResponseData.setName(deviceName_);

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setAdvertisementData(advData);
    pAdvertising->setScanResponseData(scanResponseData);
    pAdvertising->setMinInterval(0x50); // 50 ms (80 * 0.625 ms)
    pAdvertising->setMaxInterval(0xA0); // 100 ms (160 * 0.625 ms)

    requestAdvertisingStart();
    return true;
}

void BleSerialManager::platformUpdate(unsigned long nowMs) {
    const uint8_t *peer = nullptr;
    uint8_t bda[6];
    if (connected_.load() && haveRemoteBda_.load(std::memory_order_acquire)) {
        std::memcpy(bda, remoteBda_, sizeof(bda));
        peer = bda;
    }
    diag_.update(nowMs, peer);

    // Advertising watchdog: if we are disconnected and the controller has not confirmed
    // that advertising is running, (re)start it. Normal path is the immediate restart in
    // onBtDisconnect(); this only fires if that start was lost or rejected.
    if (initialized_ && !connected_.load() && !advertising_.load() &&
        (static_cast<uint32_t>(nowMs) - lastAdvStartRequestMs_.load()) >= kAdvWatchdogMs) {
        Serial.println("[BLE] watchdog: not advertising while disconnected -> restarting advertising");
        requestAdvertisingStart();
    }
}

BleSerialManager::SendResult BleSerialManager::platformSend(const uint8_t *data, size_t len) {
    uint8_t gattsIf = gattsIf_.load();
    uint16_t handle = txHandle_.load();
    if (gattsIf == ESP_GATT_IF_NONE || handle == 0) return SendResult::Retry;
    // Bluedroid copies the payload before returning. It returns an error when the L2CAP
    // buffers are exhausted; the caller backs off and retries the same chunk.
    esp_err_t rc = esp_ble_gatts_send_indicate(gattsIf, connId_.load(), handle,
                                               static_cast<uint16_t>(len),
                                               const_cast<uint8_t *>(data), false);
    return rc == ESP_OK ? SendResult::Sent : SendResult::Retry;
}

void BleSerialManager::platformDisconnect() {
    // Terminate the link-layer connection directly (HCI Disconnect, reason 0x13).
    // BLEServer::disconnect() only calls esp_ble_gatts_close(), which closes the GATT
    // virtual connection and leaves the ACL link to be torn down later by an idle timer.
    if (haveRemoteBda_.load(std::memory_order_acquire)) {
        uint8_t bda[6];
        std::memcpy(bda, remoteBda_, sizeof(bda));
        esp_err_t rc = esp_ble_gap_disconnect(bda);
        if (diag_.isEnabled() || rc != ESP_OK) {
            Serial.printf("[BLE] disconnect requested (gap_disconnect rc=%d)\r\n", (int)rc);
        }
        if (rc == ESP_OK) return;
    }
    if (pServer_) {
        Serial.println("[BLE] disconnect requested (gatts_close fallback)");
        pServer_->disconnect(connId_.load());
    }
}

unsigned long BleSerialManager::platformMillis() {
    return millis();
}

uint8_t *BleSerialManager::platformAllocTx(size_t size) {
    void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) p = std::malloc(size);  // no PSRAM: fall back to internal RAM
    return static_cast<uint8_t *>(p);
}

bool BleSerialManager::isAdvertising() const {
    return advertising_.load();
}

// ---------------------------------------------------------------------------
// Bluetooth-task hooks
// ---------------------------------------------------------------------------

void BleSerialManager::onBtConnect(uint16_t connId, const uint8_t *bda) {
    if (bda) {
        std::memcpy(remoteBda_, bda, sizeof(remoteBda_));
        haveRemoteBda_.store(true, std::memory_order_release);
    }
    // The controller stops legacy advertising automatically when a connection is created
    advertising_ = false;
    handleConnect(connId);
}

void BleSerialManager::onBtDisconnect() {
    // Stamp before clearing connected_/advertising_ so the update() watchdog (main task)
    // cannot see "disconnected + not advertising + stale timestamp" in the window before
    // the advertising restart below.
    lastAdvStartRequestMs_ = millis();
    haveRemoteBda_ = false;
    advertising_ = false;
    handleDisconnect();
    // The library keeps the CCCD value across connections; the next client must subscribe again
    if (p2902_) p2902_->setNotifications(false);
    // Restart advertising exactly once, right away. The update() watchdog covers
    // the case where this start is rejected by the controller.
    requestAdvertisingStart();
}

void BleSerialManager::requestAdvertisingStart() {
    lastAdvStartRequestMs_ = millis();
    diag_.push(BleDiagnostics::EVT_ADV_REQUEST, 0);
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
                if (!connected_.load()) advertising_ = true;
            }
            diag_.push(BleDiagnostics::EVT_ADV_START, (uint8_t)param->adv_start_cmpl.status);
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                advertising_ = false;
            }
            diag_.push(BleDiagnostics::EVT_ADV_STOP, (uint8_t)param->adv_stop_cmpl.status);
            break;
        case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT:
            diag_.push(BleDiagnostics::EVT_RSSI, (uint8_t)param->read_rssi_cmpl.status,
                       (uint16_t)(int16_t)param->read_rssi_cmpl.rssi);
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            diag_.push(BleDiagnostics::EVT_CONN_PARAMS, (uint8_t)param->update_conn_params.status,
                       param->update_conn_params.conn_int,
                       param->update_conn_params.latency,
                       param->update_conn_params.timeout, 0,
                       param->update_conn_params.bda);
            break;
        default:
            break;
    }
}

void BleSerialManager::onGattsEvent(int event, uint8_t gattsIf, const void *p) {
    if (gattsIf != ESP_GATT_IF_NONE) gattsIf_ = gattsIf;
    const esp_ble_gatts_cb_param_t *param = static_cast<const esp_ble_gatts_cb_param_t *>(p);
    if (!param) return;
    switch (static_cast<esp_gatts_cb_event_t>(event)) {
        case ESP_GATTS_CONNECT_EVT:
            diag_.push(BleDiagnostics::EVT_CONNECT, 0, param->connect.conn_id,
                       param->connect.conn_params.interval,
                       param->connect.conn_params.latency,
                       param->connect.conn_params.timeout,
                       param->connect.remote_bda);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            diag_.push(BleDiagnostics::EVT_DISCONNECT, 0, param->disconnect.conn_id,
                       (uint16_t)param->disconnect.reason, 0, 0,
                       param->disconnect.remote_bda);
            break;
        case ESP_GATTS_MTU_EVT:
            handleMtu(param->mtu.mtu);
            diag_.push(BleDiagnostics::EVT_MTU, 0, param->mtu.conn_id, param->mtu.mtu);
            break;
        case ESP_GATTS_CONGEST_EVT:
            handleCongestion(param->congest.congested);
            diag_.push(BleDiagnostics::EVT_CONGEST, 0, param->congest.congested ? 1 : 0);
            break;
        default:
            break;
    }
}

#endif // BLE_SERIAL_HW
