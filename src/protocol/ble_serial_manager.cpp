#include "ble_serial_manager.h"
#include "text_output.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
// Largest notification payload sent. 244 = 251-byte LE data-length-extension PDU minus the
// 4-byte L2CAP and 3-byte ATT headers, so each notification fits one link-layer packet.
constexpr size_t kMaxNotifyPayload = 244;
// Smallest payload: the default ATT MTU of 23 minus the 3-byte ATT header.
constexpr size_t kMinNotifyPayload = 20;
// Notifications handed to the stack per update() call. Bounds main-loop time per call;
// the stack's own congestion signal is what actually paces throughput.
constexpr int kMaxNotifiesPerUpdate = 8;
// Wait after a rejected send before retrying.
constexpr unsigned long kSendRetryMs = 20;
// disconnect(): wait this long after the last notification so it can leave the controller.
constexpr unsigned long kDisconnectGraceMs = 150;
// disconnect(): give up waiting for the queue to drain after this long.
constexpr unsigned long kDisconnectTimeoutMs = 2000;

const char kDropNoticeFmt[] = "\r\n[BLE] %lu bytes of output dropped (send queue full)\r\n";
} // namespace

BleSerialManager::BleSerialManager() {
    std::memset(deviceName_, 0, sizeof(deviceName_));
    std::strncpy(deviceName_, "GNSS-Compass", sizeof(deviceName_) - 1);
}

BleSerialManager::~BleSerialManager() {
    std::free(txBuf_);
}

void BleSerialManager::begin(const char *customName) {
    if (initialized_) return;
    bool haveCustomName = customName && customName[0] != '\0';
    if (haveCustomName) {
        std::strncpy(deviceName_, customName, sizeof(deviceName_) - 1);
        deviceName_[sizeof(deviceName_) - 1] = '\0';
    }
    if (!txBuf_) txBuf_ = platformAllocTx(kTxQueueSize);
    initialized_ = platformBegin(haveCustomName);
}

void BleSerialManager::update(unsigned long nowMs) {
    if (nowMs == 0) nowMs = platformMillis();

    platformUpdate(nowMs);
    serviceSessionChange();
    drainRx();
    pumpTx(nowMs);
    serviceDisconnect(nowMs);
}

// ---------------------------------------------------------------------------
// Transport event hooks (Bluetooth task on hardware)
// ---------------------------------------------------------------------------

void BleSerialManager::handleConnect(uint16_t connId) {
    connId_ = connId;
    negotiatedMtu_ = kDefaultMtu;
    subscribed_ = false;
    congested_ = false;
    // Bytes before this point belong to an earlier session. Only the consumer (main loop) may
    // move rxTail_, so record the boundary and let serviceSessionChange() skip past it.
    rxSessionStart_.store(rxHead_.load(std::memory_order_relaxed), std::memory_order_release);
    connected_ = true;
    sessionGen_.fetch_add(1, std::memory_order_release);
}

void BleSerialManager::handleDisconnect() {
    connected_ = false;
    subscribed_ = false;
    congested_ = false;
    connId_ = 0;
    negotiatedMtu_ = kDefaultMtu;
    sessionGen_.fetch_add(1, std::memory_order_release);
}

void BleSerialManager::handleSubscribe(bool enabled) {
    subscribed_ = enabled && connected_.load();
}

void BleSerialManager::handleMtu(uint16_t mtu) {
    if (mtu >= kDefaultMtu) negotiatedMtu_ = mtu;
}

void BleSerialManager::handleCongestion(bool congested) {
    congested_ = congested;
}

void BleSerialManager::handleRxData(const char *data, size_t len) {
    if (!data || len == 0) return;
    uint32_t head = rxHead_.load(std::memory_order_relaxed);
    uint32_t tail = rxTail_.load(std::memory_order_acquire);
    for (size_t i = 0; i < len; ++i) {
        if (head - tail >= kRxBufSize) break;  // full: drop the rest of this write
        rxRingBuf_[head % kRxBufSize] = data[i];
        ++head;
    }
    rxHead_.store(head, std::memory_order_release);
}

// ---------------------------------------------------------------------------
// Main-loop processing
// ---------------------------------------------------------------------------

void BleSerialManager::serviceSessionChange() {
    uint32_t gen = sessionGen_.load(std::memory_order_acquire);
    if (gen == seenSessionGen_) return;
    seenSessionGen_ = gen;

    // Queued output was meant for the previous session (or written before the client
    // subscribed); a new client should not receive it.
    txClear();
    pendingDisconnect_ = false;

    // Skip RX bytes left over from the previous session
    uint32_t mark = rxSessionStart_.load(std::memory_order_acquire);
    if (static_cast<int32_t>(mark - rxTail_.load(std::memory_order_relaxed)) > 0) {
        rxTail_.store(mark, std::memory_order_release);
    }

    if (connCb_) connCb_(connected_.load(), connCbContext_);
}

void BleSerialManager::drainRx() {
    uint32_t head = rxHead_.load(std::memory_order_acquire);
    uint32_t tail = rxTail_.load(std::memory_order_relaxed);
    while (tail != head) {
        size_t slot = tail % kRxBufSize;
        size_t run = head - tail;
        if (run > kRxBufSize - slot) run = kRxBufSize - slot;  // stop at the wrap point
        if (dataCb_) dataCb_(&rxRingBuf_[slot], run, dataCbContext_);
        tail += static_cast<uint32_t>(run);
        rxTail_.store(tail, std::memory_order_release);
    }
}

size_t BleSerialManager::maxNotifyPayload() const {
    size_t mtu = negotiatedMtu_.load();
    size_t payload = (mtu > 3) ? mtu - 3 : kMinNotifyPayload;
    if (payload < kMinNotifyPayload) payload = kMinNotifyPayload;
    if (payload > kMaxNotifyPayload) payload = kMaxNotifyPayload;
    return payload;
}

void BleSerialManager::pumpTx(unsigned long nowMs) {
    if (txCount_ == 0 && txDroppedPending_ == 0) return;

    if (!connected_.load() || !subscribed_.load()) {
        txClear();
        return;
    }

    // Once the queue has drained after an overflow, tell the client output was lost
    if (txCount_ == 0 && txOverflow_) {
        txOverflow_ = false;
        char notice[80];
        std::snprintf(notice, sizeof(notice), kDropNoticeFmt, (unsigned long)txDroppedPending_);
        txDroppedPending_ = 0;
        txEnqueue(reinterpret_cast<const uint8_t *>(notice), std::strlen(notice));
    }

    if (txBackoff_) {
        if (static_cast<long>(nowMs - txRetryAtMs_) < 0) return;
        txBackoff_ = false;
    }

    uint8_t chunk[kMaxNotifyPayload];
    size_t maxPayload = maxNotifyPayload();
    for (int i = 0; i < kMaxNotifiesPerUpdate && txCount_ > 0; ++i) {
        if (congested_.load()) break;  // resumes when the stack reports congestion cleared

        size_t len = (txCount_ < maxPayload) ? txCount_ : maxPayload;
        size_t first = kTxQueueSize - txTail_;
        if (first > len) first = len;
        std::memcpy(chunk, txBuf_ + txTail_, first);
        if (len > first) std::memcpy(chunk + first, txBuf_, len - first);

        if (platformSend(chunk, len) != SendResult::Sent) {
            ++txRetries_;
            txBackoff_ = true;
            txRetryAtMs_ = nowMs + kSendRetryMs;
            break;
        }
        txTail_ = (txTail_ + len) % kTxQueueSize;
        txCount_ -= len;
        lastTxMs_ = nowMs;
    }
}

void BleSerialManager::serviceDisconnect(unsigned long nowMs) {
    if (!pendingDisconnect_) return;
    if (!connected_.load()) {
        pendingDisconnect_ = false;
        return;
    }
    if (!disconnectTimerStarted_) {
        disconnectTimerStarted_ = true;
        disconnectStartMs_ = nowMs;
    }
    bool drained = (txCount_ == 0) && (nowMs - lastTxMs_ >= kDisconnectGraceMs);
    bool timedOut = (nowMs - disconnectStartMs_) >= kDisconnectTimeoutMs;
    if (drained || timedOut) {
        pendingDisconnect_ = false;
        platformDisconnect();
    }
}

void BleSerialManager::disconnect() {
    if (!connected_.load()) return;
    pendingDisconnect_ = true;
    disconnectTimerStarted_ = false;
}

// ---------------------------------------------------------------------------
// TX queue (main loop only)
// ---------------------------------------------------------------------------

bool BleSerialManager::txEnqueue(const uint8_t *data, size_t len) {
    if (!txBuf_) {
        txBuf_ = platformAllocTx(kTxQueueSize);
        if (!txBuf_) return false;
    }
    if (len > kTxQueueSize - txCount_) return false;

    size_t head = (txTail_ + txCount_) % kTxQueueSize;
    size_t first = kTxQueueSize - head;
    if (first > len) first = len;
    std::memcpy(txBuf_ + head, data, first);
    if (len > first) std::memcpy(txBuf_, data + first, len - first);
    txCount_ += len;
    return true;
}

void BleSerialManager::txClear() {
    txTail_ = 0;
    txCount_ = 0;
    txOverflow_ = false;
    txDroppedPending_ = 0;
    txBackoff_ = false;
}

void BleSerialManager::write(const uint8_t *data, size_t length) {
    if (!data || length == 0) return;
    // Without a subscribed client there is nobody to deliver to. Dropping here also keeps
    // notifications from colliding with the client's GATT service discovery.
    if (!connected_.load() || !subscribed_.load() || pendingDisconnect_) return;

    // After an overflow, drop everything until the queue drains so the client sees one
    // contiguous gap followed by a notice, not lines missing here and there.
    if (txOverflow_ || !txEnqueue(data, length)) {
        txOverflow_ = true;
        txDroppedPending_ += static_cast<uint32_t>(length);
        txDroppedTotal_ += static_cast<uint32_t>(length);
    }
}

void BleSerialManager::print(const char *str) {
    if (!str) return;
    write(reinterpret_cast<const uint8_t *>(str), std::strlen(str));
}

void BleSerialManager::printSink(const char *str, void *ctx) {
    static_cast<BleSerialManager *>(ctx)->print(str);
}

void BleSerialManager::println(const char *str) {
    textout::println(printSink, this, str);
}

void BleSerialManager::printFormat(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    textout::vprintFormat(printSink, this, fmt, args);
    va_end(args);
}
