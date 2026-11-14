#include "ble_diagnostics.h"
#include "ble_platform.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#if BLE_SERIAL_HW
#include <Arduino.h>
#include <esp_bt.h>
#include <esp_gap_ble_api.h>
#endif

namespace {
// Periodic RSSI read interval while connected
constexpr unsigned long kRssiIntervalMs = 3000;
// Report main-loop gaps longer than this
constexpr unsigned long kLoopStallReportMs = 500;

uint32_t eventTimeMs() {
#if BLE_SERIAL_HW
    return millis();
#else
    return 0;
#endif
}

void defaultLog(const char *line, void *) {
#if BLE_SERIAL_HW
    Serial.print(line);
#else
    std::fputs(line, stdout);
#endif
}
} // namespace

BleDiagnostics::BleDiagnostics()
    : head_(0),
      tail_(0),
      dropped_(0),
      enabled_(false),
      lastUpdateMs_(0),
      nextRssiMs_(0),
      maxLoopGapMs_(0),
      txPowerReported_(false),
      logFn_(defaultLog),
      logCtx_(nullptr) {
    std::memset(queue_, 0, sizeof(queue_));
}

void BleDiagnostics::setEnabled(bool enabled) {
    enabled_ = enabled;
    // Restart stall tracking so the time since the last session isn't reported as a stall,
    // re-report TX power, and take an RSSI reading on the next update if connected.
    lastUpdateMs_ = 0;
    maxLoopGapMs_ = 0;
    txPowerReported_ = false;
    nextRssiMs_ = 0;
}

const char *BleDiagnostics::disconnectReasonName(uint16_t reason) {
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

void BleDiagnostics::push(EventType type, uint8_t status, uint16_t v1, uint16_t v2,
                          uint16_t v3, uint16_t v4, const uint8_t *bda) {
    uint32_t head = head_.load(std::memory_order_relaxed);
    if (head - tail_.load(std::memory_order_acquire) >= kQueueLen) {
        dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    Event &e = queue_[head % kQueueLen];
    e.ms = eventTimeMs();
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
    head_.store(head + 1, std::memory_order_release);
}

void BleDiagnostics::update(unsigned long nowMs, const uint8_t *peerBda) {
    if (enabled_) {
        // Main-loop stall detection (a stalled loop delays UI, CLI and BLE output)
        if (lastUpdateMs_ != 0) {
            unsigned long gap = nowMs - lastUpdateMs_;
            if (gap > maxLoopGapMs_) maxLoopGapMs_ = gap;
            if (gap >= kLoopStallReportMs) {
                log("[BLE %lu] main loop stalled for %lu ms\r\n", nowMs, gap);
            }
        }
        lastUpdateMs_ = nowMs;

#if BLE_SERIAL_HW
        if (!txPowerReported_) {
            txPowerReported_ = true;
            // ESP32-S3/C3 table: level index 0 = -24 dBm, 3 dB steps
            int adv = (int)esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);
            int def = (int)esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT);
            log("[BLE] TX power level: adv=%d (~%d dBm) default=%d (~%d dBm)\r\n",
                adv, -24 + 3 * adv, def, -24 + 3 * def);
        }

        // Periodic RSSI read of the connected peer; the result arrives as EVT_RSSI
        if (peerBda && (nextRssiMs_ == 0 || (long)(nowMs - nextRssiMs_) >= 0)) {
            nextRssiMs_ = nowMs + kRssiIntervalMs;
            uint8_t bda[6];
            std::memcpy(bda, peerBda, sizeof(bda));
            esp_ble_gap_read_rssi(bda);
        }
#else
        (void)peerBda;
#endif
    }

    uint32_t head = head_.load(std::memory_order_acquire);
    uint32_t tail = tail_.load(std::memory_order_relaxed);
    while (tail != head) {
        printEvent(queue_[tail % kQueueLen]);
        ++tail;
        tail_.store(tail, std::memory_order_release);
    }

    uint32_t dropped = dropped_.exchange(0, std::memory_order_relaxed);
    if (dropped) {
        log("[BLE] %u diagnostic events dropped\r\n", (unsigned)dropped);
    }
}

void BleDiagnostics::printEvent(const Event &e) {
    const uint8_t *b = e.bda;
    unsigned long ms = (unsigned long)e.ms;
    switch (e.type) {
        case EVT_CONNECT:
            log("[BLE %lu] CONNECT conn_id=%u peer=%02x:%02x:%02x:%02x:%02x:%02x "
                "interval=%.2fms latency=%u timeout=%ums\r\n",
                ms, e.v1, b[0], b[1], b[2], b[3], b[4], b[5],
                e.v2 * 1.25f, e.v3, (unsigned)e.v4 * 10u);
            break;
        case EVT_DISCONNECT:
            log("[BLE %lu] DISCONNECT conn_id=%u peer=%02x:%02x:%02x:%02x:%02x:%02x "
                "reason=0x%02x (%s)\r\n",
                ms, e.v1, b[0], b[1], b[2], b[3], b[4], b[5],
                e.v2, disconnectReasonName(e.v2));
            break;
        case EVT_MTU:
            if (enabled_) log("[BLE %lu] MTU conn_id=%u mtu=%u\r\n", ms, e.v1, e.v2);
            break;
        case EVT_CONN_PARAMS:
            if (enabled_) {
                log("[BLE %lu] CONN_PARAMS status=%u interval=%.2fms latency=%u timeout=%ums\r\n",
                    ms, e.status, e.v1 * 1.25f, e.v2, (unsigned)e.v3 * 10u);
            }
            break;
        case EVT_ADV_REQUEST:
            if (enabled_) log("[BLE %lu] advertising start requested\r\n", ms);
            break;
        case EVT_ADV_START:
            // Failures are always reported; routine successes only with diagnostics on
            if (enabled_ || e.status != 0) {
                log("[BLE %lu] ADV_START status=%u%s\r\n", ms, e.status,
                    e.status == 0 ? " (advertising)" : " (FAILED)");
            }
            break;
        case EVT_ADV_STOP:
            if (enabled_) log("[BLE %lu] ADV_STOP status=%u\r\n", ms, e.status);
            break;
        case EVT_RSSI:
            if (enabled_) {
                log("[BLE %lu] RSSI %d dBm (status=%u, max loop gap %lu ms)\r\n",
                    ms, (int)(int16_t)e.v1, e.status, maxLoopGapMs_);
            }
            maxLoopGapMs_ = 0;
            break;
        case EVT_CONGEST:
            if (enabled_) log("[BLE %lu] %s\r\n", ms, e.v1 ? "congested (TX paused)" : "congestion cleared");
            break;
        default:
            break;
    }
}

void BleDiagnostics::log(const char *fmt, ...) {
    if (!logFn_) return;
    char buf[192];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    logFn_(buf, logCtx_);
}
