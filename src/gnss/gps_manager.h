#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include "../types.h"
#include "../config.h"
#include "../geo/bearing_filter.h"
#include "simulator.h"

#if defined(ARDUINO)
#include <Arduino.h>
#include <HardwareSerial.h>
#endif

// Listener callbacks for GNSS updates
typedef void (*GpsHeadingCallback)(float headingDeg, BearingQuality quality);
typedef void (*GpsLocationCallback)(const GeoPoint &pos);
typedef void (*GpsTimeCallback)(int hours, int minutes);
typedef void (*GpsSentenceCallback)(const char *sentence);

/**
 * @brief Manages the GNSS receiver UART hardware link, streaming sentence assembly,
 *        solution decoders, and heading staleness watchdog timer.
 */
class GpsManager {
public:
    GpsManager();

    // Peripheral & stream initialization
#if defined(ARDUINO)
    void begin(unsigned long baud = GPS_BAUD, int rxPin = RXD2, int txPin = TXD2);
#else
    void begin(unsigned long baud = GPS_BAUD);
#endif

    // Periodic processing: reads UART bytes and evaluates staleness watchdog
    void update(unsigned long nowMs = 0);

    // Watchdog evaluation: sets heading to BEARING_LOST if elapsed >= HEADING_TIMEOUT_MS
    void checkStaleness(unsigned long nowMs);

    // Data ingestion primitives (feed bytes or pre-assembled sentences)
    void feedSentence(const char *sentence, unsigned long nowMs = 0);
    void feedByte(uint8_t byte, unsigned long nowMs = 0);
    void feedBytes(const uint8_t *data, size_t len, unsigned long nowMs = 0);

    // Simulator linkage
    void setSimulator(GpsSimulator *sim) { simulator_ = sim; }
    GpsSimulator* getSimulator() const { return simulator_; }

    // State getters
    bool hasHeading() const { return hasHeading_; }
    float getRawHeading() const { return (float)bearingFilter_.getRawHeading(); }
    float getHeading() const { return (float)bearingFilter_.getRawHeading(); }
    float getCalibratedHeading(float offsetDeg) const { return (float)bearingFilter_.getCalibratedHeading(offsetDeg); }
    BearingQuality getBearingQuality() const { return bearingQuality_; }
    unsigned long getLastHeadingUpdate() const { return lastHeadingUpdate_; }

    bool hasLocation() const { return location_.isValid; }
    const GeoPoint& getLocation() const { return location_; }
    void setLocation(const GeoPoint &point, unsigned long nowMs = 0);
    void setLocation(double lat, double lon, unsigned long nowMs = 0);

    bool hasTime() const { return hasTime_; }
    int getHours() const { return hours_; }
    int getMinutes() const { return minutes_; }
    void setTime(int hours, int minutes);

    void setHeading(float headingDeg, BearingQuality quality, unsigned long nowMs = 0);

    BearingFilter& getBearingFilter() { return bearingFilter_; }
    const BearingFilter& getBearingFilter() const { return bearingFilter_; }

    // Stream diagnostic statistics
    unsigned long getTotalBytesReceived() const { return totalBytesReceived_; }
    unsigned long getLastByteMillis() const { return lastByteMillis_; }
    unsigned int getGllPacketCount() const { return gllPacketCount_; }
    unsigned int getGllCount() const { return gllPacketCount_; }
    unsigned long getLastGllMillis() const { return lastGllMillis_; }
    unsigned int getKsxtPacketCount() const { return ksxtPacketCount_; }
    unsigned int getKsxtCount() const { return ksxtPacketCount_; }
    unsigned long getLastKsxtMillis() const { return lastKsxtMillis_; }
    unsigned int getHdtPacketCount() const { return hdtPacketCount_; }
    unsigned int getHdtCount() const { return hdtPacketCount_; }
    unsigned long getLastHdtMillis() const { return lastHdtMillis_; }

    // Stream verdict evaluation
    GpsLinkVerdict evaluateStreamHealth(unsigned long nowMs = 0) const;

    // Callbacks
    void setOnHeadingUpdated(GpsHeadingCallback cb) { onHeadingUpdated_ = cb; }
    void setOnLocationUpdated(GpsLocationCallback cb) { onLocationUpdated_ = cb; }
    void setOnTimeUpdated(GpsTimeCallback cb) { onTimeUpdated_ = cb; }
    void setOnRawSentence(GpsSentenceCallback cb) { onRawSentence_ = cb; }

#if defined(ARDUINO)
    HardwareSerial& getSerial() { return serialPort_; }
#endif

private:
    void processSentence(char *sentence, unsigned long nowMs);

    BearingFilter bearingFilter_;
    bool hasHeading_;
    BearingQuality bearingQuality_;
    unsigned long lastHeadingUpdate_;

    GeoPoint location_;
    bool hasTime_;
    int hours_;
    int minutes_;

    // Stream metrics
    unsigned long totalBytesReceived_;
    unsigned long lastByteMillis_;
    unsigned int gllPacketCount_;
    unsigned long lastGllMillis_;
    unsigned int ksxtPacketCount_;
    unsigned long lastKsxtMillis_;
    unsigned int hdtPacketCount_;
    unsigned long lastHdtMillis_;

    // Line assembly buffer
    static const size_t BUFFER_SIZE = 1024;
    char lineBuffer_[BUFFER_SIZE];
    size_t lineIndex_;

    // Linked simulator
    GpsSimulator *simulator_;

    // Callbacks
    GpsHeadingCallback onHeadingUpdated_;
    GpsLocationCallback onLocationUpdated_;
    GpsTimeCallback onTimeUpdated_;
    GpsSentenceCallback onRawSentence_;

#if defined(ARDUINO)
    HardwareSerial serialPort_;
#endif
};
