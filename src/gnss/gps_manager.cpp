#include "gps_manager.h"
#include "../protocol/nmea_parser.h"
#include <cmath>

GpsManager::GpsManager()
    : hasHeading_(false),
      bearingQuality_(BEARING_NONE),
      lastHeadingUpdate_(0),
      hasTime_(false),
      hours_(0),
      minutes_(0),
      totalBytesReceived_(0),
      lastByteMillis_(0),
      gllPacketCount_(0),
      lastGllMillis_(0),
      ksxtPacketCount_(0),
      lastKsxtMillis_(0),
      hdtPacketCount_(0),
      lastHdtMillis_(0),
      lineIndex_(0),
      simulator_(nullptr),
      onHeadingUpdated_(nullptr),
      onLocationUpdated_(nullptr),
      onTimeUpdated_(nullptr),
      onRawSentence_(nullptr)
#if defined(ARDUINO)
      , serialPort_(2)
#endif
{
    lineBuffer_[0] = '\0';
}

#if defined(ARDUINO)
void GpsManager::begin(unsigned long baud, int rxPin, int txPin) {
    serialPort_.begin(baud, SERIAL_8N1, rxPin, txPin);
}
#else
void GpsManager::begin(unsigned long /* baud */) {}
#endif

void GpsManager::checkStaleness(unsigned long nowMs) {
    if (hasHeading_ && bearingQuality_ != BEARING_LOST) {
        if (nowMs >= lastHeadingUpdate_ && (nowMs - lastHeadingUpdate_) >= HEADING_TIMEOUT_MS) {
            bearingQuality_ = BEARING_LOST;
            if (onHeadingUpdated_) {
                onHeadingUpdated_(getHeading(), BEARING_LOST);
            }
        }
    }
}

void GpsManager::update(unsigned long nowMs) {
#if defined(ARDUINO)
    if (!nowMs) nowMs = millis();
    checkStaleness(nowMs);

    if (serialPort_.available()) {
        if (simulator_ && simulator_->isActive()) {
            // Suppress and discard physical bytes when simulation is active
            while (serialPort_.available()) {
                serialPort_.read();
            }
            return;
        }

        while (serialPort_.available()) {
            int c = serialPort_.read();
            if (c < 0) break;
            feedByte((uint8_t)c, nowMs);
        }
    }
#else
    if (nowMs) checkStaleness(nowMs);
#endif
}

void GpsManager::feedByte(uint8_t byte, unsigned long nowMs) {
    totalBytesReceived_++;
    lastByteMillis_ = nowMs;

    if (byte == '\n' || byte == '\r') {
        if (lineIndex_ > 0) {
            lineBuffer_[lineIndex_] = '\0';
            if (onRawSentence_) {
                onRawSentence_(lineBuffer_);
            }
            processSentence(lineBuffer_, nowMs);
            lineIndex_ = 0;
        }
    } else if (lineIndex_ < BUFFER_SIZE - 1) {
        lineBuffer_[lineIndex_++] = (char)byte;
    } else {
        // Buffer overflow protection: reset index
        lineIndex_ = 0;
    }
}

void GpsManager::feedBytes(const uint8_t *data, size_t len, unsigned long nowMs) {
    if (!data) return;
    for (size_t i = 0; i < len; ++i) {
        feedByte(data[i], nowMs);
    }
}

void GpsManager::feedSentence(const char *sentence, unsigned long nowMs) {
    if (!sentence) return;
    size_t len = strlen(sentence);
    totalBytesReceived_ += len;
    lastByteMillis_ = nowMs;

    char tempBuf[BUFFER_SIZE];
    strncpy(tempBuf, sentence, BUFFER_SIZE - 1);
    tempBuf[BUFFER_SIZE - 1] = '\0';

    // Strip trailing newlines if present
    size_t tLen = strlen(tempBuf);
    while (tLen > 0 && (tempBuf[tLen - 1] == '\r' || tempBuf[tLen - 1] == '\n')) {
        tempBuf[--tLen] = '\0';
    }

    if (onRawSentence_) {
        onRawSentence_(tempBuf);
    }
    processSentence(tempBuf, nowMs);
}

void GpsManager::setLocation(const GeoPoint &point, unsigned long /* nowMs */) {
    location_ = point;
    if (onLocationUpdated_) {
        onLocationUpdated_(location_);
    }
}

void GpsManager::setLocation(double lat, double lon, unsigned long nowMs) {
    setLocation(GeoPoint(lat, lon, true), nowMs);
}

void GpsManager::setTime(int hours, int minutes) {
    hasTime_ = true;
    hours_ = hours;
    minutes_ = minutes;
    if (onTimeUpdated_) {
        onTimeUpdated_(hours_, minutes_);
    }
}

void GpsManager::setHeading(float headingDeg, BearingQuality quality, unsigned long nowMs) {
    hasHeading_ = true;
    lastHeadingUpdate_ = nowMs;
    bearingQuality_ = quality;
    bearingFilter_.seed(headingDeg);
    if (onHeadingUpdated_) {
        onHeadingUpdated_(headingDeg, quality);
    }
}

GpsLinkVerdict GpsManager::evaluateStreamHealth(unsigned long nowMs) const {
#if defined(ARDUINO)
    if (!nowMs) nowMs = millis();
#endif
    return evaluateGpsStream(
        nowMs, totalBytesReceived_, lastByteMillis_,
        gllPacketCount_, ksxtPacketCount_, hdtPacketCount_,
        hasLocation(), hasHeading_, bearingQuality_
    );
}

void GpsManager::processSentence(char *sentence, unsigned long nowMs) {
    float newHeading = 0.0f;
    BearingQuality newQuality = BEARING_LOST;

    if (parseHeadingSentence(sentence, &newHeading, &newQuality)) {
        if (strncmp(sentence, "$KSXT", 5) == 0) {
            ksxtPacketCount_++;
            lastKsxtMillis_ = nowMs;
        } else {
            hdtPacketCount_++;
            lastHdtMillis_ = nowMs;
        }
        hasHeading_ = true;
        lastHeadingUpdate_ = nowMs;
        bearingQuality_ = newQuality;
        bearingFilter_.push(newHeading);

        if (onHeadingUpdated_) {
            onHeadingUpdated_(newHeading, newQuality);
        }
    } else if (strncmp(sentence, "$KSXT", 5) == 0 ||
               strncmp(sentence, "$GNTHS", 6) == 0 || strncmp(sentence, "$GPTHS", 6) == 0 ||
               strncmp(sentence, "$GNHDT", 6) == 0 || strncmp(sentence, "$GPHDT", 6) == 0) {
        if (strncmp(sentence, "$KSXT", 5) == 0) {
            ksxtPacketCount_++;
            lastKsxtMillis_ = nowMs;
        } else {
            hdtPacketCount_++;
            lastHdtMillis_ = nowMs;
        }
        if (hasHeading_ && bearingQuality_ != BEARING_LOST) {
            bearingQuality_ = BEARING_LOST;
            if (onHeadingUpdated_) {
                onHeadingUpdated_(getHeading(), BEARING_LOST);
            }
        }
    }

    // Extract Time and Position from $KSXT (5 Hz)
    if (strncmp(sentence, "$KSXT", 5) == 0) {
        int ksxtH = 0, ksxtM = 0;
        if (parseKsxtTime(sentence, ksxtH, ksxtM)) {
            if (!hasTime_ || ksxtH != hours_ || ksxtM != minutes_) {
                setTime(ksxtH, ksxtM);
            }
        }

        GeoPoint ksxtPt;
        if (parseKsxtLocation(sentence, ksxtPt)) {
            if (!hasLocation() ||
                std::abs(ksxtPt.latitude - location_.latitude) > 0.00001 ||
                std::abs(ksxtPt.longitude - location_.longitude) > 0.00001) {
                setLocation(ksxtPt, nowMs);
            }
        }
    } else if (strncmp(sentence, "$GNGLL", 6) == 0 || strncmp(sentence, "$GPGLL", 6) == 0) {
        gllPacketCount_++;
        lastGllMillis_ = nowMs;
        char *ptr = nullptr;
        double gllLat = 0.0, gllLon = 0.0;

        parseLegacyLatLon(&sentence[7], &gllLat, &gllLon, &ptr);
        if (ptr) {
            int gllH = aton(ptr[0]) * 10 + aton(ptr[1]);
            int gllM = aton(ptr[2]) * 10 + aton(ptr[3]);

            char *statusComma = strchr(ptr, ',');
            if ((statusComma != nullptr && *(statusComma + 1) == 'A') || ptr[10] == 'A') {
                setLocation(gllLat, gllLon, nowMs);

                if (!hasTime_ || gllH != hours_ || gllM != minutes_) {
                    setTime(gllH, gllM);
                }
            }
        }
    }
}
