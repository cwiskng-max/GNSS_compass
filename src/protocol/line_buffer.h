#pragma once

#include <cstddef>

/**
 * @brief Fixed-size line editor for a character-stream CLI transport.
 *
 * Accepts printable ASCII, handles backspace/DEL, and reports a completed line on CR or LF.
 * It does no I/O: feed() says what happened so the caller can echo (USB) or stay silent (BLE).
 */
class LineBuffer {
public:
    enum Event {
        Ignored,   // Character was dropped (non-printable, buffer full, or backspace on empty)
        Appended,  // Printable character was added
        Erased,    // Backspace/DEL removed the last character
        Cleared,   // Ctrl+C (ETX) or Ctrl+U (NAK) cancelled and cleared the line
        LineReady  // CR/LF completed a line; read it with c_str(), then clear()
    };

    static const size_t kCapacity = 256;

    LineBuffer() : len_(0), lastWasCr_(false) {
        buf_[0] = '\0';
    }

    Event feed(char c) {
        if (c == '\r') {
            lastWasCr_ = true;
            buf_[len_] = '\0';
            return LineReady;
        }
        if (c == '\n') {
            if (lastWasCr_ || len_ == 0) {
                lastWasCr_ = false;
                return Ignored;
            }
            lastWasCr_ = false;
            buf_[len_] = '\0';
            return LineReady;
        }
        lastWasCr_ = false;

        if (c == 3 || c == 21) {
            clear();
            lastWasCr_ = false;
            return Cleared;
        }

        if (c == '\b' || c == 127) {
            if (len_ == 0) return Ignored;
            buf_[--len_] = '\0';
            return Erased;
        }
        if (c < 32 || c > 126 || len_ >= kCapacity - 1) return Ignored;
        buf_[len_++] = c;
        buf_[len_] = '\0';
        return Appended;
    }

    void clear() {
        len_ = 0;
        buf_[0] = '\0';
    }

    void reset() {
        clear();
        lastWasCr_ = false;
    }

    const char *c_str() const { return buf_; }
    size_t length() const { return len_; }
    bool empty() const { return len_ == 0; }

private:
    char buf_[kCapacity];
    size_t len_;
    bool lastWasCr_;
};

