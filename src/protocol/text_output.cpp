#include "text_output.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace textout {

namespace {
// Covers almost all CLI lines without touching the heap.
constexpr size_t kStackBufLen = 256;
const char kTruncatedMarker[] = " [truncated]\r\n";
} // namespace

void println(Sink sink, void *ctx, const char *str) {
    if (!sink) return;
    if (!str || str[0] == '\0') {
        sink("\r\n", ctx);
        return;
    }

    size_t len = std::strlen(str);
    char stackBuf[kStackBufLen];
    char *buf = stackBuf;
    if (len + 3 > sizeof(stackBuf)) {
        buf = static_cast<char *>(std::malloc(len + 3));
        if (!buf) {
            sink(str, ctx);
            sink("\r\n", ctx);
            return;
        }
    }
    std::memcpy(buf, str, len);
    buf[len] = '\r';
    buf[len + 1] = '\n';
    buf[len + 2] = '\0';
    sink(buf, ctx);
    if (buf != stackBuf) std::free(buf);
}

void vprintFormat(Sink sink, void *ctx, const char *fmt, va_list args) {
    if (!sink || !fmt) return;

    char stackBuf[kStackBufLen];
    va_list retry;
    va_copy(retry, args);
    int needed = std::vsnprintf(stackBuf, sizeof(stackBuf), fmt, args);
    if (needed < 0) {
        va_end(retry);
        return;
    }
    if (static_cast<size_t>(needed) < sizeof(stackBuf)) {
        va_end(retry);
        sink(stackBuf, ctx);
        return;
    }

    char *heapBuf = static_cast<char *>(std::malloc(static_cast<size_t>(needed) + 1));
    if (heapBuf) {
        std::vsnprintf(heapBuf, static_cast<size_t>(needed) + 1, fmt, retry);
        va_end(retry);
        sink(heapBuf, ctx);
        std::free(heapBuf);
        return;
    }
    va_end(retry);
    sink(stackBuf, ctx);
    sink(kTruncatedMarker, ctx);
}

} // namespace textout

