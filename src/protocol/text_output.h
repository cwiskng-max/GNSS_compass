#pragma once

#include <cstdarg>
#include <cstddef>

/**
 * @brief Line and printf helpers shared by the CLI and BLE output paths.
 *
 * Each helper builds the complete text first and hands it to the sink in a single call.
 * This matters for BLE: sending "\r\n" as a separate write would cost a separate notification.
 *
 * Text that does not fit the stack buffer is formatted into a heap buffer instead, so output
 * is not truncated. If that allocation fails, println() falls back to two sink calls and
 * printFormat() delivers the truncated text followed by a visible "[truncated]" marker.
 */
namespace textout {

typedef void (*Sink)(const char *str, void *ctx);

// Delivers str followed by "\r\n".
void println(Sink sink, void *ctx, const char *str);

// Formats like vsnprintf and delivers the result.
void vprintFormat(Sink sink, void *ctx, const char *fmt, va_list args);

} // namespace textout

