#pragma once

#include "voice_qso_parser.h"
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

#if !defined(_WIN32)
#include <strings.h>
#endif

namespace cli_helpers {

inline bool equalsIgnoreCase(const char *a, const char *b) {
    if (!a || !b) return false;
#if defined(_WIN32)
    return _stricmp(a, b) == 0;
#else
    return strcasecmp(a, b) == 0;
#endif
}

inline bool startsWithIgnoreCase(const char *str, const char *prefix) {
    if (!str || !prefix) return false;
    size_t prefixLen = std::strlen(prefix);
#if defined(_WIN32)
    return _strnicmp(str, prefix, prefixLen) == 0;
#else
    return strncasecmp(str, prefix, prefixLen) == 0;
#endif
}

inline bool parseFloatToken(const char *s, float &out, const char *&nextOut) {
    if (!s) return false;
    while (*s == ' ' || *s == '\t') s++;
    if (*s == '\0') return false;
    char *endPtr = nullptr;
    float val = std::strtof(s, &endPtr);
    if (endPtr == s || !std::isfinite(val)) return false;
    if (*endPtr != '\0' && *endPtr != ' ' && *endPtr != '\t') return false;
    out = val;
    nextOut = endPtr;
    return true;
}

inline bool parseStrictFloat(const char *s, float &out) {
    const char *next = nullptr;
    if (!parseFloatToken(s, out, next)) return false;
    while (*next == ' ' || *next == '\t') next++;
    return *next == '\0';
}

inline bool parseStrictPositiveInt(const char *s, int &out) {
    if (!s) return false;
    while (*s == ' ' || *s == '\t') s++;
    if (*s == '\0' || *s == '-') return false;
    char *endPtr = nullptr;
    long val = std::strtol(s, &endPtr, 10);
    if (endPtr == s || val <= 0 || val > 1000000L) return false;
    while (*endPtr == ' ' || *endPtr == '\t') endPtr++;
    if (*endPtr != '\0') return false;
    out = static_cast<int>(val);
    return true;
}

inline bool parsePositiveIntOrVoice(const char *s, int &out) {
    if (!s) return false;
    while (*s == ' ' || *s == '\t') s++;
    std::string trimmed(s);
    while (!trimmed.empty()) {
        char c = trimmed.back();
        if (c == ' ' || c == '\t' || c == '.' || c == ',' || c == '!' || c == '?') {
            trimmed.pop_back();
        } else {
            break;
        }
    }
    if (trimmed.empty()) return false;
    if (parseStrictPositiveInt(trimmed.c_str(), out)) {
        return true;
    }
    if (std::isalpha(static_cast<unsigned char>(trimmed[0]))) {
        std::string voiceNorm = VoiceQsoParser::normalizeVoiceInput(trimmed);
        if (!voiceNorm.empty()) {
            return parseStrictPositiveInt(voiceNorm.c_str(), out);
        }
    }
    return false;
}

inline bool isSubcmdPunct(char c) {
    return (c == '.' || c == ',' || c == '!' || c == '?');
}

inline bool hasSubcommandPrefix(const char *arg, const char *cmd) {
    if (!arg || !cmd) return false;
    size_t len = std::strlen(cmd);
    if (!startsWithIgnoreCase(arg, cmd)) return false;
    char next = arg[len];
    if (next == '\0' || next == ' ' || next == '\t') return true;
    if (isSubcmdPunct(next)) {
        char after = arg[len + 1];
        return (after == '\0' || after == ' ' || after == '\t');
    }
    return false;
}

inline const char* skipSubcommandPrefix(const char *arg, size_t cmdLen) {
    const char *p = arg + cmdLen;
    while (*p == ' ' || *p == '\t' || isSubcmdPunct(*p)) p++;
    return p;
}

} // namespace cli_helpers
