#pragma once

#include <string>
#include <vector>
#include <cstdint>

/**
 * @brief Result of parsing a voice-dictated QSO command.
 */
struct ParsedQso {
    bool success = false;
    std::string callsign;
    std::string band;          // e.g. "10 GHz", "24 GHz", "LIGHT"
    uint32_t freq_khz = 0;     // Frequency in kHz (e.g. 10368100), or nominal calling freq
    std::string cabrillo_band; // Cabrillo 3.0 code (e.g. "10G", "24G", "LIGHT")
    std::string adif_band;     // ADIF band code (e.g. "3cm", "1.2cm", "submm")
    std::string error_message;
};

/**
 * @brief Voice dictation normalizer and QSO parser for mobile terminal inputs.
 *
 * Implements punctuation stripping, NATO phonetics lexer, spoken number homophones,
 * frequency-to-band deduction without requiring "GHz", Maidenhead grid parsing,
 * and default 10 GHz microwave contest handling.
 */
class VoiceQsoParser {
public:
    // Strip dictation punctuation (commas, question marks, exclamation, trailing periods)
    // while strictly preserving internal decimal points in numbers (e.g. "10368.1", "5.7", "77.5").
    static std::string stripPunctuation(const std::string &input);

    // Converts NATO phonetic words (Alpha-Zulu) and spoken numbers to uppercase alphanumeric characters.
    // Handles spoken digits ("zero"-"nine", homophones "won", "to", "for", "ate", "niner"),
    // and suffixes ("stroke rover", "slash rover", "rover" -> /R; "portable" -> /P).
    static std::string normalizeVoiceInput(const std::string &input);

    // Parses a Maidenhead grid locator from plain text or phonetic dictation
    // (e.g. "CM88qk", "charlie mike 88 quebec kilo", "charlie mike eight eight quebec kilo").
    // Returns true and sets outGrid (formatted as "CM88qk" or 4/6/8-char standard) if valid.
    static bool parseGrid(const std::string &input, std::string &outGrid);

    // Identifies if token is a known band or frequency, resolving band name, freq_khz,
    // Cabrillo 3.0 code, and ADIF band code.
    static bool parseBandOrFreq(const std::string &token,
                                std::string &outBand,
                                uint32_t &outFreqKhz,
                                std::string &outCabrillo,
                                std::string &outAdif);

    // Maps a band string (e.g. "10 GHz") to its nominal calling frequency in kHz.
    static uint32_t getNominalCallingFreqKhz(const std::string &band);

    // Maps a band string to Cabrillo 3.0 code (e.g. "10 GHz" -> "10G").
    static std::string bandToCabrillo(const std::string &band);

    // Maps a band string to ADIF band identifier (e.g. "10 GHz" -> "3cm").
    static std::string bandToAdif(const std::string &band);

    // Full QSO parser: parses voice dictation strings such as "log k6mg 10", "24 k6mg",
    // "k6mg", "kilo 6 mike golf stroke rover 47", "log k6mg 10368.1", etc.
    static bool parseQso(const std::string &input,
                         ParsedQso &out,
                         const std::string &defaultBand = "10 GHz");
};

