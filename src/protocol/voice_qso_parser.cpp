#include "voice_qso_parser.h"
#include "../geo/maidenhead.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <sstream>

namespace {

std::string toLower(const std::string &str) {
    std::string s = str;
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return std::tolower(c);
    });
    return s;
}

// Convert a single NATO phonetic word to its corresponding uppercase character.
// Returns '\0' if not a NATO word.
char natoToChar(const std::string &w) {
    if (w == "alfa" || w == "alpha" || w == "able" || w == "america") return 'A';
    if (w == "bravo" || w == "beta" || w == "baker" || w == "boston") return 'B';
    if (w == "charlie" || w == "canada" || w == "columbia") return 'C';
    if (w == "delta" || w == "david" || w == "dog") return 'D';
    if (w == "echo" || w == "easy" || w == "edward") return 'E';
    if (w == "foxtrot" || w == "fox" || w == "frank") return 'F';
    if (w == "golf" || w == "george") return 'G';
    if (w == "hotel" || w == "henry") return 'H';
    if (w == "india" || w == "ida" || w == "italy") return 'I';
    if (w == "juliet" || w == "juliett" || w == "john" || w == "japan") return 'J';
    if (w == "kilo" || w == "king") return 'K';
    if (w == "lima" || w == "london" || w == "lewis") return 'L';
    if (w == "mike" || w == "mic" || w == "mary" || w == "mexico") return 'M';
    if (w == "november" || w == "nancy") return 'N';
    if (w == "oscar" || w == "ocean") return 'O';
    if (w == "papa" || w == "peter") return 'P';
    if (w == "quebec" || w == "queen") return 'Q';
    if (w == "romeo" || w == "roger" || w == "robert" || w == "radio") return 'R';
    if (w == "sierra" || w == "sugar" || w == "santiago") return 'S';
    if (w == "tango" || w == "tom" || w == "texas") return 'T';
    if (w == "uniform" || w == "uncle" || w == "united") return 'U';
    if (w == "victor" || w == "vic") return 'V';
    if (w == "whiskey" || w == "william" || w == "washington") return 'W';
    if (w == "xray" || w == "x-ray") return 'X';
    if (w == "yankee" || w == "young") return 'Y';
    if (w == "zulu" || w == "zebra" || w == "zanzibar") return 'Z';
    return '\0';
}

// Convert spoken digit / homophone to numeric character '0'-'9'.
// Returns '\0' if not a spoken digit.
char spokenDigitToChar(const std::string &w) {
    if (w == "zero") return '0';
    if (w == "one" || w == "won") return '1';
    if (w == "two" || w == "to" || w == "too") return '2';
    if (w == "three") return '3';
    if (w == "four" || w == "for") return '4';
    if (w == "five") return '5';
    if (w == "six") return '6';
    if (w == "seven") return '7';
    if (w == "eight" || w == "ate") return '8';
    if (w == "nine" || w == "niner") return '9';
    return '\0';
}

char spokenTensToDigit(const std::string &w) {
    if (w == "twenty") return '2';
    if (w == "thirty") return '3';
    if (w == "forty") return '4';
    if (w == "fifty") return '5';
    if (w == "sixty") return '6';
    if (w == "seventy") return '7';
    if (w == "eighty") return '8';
    if (w == "ninety") return '9';
    return '\0';
}

const char* spokenTeenToString(const std::string &w) {
    if (w == "ten") return "10";
    if (w == "eleven") return "11";
    if (w == "twelve") return "12";
    if (w == "thirteen") return "13";
    if (w == "fourteen") return "14";
    if (w == "fifteen") return "15";
    if (w == "sixteen") return "16";
    if (w == "seventeen") return "17";
    if (w == "eighteen") return "18";
    if (w == "nineteen") return "19";
    return nullptr;
}

// Appends a single phonetic/numeric/alphanumeric token from words[i] (advancing i on 2-word tens).
void appendPhoneticToken(const std::vector<std::string> &words, size_t &i,
                         std::string &out, bool allowSlashes) {
    std::string low = toLower(words[i]);

    char nc = natoToChar(low);
    if (nc != '\0') {
        out.push_back(nc);
        return;
    }

    char tenDigit = spokenTensToDigit(low);
    if (tenDigit != '\0') {
        out.push_back(tenDigit);
        if (i + 1 < words.size()) {
            std::string nextLow = toLower(words[i + 1]);
            char unitDigit = spokenDigitToChar(nextLow);
            if (unitDigit != '\0' && unitDigit != '0') {
                out.push_back(unitDigit);
                i++;
                return;
            } else if (words[i + 1].size() == 1 && words[i + 1][0] >= '1' && words[i + 1][0] <= '9') {
                out.push_back(words[i + 1][0]);
                i++;
                return;
            }
        }
        out.push_back('0');
        return;
    }

    const char *teenStr = spokenTeenToString(low);
    if (teenStr != nullptr) {
        out.append(teenStr);
        return;
    }

    char sc = spokenDigitToChar(low);
    if (sc != '\0') {
        out.push_back(sc);
        return;
    }

    for (char ch : words[i]) {
        if (isalnum((unsigned char)ch) || (allowSlashes && ch == '/')) {
            out.push_back(std::toupper((unsigned char)ch));
        }
    }
}

bool looksLikeFrequencyToken(const std::string &word) {
    if (word.empty()) return false;
    std::string s;
    s.reserve(word.size());
    for (char c : word) {
        s.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    if (s.size() > 3 && (s.substr(s.size() - 3) == "mhz" || s.substr(s.size() - 3) == "ghz")) {
        s.resize(s.size() - 3);
    } else if (s.size() > 1 && (s.back() == 'm' || s.back() == 'g')) {
        s.pop_back();
    }
    if (s.empty()) return false;
    size_t digits = 0;
    size_t dots = 0;
    for (char c : s) {
        if (std::isdigit(static_cast<unsigned char>(c))) {
            digits++;
        } else if (c == '.') {
            dots++;
        } else {
            return false;
        }
    }
    return digits > 0 && dots <= 1 && (dots == 1 || digits >= 3);
}

struct BandSpec {
    const char *matchSub;   // Substring matched in lowercased band name (order matters: 241 before 24)
    const char *bandName;
    const char *cabrillo;
    const char *adif;
    uint32_t nominalFreqKhz;
};

const BandSpec kBandSpecs[] = {
    {"241",  "241 GHz", "241G",  "1mm",   241056100},
    {"134",  "134 GHz", "134G",  "2mm",   134928100},
    {"122",  "122 GHz", "122G",  "2.5mm", 122250100},
    {"76",   "76 GHz",  "76G",   "4mm",   76032100},
    {"77.5", "76 GHz",  "76G",   "4mm",   76032100},
    {"47",   "47 GHz",  "47G",   "6mm",   47088100},
    {"24",   "24 GHz",  "24G",   "1.2cm", 24192100},
    {"10",   "10 GHz",  "10G",   "3cm",   10368100},
    {"light","LIGHT",   "LIGHT", "submm", 0},
    {"laser","LIGHT",   "LIGHT", "submm", 0},
    {"5.7",  "5.7 GHz", "5.7G",  "6cm",   5760100},
    {"3.4",  "3.4 GHz", "3.4G",  "9cm",   3456100},
    {"2.3",  "2.3 GHz", "2.3G",  "13cm",  2304100},
    {"2.4",  "2.3 GHz", "2.3G",  "13cm",  2304100},
    {"1.2",  "1.2 GHz", "1.2G",  "23cm",  1296100},
    {"902",  "902 MHz", "902",   "33cm",  902100},
    {"432",  "432 MHz", "432",   "70cm",  432100},
    {"222",  "222 MHz", "222",   "1.25m", 222100},
    {"144",  "144 MHz", "144",   "2m",    144200},
    {"50",   "50 MHz",  "50",    "6m",    50100}
};

const BandSpec* findBandSpec(const std::string &band) {
    std::string b = toLower(band);
    for (const auto &spec : kBandSpecs) {
        if (b.find(spec.matchSub) != std::string::npos) {
            return &spec;
        }
    }
    return nullptr;
}

} // anonymous namespace

std::string VoiceQsoParser::stripPunctuation(const std::string &input) {
    std::string out;
    out.reserve(input.size());

    for (size_t i = 0; i < input.size(); ++i) {
        char c = input[i];
        if (c == '.') {
            // Keep period only if between digits (e.g. 10368.1, 5.7, 77.5)
            if (i > 0 && i + 1 < input.size() &&
                isdigit((unsigned char)input[i - 1]) &&
                isdigit((unsigned char)input[i + 1])) {
                out.push_back('.');
            } else {
                out.push_back(' ');
            }
        } else if (c == ',' || c == ':' || c == ';' || c == '?' || c == '!' ||
                   c == '"' || c == '\'' || c == '(' || c == ')' || c == '[' ||
                   c == ']' || c == '{' || c == '}' || c == '<' || c == '>' ||
                   c == '=' || c == '_') {
            out.push_back(' ');
        } else if (c == '-') {
            // Check if this is "x-ray" (preceded by x/X and followed by ray/Ray)
            bool isXray = false;
            if (i >= 1 && i + 3 < input.size()) {
                if ((input[i - 1] == 'x' || input[i - 1] == 'X') &&
                    (i == 1 || isspace((unsigned char)input[i - 2])) &&
                    tolower((unsigned char)input[i + 1]) == 'r' &&
                    tolower((unsigned char)input[i + 2]) == 'a' &&
                    tolower((unsigned char)input[i + 3]) == 'y') {
                    isXray = true;
                }
            }
            if (isXray) {
                out.push_back('-');
            } else {
                // Split hyphenated words like eighty-seven, twenty-four into separate words
                out.push_back(' ');
            }
        } else {
            out.push_back(c);
        }
    }
    return out;
}

std::string VoiceQsoParser::normalizeVoiceInput(const std::string &input) {
    std::string stripped = stripPunctuation(input);
    std::istringstream iss(stripped);
    std::vector<std::string> words;
    std::string w;
    while (iss >> w) {
        words.push_back(w);
    }

    std::string result;
    for (size_t i = 0; i < words.size(); ++i) {
        std::string low = toLower(words[i]);

        // Check suffix combinations
        if ((low == "stroke" || low == "slash") && i + 1 < words.size()) {
            std::string nextLow = toLower(words[i + 1]);
            if (nextLow == "rover" || nextLow == "r") {
                result += "/R";
                i++;
                continue;
            } else if (nextLow == "portable" || nextLow == "p") {
                result += "/P";
                i++;
                continue;
            } else {
                result += "/";
                continue;
            }
        }
        if (low == "rover") {
            result += "/R";
            continue;
        }
        if (low == "portable") {
            result += "/P";
            continue;
        }
        if (low == "stroke" || low == "slash") {
            result += "/";
            continue;
        }

        appendPhoneticToken(words, i, result, true);
    }
    return result;
}

uint32_t VoiceQsoParser::getNominalCallingFreqKhz(const std::string &band) {
    const BandSpec *spec = findBandSpec(band);
    return spec ? spec->nominalFreqKhz : 10368100;
}

std::string VoiceQsoParser::bandToCabrillo(const std::string &band) {
    const BandSpec *spec = findBandSpec(band);
    return spec ? spec->cabrillo : "10G";
}

std::string VoiceQsoParser::bandToAdif(const std::string &band) {
    const BandSpec *spec = findBandSpec(band);
    return spec ? spec->adif : "3cm";
}

bool VoiceQsoParser::parseBandOrFreq(const std::string &token,
                                     std::string &outBand,
                                     uint32_t &outFreqKhz,
                                     std::string &outCabrillo,
                                     std::string &outAdif) {
    std::string t;
    for (char c : token) {
        if (c != ' ' && c != '\t') {
            t += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
    }

    struct TokenBandAlias {
        const char *token;
        const char *matchKey;
        uint32_t freqKhz;
    };
    static const TokenBandAlias kTokenAliases[] = {
        {"light",  "light", 0}, {"laser",   "light", 0},
        {"10",     "10",    0}, {"10g",     "10",    0}, {"10ghz",   "10",    0},
        {"24",     "24",    0}, {"24g",     "24",    0}, {"24ghz",   "24",    0},
        {"47",     "47",    0}, {"47g",     "47",    0}, {"47ghz",   "47",    0},
        {"76",     "76",    0}, {"76g",     "76",    0}, {"76ghz",   "76",    0},
        {"77.5",   "76",    0}, {"77.5g",   "76",    0}, {"77.5ghz", "76",    0},
        {"122",    "122",   0}, {"122g",    "122",   0}, {"122ghz",  "122",   0},
        {"134",    "134",   0}, {"134g",    "134",   0}, {"134ghz",  "134",   0},
        {"241",    "241",   0}, {"241g",    "241",   0}, {"241ghz",  "241",   0},
        {"5.7",    "5.7",   0}, {"5.7g",    "5.7",   0}, {"5.7ghz",  "5.7",   0}, {"5760", "5.7", 0},
        {"3.4",    "3.4",   0}, {"3.4g",    "3.4",   0}, {"3.4ghz",  "3.4",   0}, {"3456", "3.4", 0}, {"3400", "3.4", 0},
        {"2.3",    "2.3",   0}, {"2.3g",    "2.3",   0}, {"2.3ghz",  "2.3",   0},
        {"2.4",    "2.3",   0}, {"2.4g",    "2.3",   0}, {"2.4ghz",  "2.3",   0}, {"2304", "2.3", 0}, {"2400", "2.3", 0},
        {"1.2",    "1.2",   0}, {"1.2g",    "1.2",   0}, {"1.2ghz",  "1.2",   0}, {"1296", "1.2", 0},
        {"902",    "902",   0}, {"903",     "902",   0}, {"902m",    "902",   0}, {"902mhz", "902", 0}, {"33cm", "902", 0},
        {"432",    "432",   0}, {"432m",    "432",   0}, {"432mhz",  "432",   0}, {"70cm", "432", 0},
        {"222",    "222",   0}, {"222m",    "222",   0}, {"222mhz",  "222",   0}, {"1.25m", "222", 0},
        {"144",    "144",   0}, {"144m",    "144",   0}, {"144mhz",  "144",   0}, {"2m",   "144", 0},
        {"50",     "50",    0}, {"50m",     "50",    0}, {"50mhz",   "50",    0}, {"6m",   "50",  0},
        {"10368",  "10",    10368000},
        {"24192",  "24",    24192000},
        {"47088",  "47",    47088000},
        {"76032",  "76",    76032000},
        {"122250", "122",   122250000},
        {"134928", "134",   134928000},
        {"241056", "241",   241056000}
    };
    for (const auto &alias : kTokenAliases) {
        if (t == alias.token) {
            const BandSpec *spec = findBandSpec(alias.matchKey);
            if (spec) {
                outBand = spec->bandName;
                outFreqKhz = alias.freqKhz;
                outCabrillo = spec->cabrillo;
                outAdif = spec->adif;
                return true;
            }
        }
    }

    // Floating-point frequency in MHz (or GHz if suffixed with g/ghz)
    std::string numStr = t;
    double scaleToMhz = 1.0;
    if (numStr.size() > 3 && numStr.substr(numStr.size() - 3) == "mhz") {
        numStr = numStr.substr(0, numStr.size() - 3);
    } else if (numStr.size() > 1 && numStr.back() == 'm') {
        numStr.pop_back();
    } else if (numStr.size() > 3 && numStr.substr(numStr.size() - 3) == "ghz") {
        numStr = numStr.substr(0, numStr.size() - 3);
        scaleToMhz = 1000.0;
    } else if (numStr.size() > 1 && numStr.back() == 'g') {
        numStr.pop_back();
        scaleToMhz = 1000.0;
    }

    char *endPtr = nullptr;
    double f = std::strtod(numStr.c_str(), &endPtr) * scaleToMhz;
    if (endPtr && *endPtr == '\0' && f > 0.0) {
        if (f >= 50.0 && f <= 54.0) {
            outBand = "50 MHz";
            outCabrillo = "50";
            outAdif = "6m";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
        if (f >= 144.0 && f <= 148.0) {
            outBand = "144 MHz";
            outCabrillo = "144";
            outAdif = "2m";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
        if (f >= 222.0 && f <= 225.0) {
            outBand = "222 MHz";
            outCabrillo = "222";
            outAdif = "1.25m";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
        if (f >= 420.0 && f <= 450.0) {
            outBand = "432 MHz";
            outCabrillo = "432";
            outAdif = "70cm";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
        if (f >= 902.0 && f <= 928.0) {
            outBand = "902 MHz";
            outCabrillo = "902";
            outAdif = "33cm";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
        if (f >= 1240.0 && f <= 1300.0) {
            outBand = "1.2 GHz";
            outCabrillo = "1.2G";
            outAdif = "23cm";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
        if (f >= 2300.0 && f <= 2450.0) {
            outBand = "2.3 GHz";
            outCabrillo = "2.3G";
            outAdif = "13cm";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
        if (f >= 3300.0 && f <= 3500.0) {
            outBand = "3.4 GHz";
            outCabrillo = "3.4G";
            outAdif = "9cm";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
        if (f >= 5650.0 && f <= 5925.0) {
            outBand = "5.7 GHz";
            outCabrillo = "5.7G";
            outAdif = "6cm";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
        if (f >= 10000.0 && f <= 10500.0) {
            outBand = "10 GHz";
            outCabrillo = "10G";
            outAdif = "3cm";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
        if (f >= 24000.0 && f <= 24250.0) {
            outBand = "24 GHz";
            outCabrillo = "24G";
            outAdif = "1.2cm";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
        if (f >= 47000.0 && f <= 47200.0) {
            outBand = "47 GHz";
            outCabrillo = "47G";
            outAdif = "6mm";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
        if (f >= 75500.0 && f <= 81000.0) {
            outBand = "76 GHz";
            outCabrillo = "76G";
            outAdif = "4mm";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
        if (f >= 122250.0 && f <= 123000.0) {
            outBand = "122 GHz";
            outCabrillo = "122G";
            outAdif = "2.5mm";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
        if (f >= 134000.0 && f <= 141000.0) {
            outBand = "134 GHz";
            outCabrillo = "134G";
            outAdif = "2mm";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
        if (f >= 241000.0 && f <= 250000.0) {
            outBand = "241 GHz";
            outCabrillo = "241G";
            outAdif = "1mm";
            outFreqKhz = (uint32_t)std::round(f * 1000.0);
            return true;
        }
    }

    return false;
}

bool VoiceQsoParser::parseGrid(const std::string &input, std::string &outGrid) {
    std::string stripped = stripPunctuation(input);
    std::istringstream iss(stripped);
    std::vector<std::string> words;
    std::string w;
    while (iss >> w) {
        words.push_back(w);
    }

    if (words.empty()) return false;

    // Drop optional leading "target" keyword
    if (toLower(words[0]) == "target") {
        words.erase(words.begin());
    }

    if (words.empty()) return false;

    std::string assembled;
    for (size_t i = 0; i < words.size(); ++i) {
        appendPhoneticToken(words, i, assembled, false);
    }

    size_t len = assembled.size();
    if (len != 4 && len != 6 && len != 8) {
        return false;
    }

    // Validate Maidenhead field characters (A-R)
    if (assembled[0] < 'A' || assembled[0] > 'R' ||
        assembled[1] < 'A' || assembled[1] > 'R') {
        return false;
    }

    // Validate square digits (0-9)
    if (!isdigit((unsigned char)assembled[2]) ||
        !isdigit((unsigned char)assembled[3])) {
        return false;
    }

    // Validate subsquare letters (A-X)
    if (len >= 6) {
        if (assembled[4] < 'A' || assembled[4] > 'X' ||
            assembled[5] < 'A' || assembled[5] > 'X') {
            return false;
        }
    }

    // Validate extended square digits (0-9)
    if (len == 8) {
        if (!isdigit((unsigned char)assembled[6]) ||
            !isdigit((unsigned char)assembled[7])) {
            return false;
        }
    }

    // Verify geo coordinates conversion
    double lat = 0.0, lon = 0.0;
    if (!gridToGeoPoint(assembled.c_str(), &lat, &lon)) {
        return false;
    }

    // Format standard casing: Field uppercase, Square digits, Subsquare lowercase, Extended digits
    std::string formatted;
    formatted.push_back(std::toupper((unsigned char)assembled[0]));
    formatted.push_back(std::toupper((unsigned char)assembled[1]));
    formatted.push_back(assembled[2]);
    formatted.push_back(assembled[3]);
    if (len >= 6) {
        formatted.push_back(std::tolower((unsigned char)assembled[4]));
        formatted.push_back(std::tolower((unsigned char)assembled[5]));
    }
    if (len == 8) {
        formatted.push_back(assembled[6]);
        formatted.push_back(assembled[7]);
    }

    outGrid = formatted;
    return true;
}

bool VoiceQsoParser::parseQso(const std::string &input,
                             ParsedQso &out,
                             const std::string &defaultBand) {
    out = ParsedQso();

    std::string stripped = stripPunctuation(input);
    std::istringstream iss(stripped);
    std::vector<std::string> words;
    std::string w;
    while (iss >> w) {
        words.push_back(w);
    }

    if (words.empty()) {
        out.error_message = "Empty input";
        return false;
    }

    // Drop optional leading "log" keyword
    if (toLower(words[0]) == "log") {
        words.erase(words.begin());
    }

    if (words.empty()) {
        out.error_message = "Missing callsign";
        return false;
    }

    // Convert spoken tens/teens (e.g. "ten", "twenty four", "forty seven", "seventy six")
    // and unit words ("gigahertz", "megahertz") BEFORE band scanning so spoken bands
    // like "log k6mg twenty four" or "log k6mg ten" are recognized as bands rather than callsigns.
    std::vector<std::string> numNormalized;
    numNormalized.reserve(words.size());
    for (size_t i = 0; i < words.size(); ++i) {
        std::string low = toLower(words[i]);
        if (low == "gigahertz") {
            numNormalized.push_back("ghz");
            continue;
        }
        if (low == "megahertz") {
            numNormalized.push_back("mhz");
            continue;
        }
        char tenDigit = spokenTensToDigit(low);
        if (tenDigit != '\0') {
            std::string num(1, tenDigit);
            if (i + 1 < words.size()) {
                std::string nextLow = toLower(words[i + 1]);
                char unitDigit = spokenDigitToChar(nextLow);
                if (unitDigit != '\0' && unitDigit != '0') {
                    num.push_back(unitDigit);
                    i++;
                } else if (words[i + 1].size() == 1 && words[i + 1][0] >= '1' && words[i + 1][0] <= '9') {
                    num.push_back(words[i + 1][0]);
                    i++;
                } else {
                    num.push_back('0');
                }
            } else {
                num.push_back('0');
            }
            numNormalized.push_back(num);
            continue;
        }
        const char *teenStr = spokenTeenToString(low);
        if (teenStr != nullptr) {
            numNormalized.push_back(teenStr);
            continue;
        }
        numNormalized.push_back(words[i]);
    }
    words = std::move(numNormalized);

    // Split speech-to-text glued tokens where trailing characters represent a band
    // (e.g. Gboard generating "k6 ml24" -> "k6", "ml", "24" or "k6ml24" -> "k6ml", "24").
    // In international amateur radio regulations (ITU RR Article 19.68), callsigns
    // NEVER end in digits; suffixes must terminate in letters (or slash-rover/portable).
    std::vector<std::string> splitWords;
    for (const auto &word : words) {
        std::string dummyBand, dummyCab, dummyAdif;
        uint32_t dummyFreq = 0;
        // If the entire word is already a valid band, keep it intact
        if (parseBandOrFreq(word, dummyBand, dummyFreq, dummyCab, dummyAdif)) {
            splitWords.push_back(word);
            continue;
        }

        bool splitDone = false;
        // Search right-to-left for a split point where prefix ends in a letter (or /R or /P)
        // and suffix is a valid microwave band or calling frequency
        for (size_t k = word.size(); k > 1; --k) {
            std::string prefix = word.substr(0, k - 1);
            std::string suffix = word.substr(k - 1);

            bool validPrefixEnd = false;
            if (!prefix.empty()) {
                if (isalpha(static_cast<unsigned char>(prefix.back()))) {
                    validPrefixEnd = true;
                } else if (prefix.size() >= 2) {
                    std::string end2 = toLower(prefix.substr(prefix.size() - 2));
                    if (end2 == "/r" || end2 == "/p") {
                        validPrefixEnd = true;
                    }
                }
            }
            if (!validPrefixEnd) continue;

            if (parseBandOrFreq(suffix, dummyBand, dummyFreq, dummyCab, dummyAdif)) {
                splitWords.push_back(prefix);
                splitWords.push_back(suffix);
                splitDone = true;
                break;
            }
        }

        if (!splitDone) {
            splitWords.push_back(word);
        }
    }
    words = std::move(splitWords);

    // Combine adjacent tokens like ["10", "ghz"] or ["stroke", "rover"]
    std::vector<std::string> combinedWords;
    for (size_t i = 0; i < words.size(); ++i) {
        std::string low = toLower(words[i]);
        if (i + 1 < words.size()) {
            std::string nextLow = toLower(words[i + 1]);
            if ((low == "stroke" || low == "slash") && (nextLow == "rover" || nextLow == "r")) {
                combinedWords.push_back("/R");
                i++;
                continue;
            }
            if ((low == "stroke" || low == "slash") && (nextLow == "portable" || nextLow == "p")) {
                combinedWords.push_back("/P");
                i++;
                continue;
            }
            if ((nextLow == "ghz" || nextLow == "mhz") && !words[i].empty() &&
                std::isdigit(static_cast<unsigned char>(words[i][0]))) {
                combinedWords.push_back(words[i] + nextLow);
                i++;
                continue;
            }
            if ((nextLow == "meter" || nextLow == "meters") && (words[i] == "2" || words[i] == "6")) {
                combinedWords.push_back(words[i] + "m");
                i++;
                continue;
            }
            if (nextLow == "g") {
                std::string db, dc, da;
                uint32_t df = 0;
                if (parseBandOrFreq(words[i], db, df, dc, da)) {
                    combinedWords.push_back(words[i] + nextLow);
                    i++;
                    continue;
                }
            }
        }
        combinedWords.push_back(words[i]);
    }

    // Scan for band or frequency argument
    bool bandFound = false;
    std::vector<std::string> remainingWords;
    for (const auto &word : combinedWords) {
        std::string bnd, cab, adif;
        uint32_t freq = 0;
        if (parseBandOrFreq(word, bnd, freq, cab, adif)) {
            if (bandFound) {
                out.error_message = "Ambiguous multiple bands/frequencies";
                return false;
            }
            out.band = bnd;
            out.freq_khz = freq;
            out.cabrillo_band = cab;
            out.adif_band = adif;
            bandFound = true;
        } else if (looksLikeFrequencyToken(word)) {
            out.error_message = "Invalid or out-of-band frequency: " + word;
            return false;
        } else {
            remainingWords.push_back(word);
        }
    }

    // If no band specified, apply default band (e.g. "10 GHz")
    if (!bandFound) {
        if (!parseBandOrFreq(defaultBand, out.band, out.freq_khz, out.cabrillo_band, out.adif_band)) {
            out.band = "10 GHz";
            out.freq_khz = 0;
            out.cabrillo_band = "10G";
            out.adif_band = "3cm";
        }
    }

    if (remainingWords.empty()) {
        out.error_message = "Missing callsign";
        return false;
    }

    // Assemble callsign from remaining tokens
    std::string callsign;
    for (size_t i = 0; i < remainingWords.size(); ++i) {
        std::string low = toLower(remainingWords[i]);

        if (low == "/r" || low == "rover") {
            callsign += "/R";
            continue;
        }
        if (low == "/p" || low == "portable") {
            callsign += "/P";
            continue;
        }
        if (low == "stroke" || low == "slash" || low == "/") {
            callsign += "/";
            continue;
        }

        appendPhoneticToken(remainingWords, i, callsign, true);
    }

    // Clean up leading/consecutive/trailing slashes
    std::string cleanCall;
    cleanCall.reserve(callsign.size());
    for (char c : callsign) {
        if (c == '/' && (cleanCall.empty() || cleanCall.back() == '/')) {
            continue;
        }
        cleanCall.push_back(c);
    }
    while (!cleanCall.empty() && cleanCall.back() == '/') {
        cleanCall.pop_back();
    }

    // Validate callsign structure: length between 3 and 11, at least 2 letters, at least 1 digit
    if (cleanCall.size() < 3 || cleanCall.size() > 11) {
        out.error_message = "Invalid callsign length: " + cleanCall;
        return false;
    }

    size_t letterCount = 0;
    size_t digitCount = 0;
    bool validChars = true;

    for (char c : cleanCall) {
        if (isalpha((unsigned char)c)) letterCount++;
        else if (isdigit((unsigned char)c)) digitCount++;
        else if (c != '/') validChars = false;
    }

    if (letterCount < 2 || digitCount < 1 || !validChars) {
        out.error_message = "Invalid callsign: " + cleanCall;
        return false;
    }

    out.callsign = cleanCall;
    out.success = true;
    return true;
}
