#include "serial_cli.h"
#include "serial_cli_helpers.h"
#include "../geo/geo_math.h"
#include "../geo/maidenhead.h"
#include <cstdio>
#include <cstring>

using namespace cli_helpers;

void SerialCli::printTargetHelp() {
    println();
    println("=== Target Preset Commands ===");
    println("  target <GRID|INDEX>       - Set active target grid (phonetics or 1-based preset #)");
    println("  target select <INDEX>     - Select saved preset by 1-based index");
    println("  target add <GRID> [LABEL] - Add or update preset (spaces allowed in label)");
    println("  target save [LABEL]       - Save current active target grid as preset");
    println("  target del <INDEX|GRID>   - Delete preset by 1-based index or 6-char grid");
    println("  target list               - List all saved target presets");
    println("  target max [N]            - Show or set max capacity (10-500, default 100)");
    println("  target clear              - Wipe all saved target presets");
    println("==============================");
    println();
}

void SerialCli::handleTargetCommand(const char *arg, unsigned long nowMs) {
    while (arg && (*arg == ' ' || *arg == '\t')) arg++;
    if (!arg || *arg == '\0') {
        println("[TARGET ERROR] Missing grid square. Usage: target <grid> (e.g. 'target CM88qk' or 'target charlie mike 88 quebec kilo')");
        return;
    }

    if (equalsIgnoreCase(arg, "help") || std::strcmp(arg, "?") == 0) {
        printTargetHelp();
        return;
    }

    uint32_t nowSec = 0;
    if (gpsManager_.hasTime() && gpsManager_.getEpochTime() > 0) {
        nowSec = gpsManager_.getEpochTime();
    } else if (nowMs > 0) {
        nowSec = static_cast<uint32_t>(nowMs / 1000);
    }

    if (hasSubcommandPrefix(arg, "add") || hasSubcommandPrefix(arg, "ad")) {
        if (!targetPresetManager_) {
            println("[TARGET] Target preset manager not available.");
            return;
        }
        const char *p = skipSubcommandPrefix(arg, hasSubcommandPrefix(arg, "add") ? 3 : 2);
        if (*p == '\0') {
            println("[TARGET ERROR] Usage: target add <GRID> [LABEL]");
            return;
        }

        // Extract first whitespace-delimited token as grid, or parse phonetic grid prefix
        char gridToken[32];
        size_t gLen = 0;
        while (p[gLen] && p[gLen] != ' ' && p[gLen] != '\t' && gLen < sizeof(gridToken) - 1) {
            gridToken[gLen] = p[gLen];
            gLen++;
        }
        gridToken[gLen] = '\0';

        char normGrid[7] = {0};
        const char *lblStart = p + gLen;
        if (!TargetPresetManager::normalizeGrid(gridToken, normGrid)) {
            bool parsedPhonetic = false;
            const char *scan = p;
            std::string prefix;
            for (int w = 0; w < 6 && *scan; ++w) {
                while (*scan == ' ' || *scan == '\t') scan++;
                if (*scan == '\0') break;
                const char *wStart = scan;
                while (*scan && *scan != ' ' && *scan != '\t') scan++;
                if (!prefix.empty()) prefix.push_back(' ');
                prefix.append(wStart, static_cast<size_t>(scan - wStart));

                std::string voiceNorm = VoiceQsoParser::normalizeVoiceInput(prefix);
                if (voiceNorm.size() == 6 && TargetPresetManager::normalizeGrid(voiceNorm.c_str(), normGrid)) {
                    lblStart = scan;
                    parsedPhonetic = true;
                    break;
                }
                if (voiceNorm.size() > 6) {
                    break;
                }
            }
            if (!parsedPhonetic) {
                printFormat("[TARGET ERROR] Invalid 6-char Maidenhead grid: '%s'\r\n", gridToken);
                return;
            }
        }

        // Remainder (if any) is the optional label (spaces allowed; strip leading comma/period separators)
        while (*lblStart == ' ' || *lblStart == '\t' || *lblStart == ',' || *lblStart == '.') lblStart++;
        char labelBuf[64];
        size_t lLen = 0;
        while (lblStart[lLen] && lLen < sizeof(labelBuf) - 1) {
            labelBuf[lLen] = lblStart[lLen];
            lLen++;
        }
        while (lLen > 0 && (labelBuf[lLen - 1] == ' ' || labelBuf[lLen - 1] == '\t')) {
            lLen--;
        }
        labelBuf[lLen] = '\0';

        bool existed = (targetPresetManager_->findIndexByGrid(normGrid) >= 0);
        if (!targetPresetManager_->addOrUpdatePreset(normGrid, lLen > 0 ? labelBuf : nullptr, nowSec)) {
            printFormat("[TARGET ERROR] Failed to add preset '%s'.\r\n", normGrid);
            return;
        }

        int idx = targetPresetManager_->findAddedIndexByGrid(normGrid);
        const TargetPreset *preset = (idx >= 0) ? targetPresetManager_->getPresetByAddedOrder(static_cast<size_t>(idx)) : nullptr;
        const char *verb = existed ? "Updated" : "Added";
        if (preset && preset->label[0] != '\0') {
            printFormat("[TARGET] %s #%d: %s (%s)\r\n", verb, idx + 1, preset->grid, preset->label);
        } else {
            printFormat("[TARGET] %s #%d: %s\r\n", verb, idx + 1, normGrid);
        }
        if (stateChangedCb_) stateChangedCb_(stateChangedContext_);
        return;
    }

    if (hasSubcommandPrefix(arg, "save")) {
        if (!targetPresetManager_) {
            println("[TARGET] Target preset manager not available.");
            return;
        }
        const char *p = skipSubcommandPrefix(arg, 4);
        char labelBuf[64];
        size_t lLen = 0;
        while (p[lLen] && lLen < sizeof(labelBuf) - 1) {
            labelBuf[lLen] = p[lLen];
            lLen++;
        }
        while (lLen > 0 && (labelBuf[lLen - 1] == ' ' || labelBuf[lLen - 1] == '\t')) {
            lLen--;
        }
        labelBuf[lLen] = '\0';

        CliNavContext nctx = getNavContext();
        char normGrid[7];
        if (!TargetPresetManager::normalizeGrid(nctx.targetGrid, normGrid)) {
            println("[TARGET ERROR] No valid 6-char active target grid to save.");
            return;
        }

        if (!targetPresetManager_->addOrUpdatePreset(normGrid, lLen > 0 ? labelBuf : nullptr, nowSec)) {
            printFormat("[TARGET ERROR] Failed to save preset '%s'.\r\n", normGrid);
            return;
        }

        int idx = targetPresetManager_->findAddedIndexByGrid(normGrid);
        const TargetPreset *preset = (idx >= 0) ? targetPresetManager_->getPresetByAddedOrder(static_cast<size_t>(idx)) : nullptr;
        if (preset && preset->label[0] != '\0') {
            printFormat("[TARGET] Saved #%d: %s (%s)\r\n", idx + 1, preset->grid, preset->label);
        } else {
            printFormat("[TARGET] Saved #%d: %s\r\n", idx + 1, normGrid);
        }
        if (stateChangedCb_) stateChangedCb_(stateChangedContext_);
        return;
    }

    if (hasSubcommandPrefix(arg, "del") || hasSubcommandPrefix(arg, "delete")) {
        if (!targetPresetManager_) {
            println("[TARGET] Target preset manager not available.");
            return;
        }
        const char *p = skipSubcommandPrefix(arg, startsWithIgnoreCase(arg, "delete") ? 6 : 3);
        if (*p == '\0') {
            println("[TARGET ERROR] Usage: target del <INDEX|GRID>");
            return;
        }

        int index1Based = 0;
        if (parsePositiveIntOrVoice(p, index1Based)) {
            size_t count = targetPresetManager_->getCount();
            if (count == 0) {
                println("[TARGET ERROR] Preset list is empty.");
                return;
            }
            if (index1Based < 1 || static_cast<size_t>(index1Based) > count) {
                printFormat("[TARGET ERROR] Preset #%d out of range (1-%zu).\r\n", index1Based, count);
                return;
            }
            const TargetPreset *existing = targetPresetManager_->getPresetByAddedOrder(static_cast<size_t>(index1Based - 1));
            char deletedGrid[7] = {0};
            if (existing) {
                std::snprintf(deletedGrid, sizeof(deletedGrid), "%s", existing->grid);
            }
            targetPresetManager_->removePresetByAddedOrder(static_cast<size_t>(index1Based - 1));
            printFormat("[TARGET] Deleted #%d: %s\r\n", index1Based, deletedGrid);
            if (stateChangedCb_) stateChangedCb_(stateChangedContext_);
            return;
        }

        std::string parsedGrid;
        char normGrid[7] = {0};
        if (TargetPresetManager::normalizeGrid(p, normGrid) ||
            (VoiceQsoParser::parseGrid(p, parsedGrid) &&
             TargetPresetManager::normalizeGrid(parsedGrid.c_str(), normGrid))) {
            int idx = targetPresetManager_->findAddedIndexByGrid(normGrid);
            if (idx < 0) {
                printFormat("[TARGET ERROR] Preset '%s' not found.\r\n", normGrid);
                return;
            }
            targetPresetManager_->removePresetByGrid(normGrid);
            printFormat("[TARGET] Deleted #%d: %s\r\n", idx + 1, normGrid);
            if (stateChangedCb_) stateChangedCb_(stateChangedContext_);
            return;
        }

        printFormat("[TARGET ERROR] Invalid preset index or grid: '%s'\r\n", p);
        return;
    }

    if (hasSubcommandPrefix(arg, "list") || hasSubcommandPrefix(arg, "show")) {
        if (!targetPresetManager_) {
            println("[TARGET] Target preset manager not available.");
            return;
        }
        size_t count = targetPresetManager_->getCount();
        size_t maxTargets = targetPresetManager_->getMaxTargets();
        printFormat("=== Saved Target Presets (%zu/%zu) ===\r\n", count, maxTargets);
        if (count == 0) {
            println("  (No saved target presets)");
            println("===================================");
            return;
        }
        pendingExportFormat_ = ExportFormat::TargetList;
        pendingExportIndex_ = 0;
        pendingExportSource_ = activeSource_;
        servicePendingExport(false);
        return;
    }

    if (hasSubcommandPrefix(arg, "max")) {
        if (!targetPresetManager_) {
            println("[TARGET] Target preset manager not available.");
            return;
        }
        const char *p = skipSubcommandPrefix(arg, 3);
        if (*p == '\0') {
            printFormat("[TARGET] Max presets capacity: %zu (range: %u-%u)\r\n",
                        targetPresetManager_->getMaxTargets(), MIN_TARGET_PRESETS, ABS_MAX_TARGET_PRESETS);
            return;
        }
        int val = 0;
        if (!parsePositiveIntOrVoice(p, val)) {
            printFormat("[TARGET ERROR] Invalid capacity. Range: %u-%u.\r\n",
                        MIN_TARGET_PRESETS, ABS_MAX_TARGET_PRESETS);
            return;
        }
        targetPresetManager_->setMaxTargets(static_cast<size_t>(val), true);
        printFormat("[TARGET] Max presets capacity set to %zu\r\n", targetPresetManager_->getMaxTargets());
        if (stateChangedCb_) stateChangedCb_(stateChangedContext_);
        return;
    }

    if (hasSubcommandPrefix(arg, "clear")) {
        if (!targetPresetManager_) {
            println("[TARGET] Target preset manager not available.");
            return;
        }
        targetPresetManager_->clearAll();
        println("[TARGET] Cleared all presets.");
        if (stateChangedCb_) stateChangedCb_(stateChangedContext_);
        return;
    }

    auto applyActiveTarget = [&](const char *grid, double &outBearing, double &outDistKm) {
        if (targetGridCb_) {
            targetGridCb_(grid, nowMs, targetGridContext_);
        } else {
            setTargetGrid(grid);
        }

        CliNavContext nctx = getNavContext();
        outBearing = nctx.targetBearing;
        outDistKm = nctx.roverDistanceMeters / 1000.0;

        // Fallback geodesic calculation when in standalone test mode without full AppController provider
        if (!navContextFn_ && nctx.currentGrid[0] != '\0') {
            GeoPoint curPt, tgtPt;
            if (gridToGeoPoint(nctx.currentGrid, curPt) && gridToGeoPoint(grid, tgtPt)) {
                double distMeters = 0.0, brg = 0.0;
                calculateWgs84(curPt, tgtPt, distMeters, brg);
                outBearing = brg;
                outDistKm = distMeters / 1000.0;
                fallbackNavContext_.targetBearing = outBearing;
                fallbackNavContext_.roverDistanceMeters = distMeters;
            }
        }
    };

    auto selectPresetByIndex = [&](int index1Based) {
        if (!targetPresetManager_) {
            println("[TARGET] Target preset manager not available.");
            return;
        }
        size_t count = targetPresetManager_->getCount();
        if (count == 0) {
            println("[TARGET ERROR] Preset list is empty.");
            return;
        }
        if (index1Based < 1 || static_cast<size_t>(index1Based) > count) {
            printFormat("[TARGET ERROR] Preset #%d out of range (1-%zu).\r\n", index1Based, count);
            return;
        }
        const TargetPreset *preset = targetPresetManager_->getPresetByAddedOrder(static_cast<size_t>(index1Based - 1));
        if (!preset) {
            printFormat("[TARGET ERROR] Preset #%d not found.\r\n", index1Based);
            return;
        }
        char selectedGrid[7] = {0};
        char selectedLabel[sizeof(preset->label)] = {0};
        std::snprintf(selectedGrid, sizeof(selectedGrid), "%s", preset->grid);
        std::snprintf(selectedLabel, sizeof(selectedLabel), "%s", preset->label);

        targetPresetManager_->recordUseByAddedOrder(static_cast<size_t>(index1Based - 1), nowSec);

        double bearing = 0.0, distKm = 0.0;
        applyActiveTarget(selectedGrid, bearing, distKm);

        if (selectedLabel[0] != '\0') {
            printFormat("[TARGET] Selected #%d: %s (%s) (Bearing: %.1f deg, Distance: %.1f km)\r\n",
                        index1Based, selectedGrid, selectedLabel, bearing, distKm);
        } else {
            printFormat("[TARGET] Selected #%d: %s (Bearing: %.1f deg, Distance: %.1f km)\r\n",
                        index1Based, selectedGrid, bearing, distKm);
        }
        if (stateChangedCb_) stateChangedCb_(stateChangedContext_);
    };

    if (hasSubcommandPrefix(arg, "select")) {
        const char *p = skipSubcommandPrefix(arg, 6);
        if (*p == '\0') {
            println("[TARGET ERROR] Usage: target select <INDEX>");
            return;
        }
        int index1Based = 0;
        if (!parsePositiveIntOrVoice(p, index1Based)) {
            printFormat("[TARGET ERROR] Invalid preset index: '%s'\r\n", p);
            return;
        }
        selectPresetByIndex(index1Based);
        return;
    }

    int directIndex = 0;
    if (parsePositiveIntOrVoice(arg, directIndex)) {
        selectPresetByIndex(directIndex);
        return;
    }

    std::string cleanGrid;
    if (!VoiceQsoParser::parseGrid(arg, cleanGrid)) {
        printFormat("[TARGET ERROR] Invalid Maidenhead grid: '%s'\r\n", arg);
        return;
    }

    double bearing = 0.0, distKm = 0.0;
    applyActiveTarget(cleanGrid.c_str(), bearing, distKm);

    printFormat("[TARGET] Target set to %s (Bearing: %.1f deg, Distance: %.1f km)\r\n",
                cleanGrid.c_str(), bearing, distKm);
}
