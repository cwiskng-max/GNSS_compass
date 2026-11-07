#include "grid_editor.h"

#if defined(ARDUINO)
#include <Arduino.h>
#endif

GridEditor::GridEditor()
    : cursorColumn_(0),
      isEditing_(true),
      lastActivity_(0)
{
    std::strcpy(targetGrid_, "ZA00AA");
}

void GridEditor::setGrid(const char *grid) {
    if (!grid) return;
    std::strncpy(targetGrid_, grid, 6);
    targetGrid_[6] = '\0';
}

char GridEditor::getCharAt(int col) const {
    if (col >= 0 && col < 6) {
        return targetGrid_[col];
    }
    return '\0';
}

void GridEditor::setCursorColumn(int col) {
    if (col < 0) col = 0;
    if (col > 5) col = 5;
    cursorColumn_ = col;
}

void GridEditor::recordActivity(unsigned long nowMs) {
#if defined(ARDUINO)
    if (nowMs == 0) nowMs = millis();
#endif
    lastActivity_ = nowMs;
}

void GridEditor::rollChar(int delta, unsigned long nowMs) {
    if (delta > 0) {
        INCG(targetGrid_[cursorColumn_]);
    } else if (delta < 0) {
        DECG(targetGrid_[cursorColumn_]);
    }
    isEditing_ = true;
    recordActivity(nowMs);
}

void GridEditor::selectCharacter(int col, unsigned long nowMs) {
    setCursorColumn(col);
    isEditing_ = true;
    recordActivity(nowMs);
}

bool GridEditor::checkTimeout(unsigned long nowMs, unsigned long timeoutMs) {
    if (isEditing_ && timeoutMs > 0) {
        if ((nowMs - lastActivity_) >= timeoutMs) {
            isEditing_ = false;
            return true;
        }
    }
    return false;
}

