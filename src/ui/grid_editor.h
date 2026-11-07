#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include "../config.h"

/**
 * @brief Manages the 6-character target Maidenhead grid editing state machine,
 *        cursor caret position, character rolling, and inactivity timeouts.
 */
class GridEditor {
public:
    GridEditor();

    // Direct grid value access
    void setGrid(const char *grid);
    const char* getGrid() const { return targetGrid_; }
    char* getGridBuffer() { return targetGrid_; }
    char getCharAt(int col) const;

    // Cursor position (0 to 5)
    int getCursorColumn() const { return cursorColumn_; }
    void setCursorColumn(int col);
    int& cursorColumnRef() { return cursorColumn_; }

    // Edit mode state
    bool isEditing() const { return isEditing_; }
    void setEditing(bool editing) { isEditing_ = editing; }
    bool& isEditingRef() { return isEditing_; }

    // Activity timestamp tracking
    unsigned long getLastActivity() const { return lastActivity_; }
    void recordActivity(unsigned long nowMs = 0);

    // Editing operations
    void rollChar(int delta, unsigned long nowMs = 0);
    void selectCharacter(int col, unsigned long nowMs = 0);

    // Evaluates inactivity timeout (5000 ms by default)
    // Returns true if timeout triggered and edit mode was deactivated
    bool checkTimeout(unsigned long nowMs, unsigned long timeoutMs = EDIT_TIMEOUT_MS);

private:
    char targetGrid_[7];
    int cursorColumn_;
    bool isEditing_;
    unsigned long lastActivity_;
};
