/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2005-2009 openDarkEngine team
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *****************************************************************************/

// In-game debug console for runtime settings management.
// Uses bgfx debug text overlay — no font loading, no shaders, no textures.
//
// Settings are organized into named groups (setGroup) for visual clarity.
// Keyword search matches against both setting names and group names in one
// step, so typing "audio" surfaces all audio settings even if the individual
// setting names don't contain "audio".
//
// Interaction flow:
//   ` (backtick) opens the console. Tab-complete a setting name, Enter to
//   lock it in, then enter a value (arrows for bool/categorical, digits for
//   float). Enter submits and closes. ` or Esc cancels at any point.

#pragma once

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include <SDL.h>
#include <bgfx/bgfx.h>

namespace Darkness {

// ── Setting types ──────────────────────────────────────────────────────────
enum class SettingType { Bool, Categorical, Float };

// Descriptor for a single console-accessible setting.
// The console only sees string getter/setter lambdas — no knowledge of
// RenderConfig or any other concrete type.
struct SettingDesc {
    std::string name;
    std::string group;                // semantic group (set via setGroup)
    std::string description;          // One-line help text shown when highlighted
    SettingType type;
    std::vector<std::string> options; // Bool: {"off","on"}, Categorical: named list
    float minVal = 0, maxVal = 0;     // Float range (ignored for other types)
    std::function<std::string()> get;           // current value as string
    std::function<bool(const std::string &)> set; // apply value, return true on success
};

// Entry in the display list — either a group header or a setting reference.
struct DisplayEntry {
    int settingIdx = -1;    // index into settings_ (-1 for group headers)
    std::string groupName;  // populated for headers only
    bool isHeader() const { return settingIdx < 0; }
};

// ── DebugConsole ───────────────────────────────────────────────────────────
//
// Three-state FSM:
//   CLOSED  --(`)--> SELECT_KEY  --(Enter)--> EDIT_VALUE  --(Enter)--> CLOSED
//     ^                  |                         |
//     +----(`/Esc)-------+------(`/Esc)------------+
//
class DebugConsole {
public:
    enum class State { Closed, SelectKey, EditValue };

    // ── Group management ────────────────────────────────────────────────────
    // All subsequent addBool/addCategorical/addFloat calls will be placed
    // into this group until the next setGroup call.
    void setGroup(const std::string &group) { currentGroup_ = group; }

    // ── Registration ───────────────────────────────────────────────────────

    void addBool(const std::string &name,
                 std::function<bool()> getter,
                 std::function<void(bool)> setter,
                 const std::string &desc = "") {
        SettingDesc sd;
        sd.name = name;
        sd.group = currentGroup_;
        sd.description = desc;
        sd.type = SettingType::Bool;
        sd.options = {"off", "on"};
        sd.get = [getter]() { return getter() ? "on" : "off"; };
        sd.set = [setter](const std::string &v) {
            if (v == "on")  { setter(true);  return true; }
            if (v == "off") { setter(false); return true; }
            return false;
        };
        settings_.push_back(std::move(sd));
    }

    void addCategorical(const std::string &name,
                        const std::vector<std::string> &options,
                        std::function<int()> getter,
                        std::function<void(int)> setter,
                        const std::string &desc = "") {
        SettingDesc sd;
        sd.name = name;
        sd.group = currentGroup_;
        sd.description = desc;
        sd.type = SettingType::Categorical;
        sd.options = options;
        sd.get = [getter, options]() -> std::string {
            int idx = getter();
            if (idx >= 0 && idx < static_cast<int>(options.size()))
                return options[idx];
            return "?";
        };
        sd.set = [setter, options](const std::string &v) {
            for (int i = 0; i < static_cast<int>(options.size()); ++i) {
                if (options[i] == v) { setter(i); return true; }
            }
            return false;
        };
        settings_.push_back(std::move(sd));
    }

    void addFloat(const std::string &name, float minVal, float maxVal,
                  std::function<float()> getter,
                  std::function<void(float)> setter,
                  const std::string &desc = "") {
        SettingDesc sd;
        sd.name = name;
        sd.group = currentGroup_;
        sd.description = desc;
        sd.type = SettingType::Float;
        sd.minVal = minVal;
        sd.maxVal = maxVal;
        sd.get = [getter]() -> std::string {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.3f", getter());
            return std::string(buf);
        };
        sd.set = [setter, minVal, maxVal](const std::string &v) {
            char *end = nullptr;
            float f = std::strtof(v.c_str(), &end);
            if (end == v.c_str() || *end != '\0') return false; // parse failure
            if (f < minVal || f > maxVal) return false;          // out of range
            setter(f);
            return true;
        };
        settings_.push_back(std::move(sd));
    }

    // ── Event handling ─────────────────────────────────────────────────────
    // Returns true if the event was consumed (caller should skip camera input).

    bool handleEvent(const SDL_Event &ev) {
        // ── CLOSED state: only consume the backtick toggle ──
        if (state_ == State::Closed) {
            if (ev.type == SDL_KEYDOWN &&
                ev.key.keysym.sym == SDLK_BACKQUOTE) {
                open();
                return true;
            }
            return false;
        }

        // ── Console is open: consume everything ──

        // Filter out text input events for the backtick character itself,
        // which arrives as SDL_TEXTINPUT right after the KEYDOWN
        if (ev.type == SDL_TEXTINPUT && ev.text.text[0] == '`')
            return true;

        // ── SELECT_KEY state ──
        if (state_ == State::SelectKey) {
            if (ev.type == SDL_KEYDOWN) {
                auto sym = ev.key.keysym.sym;

                if (sym == SDLK_BACKQUOTE || sym == SDLK_ESCAPE) {
                    close();
                    return true;
                }
                if (sym == SDLK_TAB) {
                    // Tab: if input matches exactly one setting, lock it in.
                    // If multiple matches, cycle through them.
                    if (countSettings() == 1) {
                        int si = firstSettingDisplayIdx();
                        if (si >= 0) {
                            selectedDisplayIdx_ = si;
                            inputBuf_ = settings_[displayList_[si].settingIdx].name;
                            lockKey();
                        }
                    } else if (hasSettings()) {
                        moveSelection(1);
                    }
                    return true;
                }
                if (sym == SDLK_RETURN || sym == SDLK_KP_ENTER) {
                    int si = selectedSettingIdx();
                    if (si >= 0) {
                        inputBuf_ = settings_[si].name;
                        lockKey();
                    }
                    return true;
                }
                if (sym == SDLK_BACKSPACE) {
                    if (!inputBuf_.empty()) {
                        inputBuf_.pop_back();
                        updateFilter();
                    }
                    return true;
                }
                if (sym == SDLK_UP) {
                    moveSelection(-1);
                    return true;
                }
                if (sym == SDLK_DOWN) {
                    moveSelection(1);
                    return true;
                }
            }

            // Append typed characters to the filter input
            if (ev.type == SDL_TEXTINPUT) {
                inputBuf_ += ev.text.text;
                updateFilter();
                return true;
            }

            // Consume all other events while open (prevent camera movement)
            return true;
        }

        // ── EDIT_VALUE state ──
        if (state_ == State::EditValue) {
            if (ev.type == SDL_KEYDOWN) {
                auto sym = ev.key.keysym.sym;

                if (sym == SDLK_BACKQUOTE || sym == SDLK_ESCAPE) {
                    close();
                    return true;
                }
                if (sym == SDLK_RETURN || sym == SDLK_KP_ENTER) {
                    submitValue();
                    return true;
                }

                auto &sd = settings_[editingIdx_];
                if (sd.type != SettingType::Float) {
                    // Bool / Categorical: arrow keys cycle through options
                    if (sym == SDLK_LEFT) {
                        optionIdx_--;
                        if (optionIdx_ < 0)
                            optionIdx_ = static_cast<int>(sd.options.size()) - 1;
                        return true;
                    }
                    if (sym == SDLK_RIGHT) {
                        optionIdx_ = (optionIdx_ + 1) %
                                     static_cast<int>(sd.options.size());
                        return true;
                    }
                } else {
                    // Float: backspace deletes last char
                    if (sym == SDLK_BACKSPACE) {
                        if (!valueBuf_.empty()) valueBuf_.pop_back();
                        return true;
                    }
                }
            }

            // Float text input: only accept digits, '.', '-'
            if (ev.type == SDL_TEXTINPUT) {
                auto &sd = settings_[editingIdx_];
                if (sd.type == SettingType::Float) {
                    for (const char *p = ev.text.text; *p; ++p) {
                        char c = *p;
                        if ((c >= '0' && c <= '9') || c == '.' || c == '-')
                            valueBuf_ += c;
                    }
                }
                return true;
            }

            // Consume everything
            return true;
        }

        return true; // should not reach here
    }

    // ── Rendering ──────────────────────────────────────────────────────────
    // Call every frame just before bgfx::frame(). No-op when closed.

    void render() {
        if (state_ == State::Closed) return;

        bgfx::setDebug(BGFX_DEBUG_TEXT);
        bgfx::dbgTextClear();

        // Debug text grid: 1280/8 = 160 cols, 720/16 = 45 rows
        const uint16_t COL0 = 1;   // left margin
        const uint16_t ROWS = 45;

        // VGA color attributes: upper nibble = bg, lower nibble = fg
        const uint8_t ATTR_NORMAL   = 0x0f; // white on black
        const uint8_t ATTR_HINT     = 0x07; // light gray on black
        const uint8_t ATTR_SELECTED = 0x1f; // white on dark blue
        const uint8_t ATTR_NAME     = 0x0a; // green on black
        const uint8_t ATTR_ERROR    = 0x0c; // red on black
        const uint8_t ATTR_HELP     = 0x08; // dark gray on black
        const uint8_t ATTR_GROUP    = 0x03; // dark cyan on black

        // Decrement error flash timer
        if (errorFlash_ && --errorFrames_ <= 0)
            errorFlash_ = false;

        if (state_ == State::SelectKey) {
            int totalRows = static_cast<int>(displayList_.size());
            int listCount = std::min(totalRows, MAX_VISIBLE);

            // Layout from bottom: help bar, separator, list, separator, input
            uint16_t helpY = ROWS - 1;
            uint16_t sepBottomY = helpY - 1;
            uint16_t listEndY = sepBottomY - 1;
            uint16_t listStartY = listEndY - listCount + 1;
            uint16_t sepTopY = listStartY - 1;
            uint16_t inputY = sepTopY - 1;

            // Input prompt with cursor
            bgfx::dbgTextPrintf(COL0, inputY, ATTR_NORMAL, "> %s_",
                                inputBuf_.c_str());

            // Top separator
            drawSeparator(COL0, sepTopY, 50);

            // Completion list with scroll window
            int scrollOffset = computeScrollOffset(
                totalRows, MAX_VISIBLE, selectedDisplayIdx_);

            for (int i = 0; i < listCount; ++i) {
                int di = scrollOffset + i;
                const auto &entry = displayList_[di];

                if (entry.isHeader()) {
                    // Group header row — not selectable
                    bgfx::dbgTextPrintf(COL0 + 1, listStartY + i, ATTR_GROUP,
                        "[%s]", entry.groupName.c_str());
                } else {
                    // Setting row
                    bool isSel = (di == selectedDisplayIdx_);
                    uint8_t attr = isSel ? ATTR_SELECTED : ATTR_NAME;
                    std::string val = settings_[entry.settingIdx].get();
                    bgfx::dbgTextPrintf(COL0 + 2, listStartY + i, attr,
                        "%-20s = %s",
                        settings_[entry.settingIdx].name.c_str(), val.c_str());
                }
            }

            // Bottom separator
            drawSeparator(COL0, sepBottomY, 50);

            // Description of highlighted setting (if available)
            int si = selectedSettingIdx();
            if (si >= 0 && !settings_[si].description.empty()) {
                bgfx::dbgTextPrintf(COL0 + 1, helpY, ATTR_HINT,
                    "%s", settings_[si].description.c_str());
                helpY++;  // push help bar down
            }

            // Help bar
            bgfx::dbgTextPrintf(COL0, helpY, ATTR_HELP,
                "Tab: complete  %s%s: navigate  Enter: select  `: cancel",
                "\x18", "\x19"); // up/down arrow chars

        } else if (state_ == State::EditValue) {
            auto &sd = settings_[editingIdx_];

            // Layout from bottom: help bar, separator, hint, value line
            uint16_t helpY = ROWS - 1;
            uint16_t sepY = helpY - 1;
            uint16_t hintY = sepY - 1;
            uint16_t valueY = hintY - 1;

            if (sd.type == SettingType::Float) {
                // Float editing: show typed value with cursor
                uint8_t attr = errorFlash_ ? ATTR_ERROR : ATTR_NORMAL;
                bgfx::dbgTextPrintf(COL0, valueY, attr, "%s = %s_",
                                    sd.name.c_str(), valueBuf_.c_str());

                // Range hint
                char hint[64];
                std::snprintf(hint, sizeof(hint),
                    "Range: %.3f - %.3f", sd.minVal, sd.maxVal);
                bgfx::dbgTextPrintf(COL0 + 1, hintY, ATTR_HINT, "%s", hint);
            } else {
                // Bool / Categorical: show arrow-cycling UI
                uint8_t attr = errorFlash_ ? ATTR_ERROR : ATTR_NORMAL;
                bgfx::dbgTextPrintf(COL0, valueY, attr,
                    "%s = %s %s %s", sd.name.c_str(),
                    "\x1b", // left arrow char
                    sd.options[optionIdx_].c_str(),
                    "\x1a"); // right arrow char

                // Show all options with current one highlighted
                std::string optList;
                for (int i = 0; i < static_cast<int>(sd.options.size()); ++i) {
                    if (i > 0) optList += "  ";
                    if (i == optionIdx_)
                        optList += "[" + sd.options[i] + "]";
                    else
                        optList += " " + sd.options[i] + " ";
                }
                bgfx::dbgTextPrintf(COL0 + 1, hintY, ATTR_HINT, "%s",
                                    optList.c_str());
            }

            // Separator
            drawSeparator(COL0, sepY, 50);

            // Help bar
            if (sd.type == SettingType::Float) {
                bgfx::dbgTextPrintf(COL0, helpY, ATTR_HELP,
                    "Type value  Enter: apply  `: cancel");
            } else {
                bgfx::dbgTextPrintf(COL0, helpY, ATTR_HELP,
                    "%s%s: cycle  Enter: apply  `: cancel",
                    "\x1b", "\x1a"); // left/right arrow chars
            }
        }
    }

    // ── Queries ────────────────────────────────────────────────────────────

    bool isOpen() const { return state_ != State::Closed; }
    State getState() const { return state_; }

private:
    static constexpr int MAX_VISIBLE = 14; // max list rows shown at once

    State state_ = State::Closed;
    std::vector<SettingDesc> settings_;
    std::string currentGroup_;   // group assigned to subsequent registrations

    // SELECT_KEY state
    std::string inputBuf_;
    int selectedDisplayIdx_ = 0;           // index into displayList_ (non-header)
    std::vector<DisplayEntry> displayList_; // interleaved headers + settings

    // EDIT_VALUE state
    int editingIdx_ = -1;
    std::string valueBuf_;
    int optionIdx_ = 0;

    // Error flash (brief red highlight on invalid input)
    bool errorFlash_ = false;
    int errorFrames_ = 0;

    // ── Display list helpers ──────────────────────────────────────────────

    // Index of the setting (in settings_) that is currently highlighted,
    // or -1 if nothing is selected.
    int selectedSettingIdx() const {
        if (selectedDisplayIdx_ >= 0 &&
            selectedDisplayIdx_ < static_cast<int>(displayList_.size()) &&
            !displayList_[selectedDisplayIdx_].isHeader()) {
            return displayList_[selectedDisplayIdx_].settingIdx;
        }
        return -1;
    }

    // Number of non-header entries in the display list.
    int countSettings() const {
        int n = 0;
        for (const auto &e : displayList_)
            if (!e.isHeader()) ++n;
        return n;
    }

    // Index (in displayList_) of the first non-header entry, or -1.
    int firstSettingDisplayIdx() const {
        for (int i = 0; i < static_cast<int>(displayList_.size()); ++i)
            if (!displayList_[i].isHeader()) return i;
        return -1;
    }

    bool hasSettings() const {
        for (const auto &e : displayList_)
            if (!e.isHeader()) return true;
        return false;
    }

    // Case-insensitive substring search.
    static bool containsCI(const std::string &haystack,
                           const std::string &needle) {
        if (needle.empty()) return true;
        if (needle.size() > haystack.size()) return false;
        auto it = std::search(
            haystack.begin(), haystack.end(),
            needle.begin(), needle.end(),
            [](char a, char b) {
                return std::tolower(static_cast<unsigned char>(a)) ==
                       std::tolower(static_cast<unsigned char>(b));
            });
        return it != haystack.end();
    }

    // ── State transitions ──────────────────────────────────────────────────

    void open() {
        state_ = State::SelectKey;
        inputBuf_.clear();
        selectedDisplayIdx_ = 0;
        errorFlash_ = false;
        updateFilter(); // show all settings initially
        SDL_SetRelativeMouseMode(SDL_FALSE);
        SDL_StartTextInput();
    }

    void close() {
        state_ = State::Closed;
        inputBuf_.clear();
        valueBuf_.clear();
        editingIdx_ = -1;
        errorFlash_ = false;
        errorFrames_ = 0;
        SDL_StopTextInput();
        SDL_SetRelativeMouseMode(SDL_TRUE);
        bgfx::setDebug(0); // disable debug text overlay
    }

    void lockKey() {
        int si = selectedSettingIdx();
        if (si < 0) return;
        editingIdx_ = si;
        state_ = State::EditValue;
        errorFlash_ = false;

        auto &sd = settings_[editingIdx_];
        if (sd.type == SettingType::Float) {
            // Pre-fill with current value for easy editing
            valueBuf_ = sd.get();
        } else {
            // Find current option index for arrow-key cycling
            std::string cur = sd.get();
            optionIdx_ = 0;
            for (int i = 0; i < static_cast<int>(sd.options.size()); ++i) {
                if (sd.options[i] == cur) { optionIdx_ = i; break; }
            }
        }
    }

    void submitValue() {
        auto &sd = settings_[editingIdx_];
        std::string val = (sd.type == SettingType::Float)
            ? valueBuf_
            : sd.options[optionIdx_];
        bool ok = sd.set(val);
        if (ok) {
            std::fprintf(stderr, "Console: %s = %s\n",
                         sd.name.c_str(), val.c_str());
            close();
        } else {
            // Flash red to indicate invalid input
            errorFlash_ = true;
            errorFrames_ = 30; // ~0.5s at 60fps
        }
    }

    // ── Navigation ────────────────────────────────────────────────────────

    // Move selection up (delta=-1) or down (delta=+1), skipping headers.
    void moveSelection(int delta) {
        int n = static_cast<int>(displayList_.size());
        if (n == 0 || !hasSettings()) return;
        int idx = selectedDisplayIdx_;
        for (int attempts = 0; attempts < n; ++attempts) {
            idx += delta;
            if (idx < 0) idx = n - 1;
            if (idx >= n) idx = 0;
            if (!displayList_[idx].isHeader()) {
                selectedDisplayIdx_ = idx;
                return;
            }
        }
    }

    // Ensure selectedDisplayIdx_ points to a non-header, or find the nearest.
    void snapSelection() {
        int n = static_cast<int>(displayList_.size());
        if (n == 0 || !hasSettings()) {
            selectedDisplayIdx_ = 0;
            return;
        }
        if (selectedDisplayIdx_ >= n)
            selectedDisplayIdx_ = n - 1;
        if (selectedDisplayIdx_ < 0)
            selectedDisplayIdx_ = 0;
        if (displayList_[selectedDisplayIdx_].isHeader())
            moveSelection(1);
    }

    // ── Filter / display list ─────────────────────────────────────────────

    // Rebuild the display list from the current input buffer.
    // Groups settings by their .group field and inserts header entries.
    // Search matches against both setting name and group name (case-insensitive).
    void updateFilter() {
        displayList_.clear();

        // Collect groups in registration order, preserving first-seen ordering.
        struct GroupInfo {
            std::string name;
            std::vector<int> settingIndices;
        };
        std::vector<GroupInfo> groups;

        auto findOrCreateGroup = [&](const std::string &gname) -> GroupInfo& {
            for (auto &g : groups)
                if (g.name == gname) return g;
            groups.push_back({gname, {}});
            return groups.back();
        };

        for (int i = 0; i < static_cast<int>(settings_.size()); ++i) {
            findOrCreateGroup(settings_[i].group).settingIndices.push_back(i);
        }

        // For each group, find matching settings and build display list
        for (const auto &group : groups) {
            bool groupMatches = !inputBuf_.empty() && !group.name.empty() &&
                                containsCI(group.name, inputBuf_);

            std::vector<int> matches;
            for (int si : group.settingIndices) {
                if (inputBuf_.empty() || groupMatches ||
                    containsCI(settings_[si].name, inputBuf_)) {
                    matches.push_back(si);
                }
            }

            if (matches.empty()) continue;

            // Insert group header (if group has a name)
            if (!group.name.empty()) {
                DisplayEntry hdr;
                hdr.settingIdx = -1;
                hdr.groupName = group.name;
                displayList_.push_back(std::move(hdr));
            }

            // Insert matching settings
            for (int si : matches) {
                DisplayEntry entry;
                entry.settingIdx = si;
                displayList_.push_back(std::move(entry));
            }
        }

        snapSelection();
    }

    // Compute scroll offset so the selected item is always visible
    static int computeScrollOffset(int total, int visible, int selected) {
        if (total <= visible) return 0;
        // Keep selected item centered when possible
        int offset = selected - visible / 2;
        if (offset < 0) offset = 0;
        if (offset > total - visible) offset = total - visible;
        return offset;
    }

    // Draw a horizontal separator line using box-drawing character
    static void drawSeparator(uint16_t x, uint16_t y, int width) {
        char buf[128];
        int w = std::min(width, static_cast<int>(sizeof(buf) - 1));
        std::memset(buf, '-', w);
        buf[w] = '\0';
        bgfx::dbgTextPrintf(x, y, 0x07, "%s", buf);
    }
};

} // namespace Darkness
