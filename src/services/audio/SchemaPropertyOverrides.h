/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 3 of the License, or
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

// Pure transforms that overlay on-disk schema property records onto a
// SchemaEntry. Used by AudioService::loadSchemaPropertyOverrides() at
// mission-load time. Factored out so the byte-level mapping is testable
// without standing up the full service stack.

#pragma once

#include <cstdint>
#include <cstring>

#include "audio/SchemaTypes.h"
#include "property/DarkPropertyDefs.h"

namespace Darkness {

// Map the on-disk audio-class encoding (1-based: 1=Noise, 2=Speech, ...,
// 10=Monsters; 0=None means "not set") to our internal 0-based
// SchemaAudioClass enum. Returns true and writes the mapped class on
// success; returns false for 0 (no override) or out-of-range values.
inline bool mapDiskAudioClass(uint16_t diskClass, SchemaAudioClass &out)
{
    if (diskClass == 0)
        return false;  // "None" — leave the SchemaEntry's existing class
    if (diskClass > 10)
        return false;
    out = static_cast<SchemaAudioClass>(diskClass - 1);
    return true;
}

// Apply a 20-byte P$SchPlayPa record to `sch`. Overwrites flags,
// audio-class (when non-zero on disk), volume, pan, delay, fade. The
// on-disk `flags` field's bit 12 = SFXFLG_SHARP — same bit as our
// SCH_SHARP_FALLOFF so the value carries through verbatim.
inline void applySchPlayParams(SchemaEntry &sch, const uint8_t *bytes)
{
    PropSchemaPlayParams pp;
    std::memcpy(&pp, bytes, sizeof(pp));

    sch.playParams.flags = static_cast<uint32_t>(pp.flags);

    SchemaAudioClass mapped;
    if (mapDiskAudioClass(pp.audioClass, mapped))
        sch.playParams.audioClass = mapped;

    sch.playParams.volume       = pp.volume;
    sch.playParams.pan          = pp.pan;
    sch.playParams.initialDelay = static_cast<int>(pp.delay);
    sch.playParams.fade         = pp.fade;

    sch.playParams.fieldsSet |=
        SCH_SET_VOLUME | SCH_SET_DELAY | SCH_SET_PAN |
        SCH_SET_FADE | SCH_SET_AUDIO_CLASS;
}

// Apply an 8-byte P$SchLoopPa record. Bit 0 of `flags` is Poly; bit 1
// is Auto-Halt (carried in flags semantics but not modeled in
// SchemaLoopParams yet). A non-zero loop count / interval / max-samples
// implies the schema loops, so we set isLooping accordingly — many
// archetypes declare looping only via this property.
inline void applySchLoopParams(SchemaEntry &sch, const uint8_t *bytes)
{
    PropSchemaLoopParams lp;
    std::memcpy(&lp, bytes, sizeof(lp));

    bool poly = (lp.flags & 0x01) != 0;
    bool hasLoopData = poly ||
                       lp.maxSamples > 1 ||
                       lp.loopCount != 0 ||
                       lp.minInterval != 0 ||
                       lp.maxInterval != 0;
    if (hasLoopData)
        sch.loopParams.isLooping = true;
    sch.loopParams.isPoly = poly;
    sch.loopParams.maxSamples = lp.maxSamples > 0 ? lp.maxSamples : 1;
    sch.loopParams.count = static_cast<uint16_t>(
        lp.loopCount < 0 ? 0 : lp.loopCount);
    sch.loopParams.intervalMin = static_cast<uint16_t>(
        lp.minInterval < 0 ? 0 : lp.minInterval);
    sch.loopParams.intervalMax = static_cast<uint16_t>(
        lp.maxInterval < 0 ? 0 : lp.maxInterval);
}

// Apply a 4-byte P$SchPriori record. Many ambient schemas use the
// default 128; Karras scripted-speech schemas in MISS6 raise priority
// to 255 so they win arbitration against ordinary sounds.
inline void applySchPriority(SchemaEntry &sch, const uint8_t *bytes)
{
    PropSchPriori pri;
    std::memcpy(&pri, bytes, sizeof(pri));
    sch.playParams.priority = pri.priority;
    sch.playParams.fieldsSet |= SCH_SET_PRIORITY;
}

// Apply a 16-byte P$SchMsg record. The label is a NUL-terminated
// char[16]; the AI awareness system fires this label as a script
// message when the schema triggers (e.g. break_glass → "gotonoise").
inline void applySchMessage(SchemaEntry &sch, const uint8_t *bytes)
{
    PropSchMsg m;
    std::memcpy(&m, bytes, sizeof(m));
    size_t n = ::strnlen(m.label, sizeof(m.label));
    sch.message.assign(m.label, n);
}

} // namespace Darkness
