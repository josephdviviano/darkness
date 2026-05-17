/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *    Copyright (C) 2024-2026 darkness contributors
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

// AudioLog.h — Verbosity gate for audio/sound/schema logging
//
// All audio-related fprintf() calls should use AUDIO_LOG() instead.
// Controlled by gAudioLogVerbose (default: false). Toggle at runtime
// via the 'A' key or --audio-log CLI flag.
//
// Dual role: gAudioLogVerbose also gates the audio-thread / loopStep
// performance counters defined at the top of AudioService.cpp.  Reading
// the flag once per audio callback (or per loopStep iteration) lets us
// skip ~11 atomic RMWs in the hot path when profiling is disabled.
// Enabling logging therefore also re-enables those counters; this is
// intentional — the textual `[Audio] perf=` dump that consumes them
// is itself an AUDIO_LOG() output, so users who want the perf line
// already have audio logging on.  See the perf-counter block in
// AudioService.cpp for details.

#pragma once

#include <cstdio>

namespace Darkness {

/// Global audio log verbosity flag. When false, AUDIO_LOG() calls are suppressed.
/// Toggle at runtime (e.g., 'A' key in debug builds) or set via --audio-log CLI flag.
inline bool gAudioLogVerbose = false;

} // namespace Darkness

/// Gate audio logging behind the verbosity flag.
/// Usage: AUDIO_LOG("AudioService: loaded %zu schemas\n", count);
/// Compiles to nothing when gAudioLogVerbose is false (branch predictor friendly).
#define AUDIO_LOG(...)                                          \
    do {                                                        \
        if (::Darkness::gAudioLogVerbose)                       \
            std::fprintf(stderr, __VA_ARGS__);                  \
    } while (0)
