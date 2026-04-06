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
