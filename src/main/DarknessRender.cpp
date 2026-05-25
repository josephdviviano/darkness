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

// World geometry viewer with baked lightmaps and object mesh rendering

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <ctime>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <sstream>
#include <unistd.h> // dup2, fileno
#include <sys/stat.h> // stat, S_ISDIR

#include <SDL.h>
#include <SDL_syswm.h>

#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <bx/math.h>

#include "../shaders/embedded_shaders.h"
#include "WRChunkParser.h"
#include "TXListParser.h"
#include "CRFTextureLoader.h"
#include "CRFModelLoader.h"
#include "LightmapAtlas.h"
#include "SpawnFinder.h"
#include "LightingSystem.h"
#include "ObjectPropParser.h"
#include "BinMeshParser.h"
#include "RenderConfig.h"
#include "RayCaster.h"
#include "DebugConsole.h"
#include "RoomDebugViz.h"

// Logging (must be initialized before ServiceManager)
#include "logger.h"
#include "stdlog.h"
#include "ConsoleBackend.h"

// Service stack for typed property access
#include "DarknessServiceManager.h"
#include "config/ConfigService.h"
#include "database/DatabaseService.h"
#include "game/GameService.h"
#include "inherit/InheritService.h"
#include "link/LinkService.h"
#include "link/Relation.h"
#include "loop/LoopService.h"
#include "object/ObjectService.h"
#include "platform/PlatformService.h"
#include "property/PropertyService.h"
#include "room/RoomService.h"
#include "room/Room.h"
#include "room/RoomPortal.h"
#include "sim/SimService.h"
#include "physics/PhysicsService.h"
#include "audio/AudioLog.h"
#include "audio/AudioService.h"
#include "audio/AcousticMaterials.h"
#include "audio/ProbeFile.h"
#include "audio/ProbeManager.h"
#include "motion/MotionService.h"
#include "RawDataStorage.h"
#include "PLDefParser.h"
#include "DTypeSizeParser.h"
#include "SingleFieldDataStorage.h"
#include "worldquery/ObjSysWorldState.h"
#include "sim/DoorSystem.h"
#include "sim/MovingTerrainSystem.h"
#include "sim/EdgeTriggerSystem.h"
#include "sim/PressurePlateSystem.h"
#include "sim/TweqSystem.h"
#include "sim/MessageDispatch.h"
#include "sim/ScriptManager.h"
#include "sim/IScriptServices.h"
#include "sim/ScriptServices.h"
#include "sim/ObjectPushSystem.h"
#include "StdDoor.h"
#include "StdLever.h"
#include "StdElevator.h"
#include "StdTrap.h"
#include "Triggers.h"
#include "AnimLightScript.h"
#include "SoundScripts.h"
#include "GameLogic.h"
#include "FrobSystem.h"
#include "GrabSystem.h"
#include "FunctionalLoopClient.h"

// TODO: Make these configurable via command-line or config file
static const int WINDOW_WIDTH  = 1280;
static const int WINDOW_HEIGHT = 720;

// ── Renderer modules ──
// Core: vertex formats, texture pipeline, camera, collision, culling, fog data
// Extended: mesh builders, water system, sky, fog parsing, fallback cube
// State: data-holding structs for renderer decomposition
#include "DarknessRendererCore.h"
#include "DarknessRendererExtended.h"
#include "DarknessRenderState.h"

using namespace Darkness;

#include "DarknessRenderInit.h"

// ── Head / viewport log ──────────────────────────────────────────────────
// Per-render-frame CSV capturing what the camera actually renders, plus
// underlying physics-side pose/spring state. Sampled at display rate so the
// interpolated eye position (between fixed steps) is visible and any
// render-rate discontinuities show up. Complementary to the fixed-step
// physics_log written by PlayerPhysics::writeLogRow().

static uint64_t monotonicMicros() {
    using clk = std::chrono::steady_clock;
    return std::chrono::duration_cast<std::chrono::microseconds>(
        clk::now().time_since_epoch()).count();
}

static void openHeadLog(Darkness::RuntimeState &state, const std::string &path) {
    if (state.headLog) std::fclose(state.headLog);
    state.headLog = std::fopen(path.c_str(), "w");
    if (!state.headLog) {
        std::fprintf(stderr, "[head-log] Failed to open %s\n", path.c_str());
        return;
    }
    std::fprintf(state.headLog,
        "wallTime,frameDt,physSteps,ppfCancels,mode,hSpeed,inputFwd,inputRight,onGround,mantling,"
        "camX,camY,camZ,camYaw,camPitch,camRoll,"
        "bodyX,bodyY,bodyZ,"
        "velX,velY,velZ,"
        "springX,springY,springZ,"
        "poseCurX,poseCurY,poseCurZ,"
        "poseEndX,poseEndY,poseEndZ,"
        "headClampX,headClampY,headClampZ,"
        "prevEyeX,prevEyeY,prevEyeZ,"
        "rawEyeX,rawEyeY,rawEyeZ,"
        "knockX,knockY,knockZ,"
        "punchPitch,punchYaw,punchRoll,"
        "groundObjID,"
        "leanAmount,interpAlpha\n");
    state.headLogStartUs   = monotonicMicros();
    state.headLogPrevUs    = state.headLogStartUs;
    state.headLogPrevSteps = state.physics
        ? state.physics->getPlayerPhysics().getTotalFixedSteps() : 0;
    state.headLogPrevPPF   = state.physics
        ? state.physics->getPlayerPhysics().getPPFCancels() : 0;
    state.headLogFlushCtr  = 0;
    std::fprintf(stderr, "[head-log] Writing per-frame viewport log to %s\n", path.c_str());
}

static void closeHeadLog(Darkness::RuntimeState &state) {
    if (state.headLog) {
        std::fclose(state.headLog);
        state.headLog = nullptr;
        std::fprintf(stderr, "[head-log] Closed\n");
    }
}

static void writeHeadLogRow(Darkness::RuntimeState &state) {
    if (!state.headLog || !state.physics) return;
    auto &player = state.physics->getPlayerPhysics();

    uint64_t nowUs = monotonicMicros();
    double wallTime = (nowUs - state.headLogStartUs) * 1e-6;
    double frameDt  = (nowUs - state.headLogPrevUs)  * 1e-6;
    uint64_t totalSteps   = player.getTotalFixedSteps();
    uint32_t stepsThisFrame = static_cast<uint32_t>(totalSteps - state.headLogPrevSteps);
    uint64_t totalPPF     = player.getPPFCancels();
    uint32_t ppfThisFrame = static_cast<uint32_t>(totalPPF - state.headLogPrevPPF);
    state.headLogPrevUs    = nowUs;
    state.headLogPrevSteps = totalSteps;
    state.headLogPrevPPF   = totalPPF;

    const Darkness::Vector3 &spring   = player.getSpringPos();
    const Darkness::Vector3 &poseCur  = player.getPoseCurrent();
    const Darkness::Vector3 &poseEnd  = player.getPoseEnd();
    const Darkness::Vector3 &body     = player.getPosition();
    const Darkness::Vector3 &vel      = player.getVelocity();
    const Darkness::Vector3 &clamp    = player.getHeadClamp();
    const Darkness::Vector3 &prevEye  = player.getPrevEyePos();
    const Darkness::Vector3  rawEye   = player.getRawEyePos();
    const Darkness::Vector3 &knock    = player.getPendingKnockback();
    const Darkness::Vector3 &punch    = player.getViewPunch();

    std::fprintf(state.headLog,
        "%.6f,%.6f,%u,%u,%s,%.4f,%.2f,%.2f,%d,%d,"
        "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"
        "%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,"
        "%d,"
        "%.5f,%.5f\n",
        wallTime, frameDt, stepsThisFrame, ppfThisFrame,
        Darkness::PlayerPhysics::modeName(player.getMode()),
        player.getHorizontalSpeed(),
        player.getInputForward(), player.getInputRight(),
        player.isOnGround() ? 1 : 0, player.isMantling() ? 1 : 0,
        state.cam.pos[0], state.cam.pos[1], state.cam.pos[2],
        state.cam.yaw, state.cam.pitch, state.cam.roll,
        body.x, body.y, body.z,
        vel.x, vel.y, vel.z,
        spring.x, spring.y, spring.z,
        poseCur.x, poseCur.y, poseCur.z,
        poseEnd.x, poseEnd.y, poseEnd.z,
        clamp.x, clamp.y, clamp.z,
        prevEye.x, prevEye.y, prevEye.z,
        rawEye.x, rawEye.y, rawEye.z,
        knock.x, knock.y, knock.z,
        punch.x, punch.y, punch.z,
        player.getGroundObjID(),
        spring.y, player.getInterpAlpha());

    if (++state.headLogFlushCtr >= 30) {
        std::fflush(state.headLog);
        state.headLogFlushCtr = 0;
    }
}

// Serialize the audio.* subset of RenderConfig into a single JSON object
// literal. Consumed by AudioService::openPerfJsonl which embeds it
// verbatim under the run.meta "audio" key — that snapshot is the answer
// to "which knob value produced these percentiles?" (PLAN.AUDIO_PROFILING.md
// §1.2). Keep this list in sync with every audio.* knob in RenderConfig
// AND with the --set dispatch table in RenderConfig.h.
static std::string serializeAudioConfigJson(const Darkness::RenderConfig& c) {
    char buf[8192];
    int n = std::snprintf(buf, sizeof(buf),
        "{"
        "\"performance\":{"
            "\"sample_rate\":%d,\"frame_size\":%d,\"sound_cache_mb\":%d,"
            "\"rate_divisor\":%d,\"max_active_voices\":%d,"
            "\"reverb_voices\":%d,\"reverb_voices_realtime\":%d,"
            "\"reflection_throttle\":%d,\"sim_max_occlusion_samples\":%d,"
            "\"reverb_threads\":%d,\"reverb_threads_conv_share\":%.4f,"
            "\"scene_type\":\"%s\""
        "},"
        "\"reflections\":{"
            "\"enabled\":%s,\"ambisonics_order\":%d,\"bake_skip\":%s,"
            "\"hybrid_transition_time\":%.4f,\"hybrid_overlap_percent\":%.4f,"
            "\"realtime\":{\"rays\":%d,\"bounces\":%d,\"duration\":%.4f,\"diffuse_samples\":%d},"
            "\"bake\":{\"rays\":%d,\"bounces\":%d,\"duration\":%.4f,\"diffuse_samples\":%d,\"ambisonics_order\":%d}"
        "},"
        "\"probes\":{"
            "\"spacing\":%.4f,\"height\":%.4f,"
            "\"min_wall_clearance_ft\":%.4f,\"elevation_sparsity_mul\":%.4f,"
            "\"global_dedup_radius_ft\":%.4f"
        "},"
        "\"pathing_probes\":{\"enabled\":%s,\"dedup_radius_ft\":%.4f,\"force_bake\":%s},"
        "\"occlusion\":{"
            "\"radius\":%.4f,\"samples\":%d,"
            "\"transmission_scale\":%.4f,\"absorption_scale\":%.4f"
        "},"
        "\"propagation\":{"
            "\"portal_routing\":%s,\"probe_pathing\":%s,"
            "\"max_distance\":%.2f,\"door_lpf_open_hz\":%.2f,\"door_lpf_blocked_hz\":%.2f,"
            "\"min_attenuation\":%.5f,\"max_paths\":%u,\"max_path_diff\":%.4f,"
            "\"pathing_gain_scale\":%.4f,\"pathing_blocking_scale\":%.4f,"
            "\"pathing_update_interval\":%.4f,"
            "\"pathing_gain_band_weights\":[%.4f,%.4f,%.4f]"
        "},"
        "\"spatialization\":{"
            "\"hrtf_volume\":%.4f,\"hrtf_interpolation\":\"%s\",\"spatial_blend\":%.4f"
        "},"
        "\"ambient\":{"
            "\"hysteresis_start_mul\":%.4f,\"hysteresis_stop_mul\":%.4f,"
            "\"default_priority\":%d,\"environmental_spatial_blend\":%.4f,"
            "\"global_volume_scale\":%.4f"
        "},"
        "\"mixer\":{"
            "\"master_gain\":%.4f,\"direct_gain\":%.4f,\"reflection_gain\":%.4f,"
            "\"reflection_ramp_ms\":%.4f"
        "},"
        "\"dsp\":{"
            "\"limiter_enabled\":%s,\"limiter_knee\":%.4f,"
            "\"compressor_enabled\":%s,\"compressor_threshold_db\":%.4f,"
            "\"compressor_ratio\":%.4f,\"compressor_attack_ms\":%.4f,"
            "\"compressor_release_ms\":%.4f,"
            "\"eq_enabled\":%s,\"eq_freq_hz\":%.4f,\"eq_gain_db\":%.4f,\"eq_q\":%.4f,"
            "\"ducking_enabled\":%s,\"ducking_amount\":%.4f,"
            "\"ducking_attack_ms\":%.4f,\"ducking_release_ms\":%.4f,"
            "\"wet_saturation_enabled\":%s,\"wet_saturation_drive\":%.4f"
        "}"
        "}",
        c.audioSampleRate, c.audioFrameSize, c.audioSoundCacheMB,
        c.reflectionRateDivisor, c.maxActiveVoices,
        c.reverbVoices, c.reverbVoicesRealtime,
        c.reflectionThrottle, c.simMaxOcclusionSamples,
        c.reverbThreads, c.reverbThreadsConvShare,
        c.sceneType.c_str(),
        c.realtimeReflections ? "true" : "false", c.ambisonicsOrder,
        c.reflectionBakeSkip ? "true" : "false",
        c.hybridTransitionTime, c.hybridOverlapPercent,
        c.realtimeNumRays, c.realtimeNumBounces, c.realtimeDuration, c.realtimeDiffuseSamples,
        c.bakeNumRays, c.bakeNumBounces, c.bakeDuration, c.bakeDiffuseSamples, c.bakeAmbisonicsOrder,
        c.audioProbeSpacingFt, c.audioProbeHeightFt,
        c.audioProbeMinWallClearanceFt, c.audioProbeElevationSparsityMul,
        c.audioProbeGlobalDedupRadiusFt,
        c.audioPathingProbesEnabled ? "true" : "false", c.audioPathingDedupRadiusFt,
        c.forcePathingBake ? "true" : "false",
        c.occlusionRadius, c.occlusionSamples,
        c.transmissionScale, c.absorptionScale,
        c.portalRouting ? "true" : "false", c.probePathing ? "true" : "false",
        c.propagationMaxDist, c.doorLpfOpenHz, c.doorLpfBlockedHz,
        c.propMinAttenuation, c.propMaxPaths, c.propMaxPathDiff,
        c.pathingGainScale, c.pathingBlockingScale,
        c.pathingUpdateInterval,
        c.pathingGainWeightLow, c.pathingGainWeightMid, c.pathingGainWeightHigh,
        c.hrtfVolume, c.hrtfInterpolation.c_str(), c.spatialBlend,
        c.ambHysteresisStartMul, c.ambHysteresisStopMul,
        c.ambDefaultPriority, c.ambEnvironmentalSpatialBlend,
        c.ambGlobalVolumeScale,
        c.mixerMasterGain, c.mixerDirectGain, c.mixerReflectionGain,
        c.reflectionRampMs,
        c.dspLimiter ? "true" : "false", c.dspLimiterKnee,
        c.dspCompressor ? "true" : "false", c.dspCompThreshold,
        c.dspCompRatio, c.dspCompAttackMs, c.dspCompReleaseMs,
        c.dspEQ ? "true" : "false", c.dspEQFreq, c.dspEQGain, c.dspEQQ,
        c.dspDucking ? "true" : "false", c.dspDuckAmount,
        c.dspDuckAttackMs, c.dspDuckReleaseMs,
        c.dspWetSaturation ? "true" : "false", c.dspWetSaturationDrive);
    if (n < 0 || static_cast<size_t>(n) >= sizeof(buf)) {
        // Truncation would corrupt the JSONL stream. Fail loud per
        // feedback_no_silent_fallbacks rather than emit half-baked JSON.
        std::fprintf(stderr,
            "[FALLBACK] serializeAudioConfigJson: buffer too small "
            "(needed %d bytes, have %zu) — emitting empty object\n",
            n, sizeof(buf));
        return std::string("{}");
    }
    return std::string(buf);
}

// Extract the mission filename (no extension, lowercase) for the perf
// artifact directory. e.g. "../levels/MISS6.mis" -> "miss6".
static std::string deriveMissionName(const char* misPath) {
    if (!misPath) return std::string("unknown");
    std::string s(misPath);
    // strip directory
    size_t slash = s.find_last_of("/\\");
    if (slash != std::string::npos) s = s.substr(slash + 1);
    // strip extension
    size_t dot = s.find_last_of('.');
    if (dot != std::string::npos) s = s.substr(0, dot);
    // lowercase for FS portability
    for (auto& ch : s) {
        if (ch >= 'A' && ch <= 'Z') ch = static_cast<char>(ch - 'A' + 'a');
    }
    return s;
}

static void printHelp() {
    // Help text is the canonical reference for every YAML key — it lists every
    // field whether or not the user's local config mentions it. When you add
    // a new YAML knob, also add it here so `--help` stays authoritative.
    std::fprintf(stderr,
        "darknessRender — Dark Engine world geometry + audio viewer\n"
        "\n"
        "USAGE\n"
        "  darknessRender <mission.mis> [--res <path>] [--schemas <path>]\n"
        "                                [--config <path>] [--skip-reflection-bake]\n"
        "                                [--force-pathing-bake]\n"
        "                                [--set <yaml.path>=<value>]\n"
        "                                [--perf-label <name>]\n"
        "                                [--exit-after-seconds <N>]\n"
        "                                [--auto-fly] [--auto-fly-speed <N>]\n"
        "                                [--auto-fly-waypoints <N>]\n"
        "                                [--auto-fly-seed <N>]\n"
        "                                [--auto-fly-pause-sec <N>]\n"
        "                                [-h | --help]\n"
        "\n"
        "  All other tunables live in the YAML config (default: ./darknessRender.yaml).\n"
        "  Run with --help to see every supported config key.\n"
        "\n"
        "CLI FLAGS\n"
        "  <mission.mis>     Path to the .mis file to load (required, positional).\n"
        "  --res <path>      Thief 2 RES directory (fam.crf, obj.crf, snd.crf).\n"
        "                    Overrides paths.res from YAML.\n"
        "  --schemas <path>  Schema directory (.sch / .spc / .arc). Overrides\n"
        "                    paths.schemas; default: search next to RES.\n"
        "  --config <path>   YAML config path. Default: ./darknessRender.yaml.\n"
        "  --skip-reflection-bake\n"
        "                    Carry forward the existing .probes reflection section\n"
        "                    (skip the multi-minute reflection bake; only re-bake\n"
        "                    pathing). Hard-fails if no reflection section exists.\n"
        "                    Overrides YAML audio.reflections.bake_skip.\n"
        "  --force-pathing-bake\n"
        "                    Drop the existing .probes pathing section and re-bake\n"
        "                    it fresh even when a valid pathing section is on disk.\n"
        "                    Symmetric to --skip-reflection-bake; the intended\n"
        "                    composition is:\n"
        "                      --skip-reflection-bake --force-pathing-bake\n"
        "                    which carries reflections forward (skip multi-minute\n"
        "                    bake) but always re-bakes pathing (seconds). This is\n"
        "                    the canonical Sweep 2 Phase B invocation —\n"
        "                    PLAN.AUDIO_PROFILING.md §4.3.\n"
        "  --set <p>=<v>     Generic YAML-path override (repeatable). Applied AFTER\n"
        "                    the YAML load and BEFORE audio init. Supports any\n"
        "                    audio.* leaf — see PLAN.AUDIO_PROFILING.md §1.4.\n"
        "                    Example:\n"
        "                      --set audio.reflections.hybrid_transition_time=0.5\n"
        "                      --set audio.performance.reverb_voices=24\n"
        "                    Unknown paths emit a [FALLBACK] line and are ignored.\n"
        "  --perf-label <name>\n"
        "                    Tag for the per-run audio_perf.jsonl directory:\n"
        "                      ./perf/<mission>/<utc_iso>__<label>/audio_perf.jsonl\n"
        "                    Default: 'default'. Must match [A-Za-z0-9_.-]{1,64}.\n"
        "  --exit-after-seconds <N>\n"
        "                    Exit cleanly after N seconds of wall-clock from main()\n"
        "                    start. Lets `tools/perf_sweep.sh` run unattended. The\n"
        "                    JSONL sink flushes + closes on shutdown.\n"
        "  --auto-fly        Drive the camera through a deterministic random tour\n"
        "                    of the N-nearest pathing probes (forces fly mode).\n"
        "                    Companion to --exit-after-seconds so every sweep\n"
        "                    iteration captures the same flythrough load profile.\n"
        "                    Emits [AUTO_FLY] stderr lines per waypoint reached.\n"
        "                    Falls back to standard fly mode (with [FALLBACK] log)\n"
        "                    when no .probes file is loaded for the mission.\n"
        "  --auto-fly-speed N        Travel speed in world units/sec (default 10).\n"
        "  --auto-fly-waypoints N    N-nearest probes to include (default 50).\n"
        "  --auto-fly-seed N         PRNG seed for visit-order shuffle (default\n"
        "                            0xC0FFEE). Accepts decimal or 0xHEX.\n"
        "  --auto-fly-pause-sec N    Dwell time per waypoint (default 0).\n"
        "  -h, --help        Show this message and exit.\n"
        "\n"
        "YAML CONFIG REFERENCE (defaults shown; see darknessRender.example.yaml)\n"
        "\n"
        "  paths:\n"
        "    res: <path>                     Thief 2 RES directory (CLI --res overrides)\n"
        "    schemas: <path>                 Schema dir (CLI --schemas overrides)\n"
        "\n"
        "  graphics:\n"
        "    texture_filter: point           point | bilinear | trilinear | anisotropic\n"
        "    lightmap_filter: bilinear       bilinear | bicubic\n"
        "    linear_mips: false              gamma-correct mipmap generation\n"
        "    sharp_mips:  false              unsharp mask on mip levels\n"
        "\n"
        "  water:\n"
        "    wave_amplitude: 0.3             vertex Z displacement, 0.0–10.0\n"
        "    uv_distortion:  0.015           UV wobble strength, 0.0–0.1\n"
        "    rotation_speed: 0.015           UV rotation rad/s, 0.0–1.0\n"
        "    scroll_speed:   0.05            UV scroll units/s, 0.0–1.0\n"
        "\n"
        "  physics:\n"
        "    rate: 60                        12=vintage (12.5Hz) | 60=modern | 120=ultra\n"
        "\n"
        "  developer:\n"
        "    show_objects: true              render object meshes\n"
        "    show_fallback_cubes: false      colored cubes for missing models\n"
        "    portal_culling: true            portal/frustum culling\n"
        "    camera_collision: false         clip to world (fly mode only)\n"
        "    step_log: false                 stair-step CSV to stderr\n"
        "    debug_objects: false            per-object filtering diagnostics\n"
        "    toggle_platforms: false         auto-activate moving terrain at startup\n"
        "    no_probes: false                skip Steam Audio probe baking\n"
        "    audio_log: false                audio/sound/schema log output\n"
        "\n"
        "  audio: (eight subsections — see darknessRender.example.yaml for the full set)\n"
        "    audio.performance.*    sample rate, frame size, voice/thread caps, scene type\n"
        "    audio.reflections.*    enabled, rays, bounces, duration, ambisonics_order\n"
        "    audio.occlusion.*      radius, samples, transmission/absorption_scale, diffuse\n"
        "    audio.propagation.*    portal_routing, probe_pathing, max_distance, door LPF\n"
        "    audio.spatialization.* hrtf_volume, hrtf_interpolation, spatial_blend\n"
        "    audio.ambient.*        hysteresis, default_priority, global_volume_scale\n"
        "    audio.mixer.*          master_gain, reflection_gain, reflection_ramp_ms\n"
        "    audio.dsp.*            limiter, compressor, EQ, ducking (per-stage)\n"
        "\n"
        "RUNTIME CONTROLS (in window)\n"
        "  WASD              Move forward/left/back/right\n"
        "  Mouse             Look around\n"
        "  Space/LShift      Move up/down (fly) or jump/crouch (physics mode)\n"
        "  Ctrl              Sprint (3x speed fly / 2x speed walk)\n"
        "  Scroll wheel      Adjust movement speed (shown in title bar)\n"
        "  Home              Teleport to player spawn\n"
        "  ` (backtick)      Open settings console (live-tune any config knob)\n"
        "  Esc               Quit\n"
        "\n"
        "DEBUG CONSOLE (` backtick to open) — selected toggles\n"
        "  texture_filter      Cycle point | bilinear | trilinear | anisotropic\n"
        "  lightmap_filter     Cycle bilinear | bicubic\n"
        "  portal_culling      Toggle portal culling\n"
        "  camera_collision    Toggle camera collision (fly mode)\n"
        "  physics_mode        Toggle fly ↔ walk-with-gravity\n"
        "  refl_enabled        Toggle audio reflections (convolution reverb)\n"
        "  head_log            Write per-frame head/viewport CSV to ./head_log.csv\n"
        "  physics_log         Write physics diagnostics to ./physics_log.csv\n"
        "  auto_fly            Deterministic probe-tour flythrough (forces fly mode)\n"
        "  show_raycast        Toggle raycast debug visualization\n"
        "  isolate_model       Cycle model isolation (debug)\n"
        "\n"
        "RESOURCE SETUP\n"
        "  The RES path must contain fam.crf, obj.crf, snd.crf. Sources:\n"
        "    1. Mounted ISO (macOS):\n"
        "       hdiutil mount ../disk_images/thief_2_disk_1.iso\n"
        "       /Volumes/THIEF2_INSTALL_C/THIEF2/RES\n"
        "    2. GOG/Steam install: /path/to/Thief2/RES\n"
        "    3. Any directory containing the three CRF files.\n"
        "\n"
        "EXAMPLES\n"
        "  # YAML provides paths.res, just point to a mission:\n"
        "  darknessRender path/to/miss6.mis\n"
        "\n"
        "  # One-shot override:\n"
        "  darknessRender path/to/miss6.mis --res /Volumes/THIEF2_INSTALL_C/THIEF2/RES\n"
        "\n"
        "  # Use an alternate config for testing:\n"
        "  darknessRender path/to/miss6.mis --config experiments/dry-room.yaml\n"
    );
}

// ── Render-loop functions ──
// Each function handles one pass of the per-frame render loop.
// Parameters are struct references defined in DarknessRenderState.h.

// Render sky dome or textured skybox into View 0 (no depth write/test).
static void renderSky(
    const Darkness::FrameContext &fc,
    const Darkness::BuiltMeshes &meshes,
    const Darkness::GPUResources &gpu,
    const Darkness::MissionData &mission,
    const Darkness::RuntimeState &state)
{
    float skyView[16];
    state.cam.getSkyViewMatrix(skyView);
    bgfx::setViewTransform(0, skyView, fc.proj);

    float skyModel[16];
    bx::mtxIdentity(skyModel);

    // Cull CCW (not CW) because the camera is inside the cube/sphere —
    // we see the back faces, so cull the outward-facing front faces.
    uint64_t skyState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                      | BGFX_STATE_CULL_CCW;

    // Inline sky fog uniform helper
    float opaqueParams[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    // Sky and other non-object passes still feed shaders that read
    // u_objectLight (fs_basic, fs_textured); pass white so the multiply is
    // a no-op for those passes.
    float whiteLight[4] = { 1.0f, 1.0f, 1.0f, 0.0f };
    auto setFogSky = [&]() {
        bgfx::setUniform(gpu.u_fogColor, fc.fogColorArr);
        bgfx::setUniform(gpu.u_fogParams, fc.skyFogArr);
        bgfx::setUniform(gpu.u_objectParams, opaqueParams);
        bgfx::setUniform(gpu.u_objectLight, whiteLight);
    };

    if (mission.hasSkybox && bgfx::isValid(gpu.skyboxVBH)) {
        // Textured skybox (old sky system) — render each face with its texture
        for (const auto &face : meshes.skyboxCube.faces) {
            auto texIt = gpu.skyboxTexHandles.find(face.key);
            if (texIt == gpu.skyboxTexHandles.end()) continue;

            setFogSky();
            bgfx::setTransform(skyModel);
            bgfx::setVertexBuffer(0, gpu.skyboxVBH);
            bgfx::setIndexBuffer(gpu.skyboxIBH, face.firstIndex, face.indexCount);
            bgfx::setState(skyState);
            bgfx::setTexture(0, gpu.s_texColor, texIt->second, fc.skySampler);
            bgfx::submit(0, gpu.texturedProgram);
        }
    } else if (bgfx::isValid(gpu.skyVBH)) {
        // Procedural dome (new sky system) — vertex-coloured hemisphere
        setFogSky();
        bgfx::setTransform(skyModel);
        bgfx::setVertexBuffer(0, gpu.skyVBH);
        bgfx::setIndexBuffer(gpu.skyIBH);
        bgfx::setState(skyState);
        bgfx::submit(0, gpu.flatProgram);
    }
}

// Render world geometry into View 1 — lightmapped, textured, or flat-shaded.
// Iterates per-cell draw groups, skipping cells not in the visible set.
static void renderWorld(
    const Darkness::FrameContext &fc,
    const Darkness::BuiltMeshes &meshes,
    const Darkness::GPUResources &gpu,
    const Darkness::MissionData &mission,
    const Darkness::RuntimeState &state)
{
    float model[16];
    bx::mtxIdentity(model);

    float opaqueParams[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    float whiteLight[4]   = { 1.0f, 1.0f, 1.0f, 0.0f };
    auto setFogOn = [&]() {
        bgfx::setUniform(gpu.u_fogColor, fc.fogColorArr);
        bgfx::setUniform(gpu.u_fogParams, fc.fogOnArr);
        bgfx::setUniform(gpu.u_objectParams, opaqueParams);
        bgfx::setUniform(gpu.u_objectLight, whiteLight);
    };

    auto isCellVisible = [&](uint32_t cellID) -> bool {
        if (!state.portalCulling) return true;
        return fc.visibleCells.count(cellID) > 0;
    };

    if (meshes.lightmappedMode) {
        for (const auto &grp : meshes.lmMesh.groups) {
            if (!isCellVisible(grp.cellID)) continue;

            setFogOn();
            bgfx::setTransform(model);
            bgfx::setVertexBuffer(0, gpu.vbh);
            bgfx::setIndexBuffer(gpu.ibh, grp.firstIndex, grp.numIndices);
            bgfx::setState(fc.renderState);

            if (grp.txtIndex == 0) {
                bgfx::submit(1, gpu.flatProgram);
            } else {
                auto it = gpu.textureHandles.find(grp.txtIndex);
                if (it != gpu.textureHandles.end()) {
                    bgfx::setTexture(0, gpu.s_texColor, it->second, fc.texSampler);
                    if (!gpu.lightmapAtlasHandles.empty())
                        bgfx::setTexture(1, gpu.s_texLightmap, gpu.lightmapAtlasHandles[0]);

                    // Select lightmap program: bilinear (hardware) or bicubic (shader)
                    bgfx::ProgramHandle lmProg = gpu.lightmappedProgram;
                    if (state.lightmapFiltering == 1 && bgfx::isValid(gpu.lightmappedBicubicProgram)) {
                        lmProg = gpu.lightmappedBicubicProgram;
                        // Pass atlas dimensions for texel-space calculations in bicubic shader
                        float sz[4] = { float(gpu.lmAtlasSet.atlases[0].size),
                                        float(gpu.lmAtlasSet.atlases[0].size), 0, 0 };
                        bgfx::setUniform(gpu.u_lmAtlasSize, sz);
                    }
                    bgfx::submit(1, lmProg);
                } else {
                    bgfx::submit(1, gpu.flatProgram);
                }
            }
        }
    } else if (mission.texturedMode) {
        for (const auto &grp : meshes.worldMesh.groups) {
            if (!isCellVisible(grp.cellID)) continue;

            setFogOn();
            bgfx::setTransform(model);
            bgfx::setVertexBuffer(0, gpu.vbh);
            bgfx::setIndexBuffer(gpu.ibh, grp.firstIndex, grp.numIndices);
            bgfx::setState(fc.renderState);

            if (grp.txtIndex == 0) {
                bgfx::submit(1, gpu.flatProgram);
            } else {
                auto it = gpu.textureHandles.find(grp.txtIndex);
                if (it != gpu.textureHandles.end()) {
                    bgfx::setTexture(0, gpu.s_texColor, it->second, fc.texSampler);
                    bgfx::submit(1, gpu.texturedProgram);
                } else {
                    bgfx::submit(1, gpu.flatProgram);
                }
            }
        }
    } else {
        for (const auto &grp : meshes.flatMesh.groups) {
            if (!isCellVisible(grp.cellID)) continue;

            setFogOn();
            bgfx::setTransform(model);
            bgfx::setVertexBuffer(0, gpu.vbh);
            bgfx::setIndexBuffer(gpu.ibh, grp.firstIndex, grp.numIndices);
            bgfx::setState(fc.renderState);
            bgfx::submit(1, gpu.flatProgram);
        }
    }
}

// Render water surfaces into View 1 — alpha-blended, no depth write, double-sided.
// Rendered last so all opaque geometry is already in the depth buffer.
static void renderWater(
    const Darkness::FrameContext &fc,
    const Darkness::BuiltMeshes &meshes,
    const Darkness::GPUResources &gpu,
    const Darkness::RuntimeState &state)
{
    if (!meshes.hasWater) return;

    float identity[16];
    bx::mtxIdentity(identity);

    // Alpha blend, depth test (read) but no depth write, no face culling
    uint64_t waterState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
        | BGFX_STATE_DEPTH_TEST_LESS
        | BGFX_STATE_BLEND_ALPHA;
    // No BGFX_STATE_WRITE_Z — water doesn't occlude geometry behind it
    // No BGFX_STATE_CULL_* — water is visible from both sides

    float opaqueParams[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    float whiteLight[4]   = { 1.0f, 1.0f, 1.0f, 0.0f };
    auto setFogOn = [&]() {
        bgfx::setUniform(gpu.u_fogColor, fc.fogColorArr);
        bgfx::setUniform(gpu.u_fogParams, fc.fogOnArr);
        bgfx::setUniform(gpu.u_objectParams, opaqueParams);
        bgfx::setUniform(gpu.u_objectLight, whiteLight);
    };

    for (const auto &grp : meshes.waterMesh.groups) {
        setFogOn();
        bgfx::setTransform(identity);
        bgfx::setVertexBuffer(0, gpu.waterVBH);
        bgfx::setIndexBuffer(gpu.waterIBH, grp.firstIndex, grp.numIndices);
        bgfx::setState(waterState);

        if (grp.txtIndex != 0 || grp.flowGroup > 0) {
            // Textured water: water shader with vertex displacement + UV distortion
            // u_waterParams: x=elapsed time, y=scroll speed, z=wave amplitude, w=UV distortion
            float wp[4] = { state.waterElapsed, state.waterScrollSpeed, state.waveAmplitude, state.uvDistortion };
            bgfx::setUniform(gpu.u_waterParams, wp);

            // u_waterFlow: x=rotation speed, y=use_world_uv flag, z=tex_unit_len
            // Flow-textured water: vertex shader computes UVs from world position
            // with rotation.
            // TXLIST-textured water: uses pre-computed UVs from mesh.
            bool isFlowTextured = (grp.flowGroup > 0);
            float useWorldUV = isFlowTextured ? 1.0f : 0.0f;
            constexpr float TEX_UNIT_LEN = 4.0f; // 4 world units per texture repeat
            float wf[4] = { state.waterRotation, useWorldUV, TEX_UNIT_LEN, 0.0f };
            bgfx::setUniform(gpu.u_waterFlow, wf);

            // Resolve texture: flow texture by group first, then TXLIST by index
            bgfx::TextureHandle tex = BGFX_INVALID_HANDLE;
            if (grp.flowGroup > 0) {
                auto fit = gpu.flowTextureHandles.find(grp.flowGroup);
                if (fit != gpu.flowTextureHandles.end())
                    tex = fit->second;
            }
            if (!bgfx::isValid(tex) && grp.txtIndex != 0) {
                auto it = gpu.textureHandles.find(grp.txtIndex);
                if (it != gpu.textureHandles.end())
                    tex = it->second;
            }

            if (bgfx::isValid(tex)) {
                bgfx::setTexture(0, gpu.s_texColor, tex, fc.texSampler);
                bgfx::submit(1, gpu.waterProgram);
            } else {
                bgfx::submit(1, gpu.flatProgram);
            }
        } else {
            // Non-textured water: flat blue-green from vertex color
            bgfx::submit(1, gpu.flatProgram);
        }
    }
}

// Indices into state.roomDebug for the closest-N rooms to camPos. If
// debugRoomMaxCount is 0 or >= number of rooms, every room is returned
// (in original order — no sorting wasted). Otherwise we partial-sort by
// squared distance to room center and return the closest N indices in
// nearest-first order.
//
// Used by both renderDebugOverlay (wireframe/portal overlay) and
// renderRoomLabelsOverlay so they agree on which rooms are visible: the
// labels you can read always correspond to the wireframes you can see,
// and vice versa.
static std::vector<size_t> selectClosestRooms(
    const Darkness::RuntimeState &state,
    const Darkness::Vector3 &camPos)
{
    const auto &rd = state.roomDebug;
    std::vector<size_t> indices;
    indices.reserve(rd.size());
    for (size_t i = 0; i < rd.size(); ++i) indices.push_back(i);

    int cap = state.debugRoomMaxCount;
    if (cap <= 0 || static_cast<size_t>(cap) >= rd.size()) {
        return indices;  // no cull
    }

    // Partial sort: we only need the closest `cap`, so nth_element +
    // sort of the head is cheaper than a full sort of the room list.
    std::nth_element(indices.begin(), indices.begin() + cap, indices.end(),
        [&](size_t a, size_t b) {
            float da = glm::length2(rd[a].center - camPos);
            float db = glm::length2(rd[b].center - camPos);
            return da < db;
        });
    indices.resize(cap);
    return indices;
}

// ── Shared pathing-probe layer-2 occlusion classifier ──
//
// Replicates the runtime filter Steam Audio applies before findPaths
// runs: `ProbeNeighborhood::checkOcclusion` (probe_manager.cpp:51-82)
// casts a single ray from the listener to each probe's center against
// the acoustic scene. Probes whose ray hits geometry get dropped from
// the neighborhood and cannot contribute to the wet bus.
//
// We approximate this here client-side using the renderer's CPU copy
// of the acoustic mesh (Möller-Trumbore any-hit). Output is consumed
// by BOTH show_probes (cube tint for pathing probes) and
// show_probe_radius (wireframe sphere color) so the two overlays agree
// per-probe. Single-probe state ordering matches the at-a-glance
// triage colors:
//   Visible    — listener inside sphere AND ray clears mesh (green)
//   Occluded   — listener inside sphere AND ray hits mesh (red)
//   OutOfRange — listener outside influence sphere (gray)
//
// Caveats baked into the implementation:
//   • Door OBBs are NOT in state.acousticVerts/Indices (they're separate
//     IPL instanced meshes on the audio scene). Closed doors won't
//     affect this classifier; that's fine — this is a static-geometry
//     wall-occlusion diagnostic, not a door-state diagnostic.
//   • Single ray per probe matches Steam Audio's actual check exactly,
//     so the classification faithfully reflects runtime behaviour
//     including its single-ray-can-thread-through-seams fragility.
namespace {

enum class ProbeOccState : uint8_t {
    OutOfRange = 0,  ///< Listener outside probe.influence sphere
                     ///< (layer-1 culls before layer-2 runs).
    Visible    = 1,  ///< In sphere AND ray clears → contributes.
    Occluded   = 2,  ///< In sphere AND ray hit → layer-2 drops it.
};

// Möller-Trumbore any-hit, two-sided. Ray from `from` to `to` against
// the triangle soup `aVerts` / `aIdx` (engine feet, Z-up). Hits at
// t ∈ (epsT, length-epsT) — epsilon margin so listener-side and
// probe-side touch points don't self-occlude. Returns true on first
// hit (no closest-tri tracking).
bool rayHitsAcousticMesh(const Darkness::Vector3 &from,
                         const Darkness::Vector3 &to,
                         const float *aVerts,
                         const int32_t *aIdx,
                         size_t numTris)
{
    const float dx = to.x - from.x;
    const float dy = to.y - from.y;
    const float dz = to.z - from.z;
    const float maxT = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (maxT < 1e-3f) return false;
    const float invLen = 1.0f / maxT;
    const float rdx = dx * invLen;
    const float rdy = dy * invLen;
    const float rdz = dz * invLen;
    constexpr float kEps = 1e-3f;
    const float epsT = kEps;
    const float testMax = maxT - kEps;
    if (testMax <= epsT) return false;
    for (size_t t = 0; t < numTris; ++t) {
        int32_t i0 = aIdx[t*3 + 0];
        int32_t i1 = aIdx[t*3 + 1];
        int32_t i2 = aIdx[t*3 + 2];
        float ax = aVerts[i0*3+0], ay = aVerts[i0*3+1], az = aVerts[i0*3+2];
        float bx = aVerts[i1*3+0], by = aVerts[i1*3+1], bz = aVerts[i1*3+2];
        float cx = aVerts[i2*3+0], cy = aVerts[i2*3+1], cz = aVerts[i2*3+2];
        float e1x = bx - ax, e1y = by - ay, e1z = bz - az;
        float e2x = cx - ax, e2y = cy - ay, e2z = cz - az;
        float px = rdy*e2z - rdz*e2y;
        float py = rdz*e2x - rdx*e2z;
        float pz = rdx*e2y - rdy*e2x;
        float det = e1x*px + e1y*py + e1z*pz;
        if (std::fabs(det) < 1e-8f) continue;
        float invDet = 1.0f / det;
        float tx = from.x - ax;
        float ty = from.y - ay;
        float tz = from.z - az;
        float u = (tx*px + ty*py + tz*pz) * invDet;
        if (u < 0.0f || u > 1.0f) continue;
        float qx = ty*e1z - tz*e1y;
        float qy = tz*e1x - tx*e1z;
        float qz = tx*e1y - ty*e1x;
        float v = (rdx*qx + rdy*qy + rdz*qz) * invDet;
        if (v < 0.0f || u + v > 1.0f) continue;
        float hitT = (e2x*qx + e2y*qy + e2z*qz) * invDet;
        if (hitT > epsT && hitT < testMax) return true;
    }
    return false;
}

// Classify a single pathing probe relative to the listener.
ProbeOccState classifyPathingProbe(const Darkness::Vector3 &listener,
                                    const Darkness::Vector3 &probePos,
                                    float probeRadiusFt,
                                    const float *aVerts,
                                    const int32_t *aIdx,
                                    size_t numTris)
{
    Darkness::Vector3 d = probePos - listener;
    float distSq = d.x*d.x + d.y*d.y + d.z*d.z;
    float rSq    = probeRadiusFt * probeRadiusFt;
    if (distSq > rSq)
        return ProbeOccState::OutOfRange;
    if (rayHitsAcousticMesh(listener, probePos, aVerts, aIdx, numTris))
        return ProbeOccState::Occluded;
    return ProbeOccState::Visible;
}

// State → ABGR uint32 for the vertex-color sphere shader.
uint32_t probeStateToABGR(ProbeOccState s) {
    switch (s) {
        case ProbeOccState::Visible:    return 0xFF22FF22;  // green
        case ProbeOccState::Occluded:   return 0xFF2222FF;  // red
        case ProbeOccState::OutOfRange: return 0x80808080;  // dim gray
    }
    return 0xFFFFFFFF;
}

// State → RGBA float[4] for the flat-shaded cube uniform u_objectLight.
// Alpha is unused by the flat program (set to 0 to match existing
// tints in this file).
void probeStateToRGBA(ProbeOccState s, float out[4]) {
    out[3] = 0.0f;
    switch (s) {
        case ProbeOccState::Visible:
            out[0] = 0.13f; out[1] = 1.00f; out[2] = 0.13f; break;
        case ProbeOccState::Occluded:
            out[0] = 1.00f; out[1] = 0.13f; out[2] = 0.13f; break;
        case ProbeOccState::OutOfRange:
            out[0] = 0.50f; out[1] = 0.50f; out[2] = 0.50f; break;
    }
}

} // anonymous namespace

// Render the room-ID overlay (HUD line + per-corner labels). Called
// from the main render loop AFTER the frob-hint section so the labels
// survive the dbgTextClear at the top of that section. Wireframe edges
// for show_rooms are drawn separately inside renderDebugOverlay — only
// the text portion lives here.
static void renderRoomLabelsOverlay(
    const Darkness::FrameContext &fc,
    const Darkness::RuntimeState &state)
{
    if (!state.showRooms) return;

    Darkness::RoomServicePtr roomSvc = GET_SERVICE(Darkness::RoomService);
    Darkness::Room *currentRoom = nullptr;
    // Collect every room whose raw OBB contains the camera position, plus
    // the closest room by center distance. The "current cam room" is the
    // canonical (disambiguator-resolved) answer; the OBB list reveals
    // whether the cam sits inside multiple overlapping OBBs (expected at
    // portal boundaries) or no OBB at all (a gap, in which case the
    // closest-room readout tells you which side you're nearer to).
    Vector3 camPos(state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
    constexpr int kMaxObbHits = 8;
    int16_t obbHits[kMaxObbHits];
    int     numObbHits = 0;
    int16_t closestRoomID = -1;
    float   closestDist   = std::numeric_limits<float>::infinity();
    if (roomSvc && roomSvc->isLoaded()) {
        currentRoom = roomSvc->roomFromPoint(camPos);
        for (const auto &rp : roomSvc->getAllRooms()) {
            if (!rp) continue;
            if (rp->obbContains(camPos) && numObbHits < kMaxObbHits) {
                obbHits[numObbHits++] = rp->getRoomID();
            }
            float d = glm::length(rp->getCenter() - camPos);
            if (d < closestDist) {
                closestDist = d;
                closestRoomID = rp->getRoomID();
            }
        }
    }
    int16_t currentRoomID = currentRoom ? currentRoom->getRoomID() : -1;

    // HUD line at top-left. Three shapes:
    //   "current cam room: 217"                        — sole OBB match
    //   "current cam room: 217 (OBBs: 217, 219)"       — disambiguator picked
    //                                                    one of multiple overlapping OBBs
    //   "current cam room: Null (217 closest, d=0.6)"  — no OBB contains
    //                                                    camPos
    char detailBuf[160];
    if (numObbHits == 0) {
        if (closestRoomID >= 0) {
            std::snprintf(detailBuf, sizeof(detailBuf),
                "Null (%d closest, d=%.1f)", closestRoomID, closestDist);
        } else {
            std::snprintf(detailBuf, sizeof(detailBuf), "Null (no rooms loaded)");
        }
    } else if (numObbHits == 1) {
        std::snprintf(detailBuf, sizeof(detailBuf), "%d", currentRoomID);
    } else {
        // List every OBB hit, marking the disambiguator's pick with '*'.
        char list[128];
        size_t off = 0;
        for (int i = 0; i < numObbHits; ++i) {
            int written = std::snprintf(list + off, sizeof(list) - off,
                "%s%d%s",
                (i == 0 ? "" : ", "),
                obbHits[i],
                (obbHits[i] == currentRoomID ? "*" : ""));
            if (written < 0 || static_cast<size_t>(written) >= sizeof(list) - off) break;
            off += static_cast<size_t>(written);
        }
        std::snprintf(detailBuf, sizeof(detailBuf),
            "%d (OBBs: %s)", currentRoomID, list);
    }
    bgfx::dbgTextPrintf(2, 1, 0x0F,
        "ROOM DEBUG — (`%s) current cam room: %s",
        "show_rooms", detailBuf);

    // Project each room label into screen space and place at the
    // appropriate dbgText character cell.
    const bgfx::Stats *stats = bgfx::getStats();
    if (!stats || stats->width <= 0 || stats->height <= 0) return;
    float fbWidth  = static_cast<float>(stats->width);
    float fbHeight = static_cast<float>(stats->height);
    constexpr float cellW = 8.0f;
    constexpr float cellH = 16.0f;

    float vp[16];
    bx::mtxMul(vp, fc.view, fc.proj);

    // Apply the same closest-N cull as the wireframe overlay so the
    // labels visible on-screen match the wireframes drawn behind them.
    // Without this, labels for distant rooms would still print on top
    // of nearby wireframes and re-introduce all the clutter the cull
    // was meant to remove.
    Darkness::Vector3 cullCamPos(state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
    auto picks = selectClosestRooms(state, cullCamPos);

    for (size_t idx : picks) {
        for (const auto &lbl : state.roomDebug[idx].labels) {
            float p[4] = { lbl.pos.x, lbl.pos.y, lbl.pos.z, 1.0f };
            float c[4];
            bx::vec4MulMtx(c, p, vp);
            if (c[3] <= 0.001f) continue;  // behind camera
            float ndcX = c[0] / c[3];
            float ndcY = c[1] / c[3];
            if (ndcX < -1.0f || ndcX > 1.0f) continue;
            if (ndcY < -1.0f || ndcY > 1.0f) continue;

            float sx = (ndcX * 0.5f + 0.5f) * fbWidth;
            float sy = (1.0f - (ndcY * 0.5f + 0.5f)) * fbHeight;
            int col = static_cast<int>(sx / cellW);
            int row = static_cast<int>(sy / cellH);
            if (col < 0 || row < 0) continue;

            bool isCurrent = (lbl.roomID == currentRoomID);
            // Attribute byte: high nibble = bg, low nibble = fg.
            //   Current room (center): bright yellow on black with "*"
            //   Other rooms (center):  bright cyan on black
            //   Corner labels:         dim cyan on black (less visual noise —
            //     8 corners per room — but legible from a few feet away).
            uint8_t attr;
            if (isCurrent && lbl.isCenter) {
                attr = 0x0E;
            } else if (lbl.isCenter) {
                attr = 0x0B;
            } else {
                attr = 0x03;
            }
            bgfx::dbgTextPrintf(col, row, attr, "%d%s",
                lbl.roomID, (isCurrent && lbl.isCenter) ? "*" : "");
        }
    }
}

// Render debug raycast visualization + HUD text into View 2.
// Casts a ray from camera forward, draws cross at hit point, normal line,
// and HUD text overlay with hit details.
static void renderDebugOverlay(
    const Darkness::FrameContext &fc,
    const Darkness::GPUResources &gpu,
    const Darkness::MissionData &mission,
    const Darkness::RuntimeState &state)
{
    // Skip view-state setup if nothing in this overlay is enabled. The probe
    // overlay covers both the grid and the listener-flash marker; we only
    // pay the AudioService GET_SERVICE call when it's on.
    //
    // EVERY show_* boolean that gets drawn inside this function must
    // appear here, otherwise the wireframe silently requires another
    // overlay to be on first — the exact bug this gate was supposed to
    // prevent. show_portals and show_vpos were added later and missed
    // the list.
    if (!state.showRaycast && !state.showAcousticMesh
        && !state.showAcousticHit
        && !state.showProbes && !state.showRooms
        && !state.showPortals && !state.showVPos
        && !state.showProbeRadius) {
        return;
    }

    // Set up view 2 with same transform as view 1
    bgfx::setViewTransform(2, fc.view, fc.proj);

    // Enable bgfx debug-text mode once for the whole function, so any
    // sub-overlay (raycast HUD, room ID labels, future debug text) can
    // dbgTextPrintf into the framebuffer. Previously this was buried
    // inside the `if (state.showRaycast)` branch, which meant any other
    // overlay that wanted text was invisible unless show_raycast was
    // also on.
    bgfx::setDebug(BGFX_DEBUG_TEXT);
    bgfx::dbgTextClear();

    // ── Raycast debug lines ──
    if (state.showRaycast) {
    // Compute camera forward direction (same as Camera::getViewMatrix)
    float cosPitch = std::cos(state.cam.pitch);
    float fwdX = std::cos(state.cam.yaw) * cosPitch;
    float fwdY = std::sin(state.cam.yaw) * cosPitch;
    float fwdZ = std::sin(state.cam.pitch);

    // Ray from camera position, extending forward up to 500 world units
    constexpr float RAY_VIS_DIST = 500.0f;
    Darkness::Vector3 rayFrom(state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
    Darkness::Vector3 rayTo(state.cam.pos[0] + fwdX * RAY_VIS_DIST,
                            state.cam.pos[1] + fwdY * RAY_VIS_DIST,
                            state.cam.pos[2] + fwdZ * RAY_VIS_DIST);

    Darkness::RayHit rayHit;
    bool rayDidHit = Darkness::raycastWorld(mission.wrData, rayFrom, rayTo, rayHit);

    // Draw debug lines using bgfx transient vertex buffers.
    // Hit: normal (2) + cross (6) + offset ray (2) = 10.  Miss: ray (2).
    constexpr uint32_t MAX_DEBUG_VERTS = 10;
    bgfx::TransientVertexBuffer tvb;
    if (bgfx::getAvailTransientVertexBuffer(MAX_DEBUG_VERTS, Darkness::PosColorVertex::layout) >= MAX_DEBUG_VERTS) {
        bgfx::allocTransientVertexBuffer(&tvb, MAX_DEBUG_VERTS, Darkness::PosColorVertex::layout);
        auto *verts = reinterpret_cast<Darkness::PosColorVertex *>(tvb.data);
        uint32_t numVerts = 0;

        // Helper: emit one debug vertex
        auto addVert = [&](float vx, float vy, float vz, uint32_t color) {
            verts[numVerts].x = vx;
            verts[numVerts].y = vy;
            verts[numVerts].z = vz;
            verts[numVerts].abgr = color;
            ++numVerts;
        };

        // Colors (ABGR format for bgfx)
        uint32_t green  = 0xFF00FF00;  // hit ray segment
        uint32_t yellow = 0xFF00FFFF;  // miss ray segment
        uint32_t cyan   = 0xFFFFFF00;  // surface normal
        uint32_t red    = 0xFF0000FF;  // hit marker cross

        if (rayDidHit) {
            // Cyan line: surface normal at hit point (4 unit length)
            constexpr float NORM_LEN = 4.0f;
            addVert(rayHit.point.x, rayHit.point.y, rayHit.point.z, cyan);
            addVert(rayHit.point.x + rayHit.normal.x * NORM_LEN,
                    rayHit.point.y + rayHit.normal.y * NORM_LEN,
                    rayHit.point.z + rayHit.normal.z * NORM_LEN, cyan);

            // Red cross at hit point (3 axes, 2 unit half-size — visible at distance)
            constexpr float CROSS = 2.0f;
            addVert(rayHit.point.x - CROSS, rayHit.point.y, rayHit.point.z, red);
            addVert(rayHit.point.x + CROSS, rayHit.point.y, rayHit.point.z, red);
            addVert(rayHit.point.x, rayHit.point.y - CROSS, rayHit.point.z, red);
            addVert(rayHit.point.x, rayHit.point.y + CROSS, rayHit.point.z, red);
            addVert(rayHit.point.x, rayHit.point.y, rayHit.point.z - CROSS, red);
            addVert(rayHit.point.x, rayHit.point.y, rayHit.point.z + CROSS, red);

            // Green line: from hit point pulled back slightly toward camera,
            // offset perpendicular so it's not collinear with the view ray.
            // This gives a visible green "approach" line near the hit.
            constexpr float PULLBACK = 5.0f;
            constexpr float OFFSET = 0.3f;
            // Camera right vector for perpendicular offset
            float rtX = std::sin(state.cam.yaw);
            float rtY = -std::cos(state.cam.yaw);
            addVert(rayHit.point.x - fwdX * PULLBACK + rtX * OFFSET,
                    rayHit.point.y - fwdY * PULLBACK + rtY * OFFSET,
                    rayHit.point.z - fwdZ * PULLBACK, green);
            addVert(rayHit.point.x, rayHit.point.y, rayHit.point.z, green);
        } else {
            // Yellow line: camera → ray end (visible against sky/void)
            addVert(state.cam.pos[0], state.cam.pos[1], state.cam.pos[2], yellow);
            addVert(rayTo.x, rayTo.y, rayTo.z, yellow);
        }

        // Submit to view 2 (debug overlay) — no depth test so lines
        // render on top of world geometry
        float identity[16];
        bx::mtxIdentity(identity);
        bgfx::setTransform(identity);
        bgfx::setVertexBuffer(0, &tvb, 0, numVerts);
        uint64_t lineState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                           | BGFX_STATE_PT_LINES;
        bgfx::setState(lineState);
        float opaqueParams[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
        float whiteLight[4]   = { 1.0f, 1.0f, 1.0f, 0.0f };
        bgfx::setUniform(gpu.u_fogColor, fc.fogColorArr);
        bgfx::setUniform(gpu.u_fogParams, fc.fogOnArr);
        bgfx::setUniform(gpu.u_objectParams, opaqueParams);
        bgfx::setUniform(gpu.u_objectLight, whiteLight);
        bgfx::submit(2, gpu.flatProgram);
    }
    // (Debug-text mode and dbgTextClear were already set up at the top
    // of renderDebugOverlay so every show_* overlay can dbgTextPrintf,
    // not only show_raycast. Acoustic-mesh and room-wireframe overlays
    // are submitted OUTSIDE this if-block so they aren't gated on
    // show_raycast — see after `} // end showRaycast` below.)

    // Screen-center crosshair (160 cols x 45 rows at 1280x720)
    uint8_t cross_attr = 0x0E; // yellow on black
    bgfx::dbgTextPrintf(79, 22, cross_attr, "+");

    uint8_t hud_attr = 0x0F; // white on black
    uint8_t val_attr = 0x0A; // green on black

    bgfx::dbgTextPrintf(2, 1, hud_attr, "RAYCAST DEBUG (` > show_raycast to toggle)");

    if (rayDidHit) {
        bgfx::dbgTextPrintf(2, 3, val_attr, "Hit:     YES");
        bgfx::dbgTextPrintf(2, 4, val_attr, "Dist:    %.2f", rayHit.distance);
        bgfx::dbgTextPrintf(2, 5, val_attr, "Point:   (%.2f, %.2f, %.2f)",
            rayHit.point.x, rayHit.point.y, rayHit.point.z);
        bgfx::dbgTextPrintf(2, 6, val_attr, "Normal:  (%.3f, %.3f, %.3f)",
            rayHit.normal.x, rayHit.normal.y, rayHit.normal.z);
        bgfx::dbgTextPrintf(2, 7, val_attr, "TexIdx:  %d",
            rayHit.textureIndex);
        // Show texture name if available from TXLIST
        if (rayHit.textureIndex >= 0 &&
            static_cast<size_t>(rayHit.textureIndex) < mission.txList.textures.size()) {
            const auto &tex = mission.txList.textures[rayHit.textureIndex];
            bgfx::dbgTextPrintf(2, 8, val_attr, "Texture: %s/%s",
                tex.family.c_str(), tex.name.c_str());
        }
    } else {
        bgfx::dbgTextPrintf(2, 3, 0x0C, "Hit:     NO (miss)");
    }

    // Camera info line
    int32_t camCellIdx = findCameraCell(mission.wrData, state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
    bgfx::dbgTextPrintf(2, 10, hud_attr, "Camera:  (%.2f, %.2f, %.2f)  cell=%d",
        state.cam.pos[0], state.cam.pos[1], state.cam.pos[2], camCellIdx);
    } // end showRaycast

    // ── Acoustic mesh wireframe overlay ──
    // Cyan line wireframe of the Steam Audio acoustic scene mesh, gated
    // on `show_acoustic_mesh`. Submitted to view 2 with the same
    // identity transform / line topology setup as the room wireframe.
    if (state.showAcousticMesh && bgfx::isValid(state.acousticVBH)) {
        float identity[16];
        bx::mtxIdentity(identity);
        bgfx::setTransform(identity);
        bgfx::setVertexBuffer(0, state.acousticVBH, 0, state.acousticLineCount);
        uint64_t wireState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                           | BGFX_STATE_PT_LINES
                           | BGFX_STATE_BLEND_ALPHA
                           | BGFX_STATE_DEPTH_TEST_LESS;
        bgfx::setState(wireState);
        float noFog[4] = {0, 0, 0, 0};
        bgfx::setUniform(gpu.u_fogColor, noFog);
        bgfx::setUniform(gpu.u_fogParams, noFog);
        float opaqueParams[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        float whiteLight[4]   = {1.0f, 1.0f, 1.0f, 0.0f};
        bgfx::setUniform(gpu.u_objectParams, opaqueParams);
        bgfx::setUniform(gpu.u_objectLight, whiteLight);
        bgfx::submit(2, gpu.flatProgram);
    }

    // ── Door geometry wireframe overlay ──
    // Orange line wireframe of all doors registered with the audio scene
    // as IPLInstancedMesh. Doors are the only geometry source NOT in the
    // static acoustic mesh — they're tracked separately so the bake's
    // visibility-edge tests + runtime path-validation can re-evaluate
    // them as the doors open/close. Visualizing them alongside
    // show_acoustic_mesh gives the full picture of geometry the Steam
    // Audio scene actually sees, which is useful when diagnosing why an
    // emitter probe's pathing fails to form expected edges (look for
    // closed doors between the probe and its neighbors).
    if (state.showDoorGeometry) {
        auto audioSvc = GET_SERVICE(Darkness::AudioService);
        if (audioSvc) {
            auto doors = audioSvc->getDoorGeometryForDebug();
            if (!doors.empty()) {
                // Build a transient line buffer from the door triangles.
                // For each triangle (a,b,c) we emit three line segments
                // (a,b)(b,c)(c,a) so the GPU draws a wireframe at LINES
                // topology. Total line-vertex count = numTris * 6.
                size_t totalLineVerts = 0;
                for (const auto &d : doors) totalLineVerts += d.indices.size() * 2;
                bgfx::VertexLayout layout;
                layout.begin()
                      .add(bgfx::Attrib::Position,
                           3, bgfx::AttribType::Float)
                      .end();
                if (totalLineVerts > 0
                    && bgfx::getAvailTransientVertexBuffer(
                           static_cast<uint32_t>(totalLineVerts), layout)
                       == totalLineVerts)
                {
                    bgfx::TransientVertexBuffer tvb;
                    bgfx::allocTransientVertexBuffer(
                        &tvb,
                        static_cast<uint32_t>(totalLineVerts),
                        layout);
                    float *vp = reinterpret_cast<float *>(tvb.data);
                    size_t out = 0;
                    auto pushLine = [&](const float *a, const float *b) {
                        vp[out * 3 + 0] = a[0];
                        vp[out * 3 + 1] = a[1];
                        vp[out * 3 + 2] = a[2];
                        ++out;
                        vp[out * 3 + 0] = b[0];
                        vp[out * 3 + 1] = b[1];
                        vp[out * 3 + 2] = b[2];
                        ++out;
                    };
                    for (const auto &d : doors) {
                        const float *V = d.worldVertices.data();
                        const size_t numTris = d.indices.size() / 3;
                        for (size_t t = 0; t < numTris; ++t) {
                            const float *a = &V[d.indices[t * 3 + 0] * 3];
                            const float *b = &V[d.indices[t * 3 + 1] * 3];
                            const float *c = &V[d.indices[t * 3 + 2] * 3];
                            pushLine(a, b);
                            pushLine(b, c);
                            pushLine(c, a);
                        }
                    }
                    float identity[16];
                    bx::mtxIdentity(identity);
                    bgfx::setTransform(identity);
                    bgfx::setVertexBuffer(0, &tvb, 0,
                                          static_cast<uint32_t>(out));
                    uint64_t wireState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                                       | BGFX_STATE_PT_LINES
                                       | BGFX_STATE_BLEND_ALPHA
                                       | BGFX_STATE_DEPTH_TEST_LESS;
                    bgfx::setState(wireState);
                    float noFog[4] = {0, 0, 0, 0};
                    bgfx::setUniform(gpu.u_fogColor, noFog);
                    bgfx::setUniform(gpu.u_fogParams, noFog);
                    float opaqueParams[4] = {1.0f, 0.0f, 0.0f, 0.0f};
                    // Orange (RGB 1.0 / 0.55 / 0.0) — visually distinct
                    // from the cyan acoustic mesh, the warm yellow probe
                    // markers, and the red/magenta probe-reachability
                    // tints. Doors usually appear singly so a single
                    // distinctive color is enough.
                    float doorTint[4] = {1.0f, 0.55f, 0.0f, 0.0f};
                    bgfx::setUniform(gpu.u_objectParams, opaqueParams);
                    bgfx::setUniform(gpu.u_objectLight, doorTint);
                    bgfx::submit(2, gpu.flatProgram);
                }
            }
        }
    }

    // ── Acoustic-mesh raycast highlighter ──
    // Casts a ray from the camera along the forward direction against the
    // stored acoustic triangles (CPU brute-force Möller-Trumbore) and
    // renders the closest hit triangle solid red. HUD shows hit distance
    // + per-triangle texture name. Designed to scan for holes — anywhere
    // the user expects a wall but no triangle highlights within the
    // configured range indicates missing geometry in the acoustic mesh.
    if (state.showAcousticHit
        && !state.acousticIndices.empty()
        && !state.acousticVerts.empty())
    {
        // Engine Z-up camera forward (matches show_raycast above).
        const float cosPitch = std::cos(state.cam.pitch);
        const float fwdX = std::cos(state.cam.yaw) * cosPitch;
        const float fwdY = std::sin(state.cam.yaw) * cosPitch;
        const float fwdZ = std::sin(state.cam.pitch);
        const float ox = state.cam.pos[0];
        const float oy = state.cam.pos[1];
        const float oz = state.cam.pos[2];

        constexpr float kRayMaxDist = 500.0f;  // engine feet
        const size_t numTris = state.acousticIndices.size() / 3;

        // Möller-Trumbore single-precision, two-sided (audio meshes
        // have no consistent winding rule). Tracks closest positive-t hit.
        float bestT = std::numeric_limits<float>::infinity();
        size_t bestTri = static_cast<size_t>(-1);
        float bestVx0=0, bestVy0=0, bestVz0=0;
        float bestVx1=0, bestVy1=0, bestVz1=0;
        float bestVx2=0, bestVy2=0, bestVz2=0;
        const float *verts = state.acousticVerts.data();
        const int32_t *idx = state.acousticIndices.data();

        for (size_t t = 0; t < numTris; ++t) {
            int32_t i0 = idx[t*3 + 0];
            int32_t i1 = idx[t*3 + 1];
            int32_t i2 = idx[t*3 + 2];
            float ax = verts[i0*3+0], ay = verts[i0*3+1], az = verts[i0*3+2];
            float bx_ = verts[i1*3+0], by = verts[i1*3+1], bz = verts[i1*3+2];
            float cx = verts[i2*3+0], cy = verts[i2*3+1], cz = verts[i2*3+2];
            float e1x = bx_-ax, e1y = by-ay, e1z = bz-az;
            float e2x = cx-ax, e2y = cy-ay, e2z = cz-az;
            // h = dir × e2
            float hx = fwdY*e2z - fwdZ*e2y;
            float hy = fwdZ*e2x - fwdX*e2z;
            float hz = fwdX*e2y - fwdY*e2x;
            float a = e1x*hx + e1y*hy + e1z*hz;
            if (std::fabs(a) < 1e-7f) continue;
            float f = 1.0f / a;
            float sx = ox - ax, sy = oy - ay, sz = oz - az;
            float u = f * (sx*hx + sy*hy + sz*hz);
            if (u < 0.0f || u > 1.0f) continue;
            // q = s × e1
            float qx = sy*e1z - sz*e1y;
            float qy = sz*e1x - sx*e1z;
            float qz = sx*e1y - sy*e1x;
            float vv = f * (fwdX*qx + fwdY*qy + fwdZ*qz);
            if (vv < 0.0f || u + vv > 1.0f) continue;
            float tt = f * (e2x*qx + e2y*qy + e2z*qz);
            if (tt > 1e-4f && tt < bestT && tt < kRayMaxDist) {
                bestT = tt;
                bestTri = t;
                bestVx0 = ax; bestVy0 = ay; bestVz0 = az;
                bestVx1 = bx_; bestVy1 = by; bestVz1 = bz;
                bestVx2 = cx; bestVy2 = cy; bestVz2 = cz;
            }
        }

        // HUD readout — always emit, hit or miss, since "miss within
        // kRayMaxDist" is the signal we're hunting.
        uint8_t hudAttr = 0x4f;  // white on red — high contrast
        if (bestTri == static_cast<size_t>(-1)) {
            bgfx::dbgTextPrintf(2, 12, hudAttr,
                "Acoustic mesh: NO HIT within %.0f ft (camera forward) — likely hole in mesh",
                kRayMaxDist);
        } else {
            const char *texName = (bestTri < state.acousticTexNames.size())
                ? state.acousticTexNames[bestTri].c_str() : "<unknown>";
            float hx = ox + fwdX * bestT;
            float hy = oy + fwdY * bestT;
            float hz = oz + fwdZ * bestT;
            bgfx::dbgTextPrintf(2, 12, 0x2f,  // white on green
                "Acoustic mesh hit: tri=%zu t=%.2f ft  pos=(%.1f,%.1f,%.1f)  tex='%s'",
                bestTri, bestT, hx, hy, hz, texName);

            // Nudge the triangle slightly toward the camera along its
            // surface normal. The hit triangle lies on the exact same
            // surface as the rendered wall (the world pass drew the
            // same poly at the same depth), so without an offset the
            // highlight z-fights or fails DEPTH_TEST. 0.05 ft = 1.5 cm
            // — invisible at any normal viewing distance, more than
            // enough to break the tie.
            float nx_ = (bestVy1 - bestVy0) * (bestVz2 - bestVz0)
                      - (bestVz1 - bestVz0) * (bestVy2 - bestVy0);
            float ny_ = (bestVz1 - bestVz0) * (bestVx2 - bestVx0)
                      - (bestVx1 - bestVx0) * (bestVz2 - bestVz0);
            float nz_ = (bestVx1 - bestVx0) * (bestVy2 - bestVy0)
                      - (bestVy1 - bestVy0) * (bestVx2 - bestVx0);
            float nLen = std::sqrt(nx_*nx_ + ny_*ny_ + nz_*nz_);
            if (nLen > 1e-6f) {
                nx_ /= nLen; ny_ /= nLen; nz_ /= nLen;
                // Orient toward the camera (flip if facing away).
                if (nx_ * fwdX + ny_ * fwdY + nz_ * fwdZ > 0.0f) {
                    nx_ = -nx_; ny_ = -ny_; nz_ = -nz_;
                }
                constexpr float kBias = 0.05f;
                bestVx0 += nx_ * kBias; bestVy0 += ny_ * kBias; bestVz0 += nz_ * kBias;
                bestVx1 += nx_ * kBias; bestVy1 += ny_ * kBias; bestVz1 += nz_ * kBias;
                bestVx2 += nx_ * kBias; bestVy2 += ny_ * kBias; bestVz2 += nz_ * kBias;
            }

            // Render via transient VB — same pattern as show_raycast
            // above. Solid bright magenta. Depth test LEQUAL with the
            // 0.05 ft camera-ward bias above: the hit triangle's bias
            // is more than enough to win the depth comparison against
            // the wall the ray actually hit (which is exactly the
            // same surface, so they z-fight without the bias), but
            // nowhere near enough to escape occlusion by any closer
            // geometry. Net effect: the highlight is visible when
            // the camera has line-of-sight to the hit triangle, and
            // hidden when a closer wall (that may or may not be in
            // the acoustic mesh itself) sits between camera and hit.
            // The latter case is the smoking-gun for a hole in the
            // acoustic mesh: HUD reports a hit at e.g. 80 ft, but
            // the user sees a wall 10 ft ahead and no magenta — the
            // raycast went THROUGH the visible wall because it's
            // missing from the acoustic mesh. No fog. No cull —
            // brute-force raycast is two-sided.
            constexpr uint32_t kHitVerts = 3;
            if (bgfx::getAvailTransientVertexBuffer(
                    kHitVerts, Darkness::PosColorVertex::layout) >= kHitVerts) {
                bgfx::TransientVertexBuffer hitTvb;
                bgfx::allocTransientVertexBuffer(
                    &hitTvb, kHitVerts, Darkness::PosColorVertex::layout);
                auto *tv = reinterpret_cast<Darkness::PosColorVertex *>(hitTvb.data);
                tv[0] = {bestVx0, bestVy0, bestVz0, 0xFFFF00FFu};
                tv[1] = {bestVx1, bestVy1, bestVz1, 0xFFFF00FFu};
                tv[2] = {bestVx2, bestVy2, bestVz2, 0xFFFF00FFu};

                float identity[16];
                bx::mtxIdentity(identity);
                bgfx::setTransform(identity);
                bgfx::setVertexBuffer(0, &hitTvb, 0, kHitVerts);
                uint64_t triState = BGFX_STATE_WRITE_RGB
                                  | BGFX_STATE_WRITE_A
                                  | BGFX_STATE_DEPTH_TEST_LEQUAL
                                  | BGFX_STATE_BLEND_ALPHA;
                bgfx::setState(triState);
                float noFog[4] = {0, 0, 0, 0};
                bgfx::setUniform(gpu.u_fogColor, noFog);
                bgfx::setUniform(gpu.u_fogParams, noFog);
                float opaqueParams[4] = {1.0f, 0.0f, 0.0f, 0.0f};
                float whiteLight[4]   = {1.0f, 1.0f, 1.0f, 0.0f};
                bgfx::setUniform(gpu.u_objectParams, opaqueParams);
                bgfx::setUniform(gpu.u_objectLight, whiteLight);
                bgfx::submit(2, gpu.flatProgram);
            }
        }
    }

    // ── Room/portal wireframe overlay ──
    // Identity transform, line topology, alpha-blended, depth-test
    // ALWAYS so wireframes draw on top of world geometry (LESS would
    // hide them behind walls when the OBBs are correctly positioned
    // *inside* the architecture). Gated on `show_rooms` only — used
    // to be inside the `if (state.showRaycast)` scope by accident,
    // which is why the wireframes only appeared when both flags were
    // on.
    //
    // Geometry is rebuilt every frame from per-room CPU buffers
    // (state.roomDebug) into a transient vertex buffer. The closest-N
    // cull (debug_room_max_count) keeps visual clutter manageable in
    // levels with hundreds of rooms — sorting a few hundred floats per
    // frame is negligible compared to even one draw call.
    auto submitWireframeBatch = [&](const std::vector<Darkness::PosColorVertex> &verts) {
        if (verts.empty()) return;
        uint32_t count = static_cast<uint32_t>(verts.size());
        if (bgfx::getAvailTransientVertexBuffer(count, Darkness::PosColorVertex::layout) < count)
            return;
        bgfx::TransientVertexBuffer tvb;
        bgfx::allocTransientVertexBuffer(&tvb, count, Darkness::PosColorVertex::layout);
        std::memcpy(tvb.data, verts.data(), count * sizeof(Darkness::PosColorVertex));

        float identity[16];
        bx::mtxIdentity(identity);
        bgfx::setTransform(identity);
        bgfx::setVertexBuffer(0, &tvb, 0, count);
        uint64_t wireState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                           | BGFX_STATE_PT_LINES
                           | BGFX_STATE_BLEND_ALPHA
                           | BGFX_STATE_DEPTH_TEST_ALWAYS;
        bgfx::setState(wireState);
        float noFog[4] = {0, 0, 0, 0};
        bgfx::setUniform(gpu.u_fogColor, noFog);
        bgfx::setUniform(gpu.u_fogParams, noFog);
        float opaqueParams[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        float whiteLight[4]   = {1.0f, 1.0f, 1.0f, 0.0f};
        bgfx::setUniform(gpu.u_objectParams, opaqueParams);
        bgfx::setUniform(gpu.u_objectLight, whiteLight);
        bgfx::submit(2, gpu.flatProgram);
    };

    if ((state.showRooms || state.showPortals) && !state.roomDebug.empty()) {
        Darkness::Vector3 camPos(state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
        auto picks = selectClosestRooms(state, camPos);

        // Concatenate the per-room buffers for whichever overlay(s) are
        // enabled, then submit each as a single draw call. The buffers
        // never share verts between overlays (different colors), so
        // show_rooms and show_portals each get their own TVB.
        if (state.showRooms) {
            size_t total = 0;
            for (size_t idx : picks) total += state.roomDebug[idx].roomLines.size();
            std::vector<Darkness::PosColorVertex> batch;
            batch.reserve(total);
            for (size_t idx : picks) {
                const auto &rl = state.roomDebug[idx].roomLines;
                batch.insert(batch.end(), rl.begin(), rl.end());
            }
            submitWireframeBatch(batch);
        }

        // Portal-only overlay (light pink). Independent of show_rooms;
        // useful for tracing BFS path geometry without the room-OBB
        // wireframe clutter. Same closest-N cull as show_rooms.
        if (state.showPortals) {
            size_t total = 0;
            for (size_t idx : picks) total += state.roomDebug[idx].portalLines.size();
            std::vector<Darkness::PosColorVertex> batch;
            batch.reserve(total);
            for (size_t idx : picks) {
                const auto &pl = state.roomDebug[idx].portalLines;
                batch.insert(batch.end(), pl.begin(), pl.end());
            }
            submitWireframeBatch(batch);
        }
    }

    // ── Per-voice virtual-position overlay (show_vpos) ──
    //
    // For each active voice, draw one polyline per ACTIVE propagation
    // path (the multi-path renderer mixes up to kMaxSubSources of
    // these simultaneously). Each path's polyline runs
    // source → bend₀ → bend₁ → … → endpoint → listener and is
    // coloured by slot index so two paths through different doorways
    // are visually distinct. Same-room / clean-threaded paths render
    // as a single straight magenta segment from source to listener.
    //
    // What to look for:
    //   • A single jumpy line per voice ⇒ propagation is keeping
    //     only one path (legacy single-source behaviour or
    //     maxPaths == 1).
    //   • Multiple stable lines per voice ⇒ multi-path renderer is
    //     working as intended; the listener perceives a spatial
    //     spread across all the doorways the lines pass through.
    //   • A line ending inside solid geometry ⇒ that path's
    //     endpoint is mis-positioned. The audio for that slot will
    //     point HRTF panning into the wall and may interact badly
    //     with Steam Audio's per-slot volumetric occlusion sampling.
    if (state.showVPos) {
        Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);
        if (audioSvc) {
            auto snapshots = audioSvc->getVoiceSpatialSnapshots();
            // Worst-case vert count: each voice = Σ_paths (bends + 1) * 2.
            // Bound paths by kMaxSubSources (4) and bends by ~16 per path.
            uint32_t needed = 0;
            for (const auto &s : snapshots) {
                if (!s.reached) continue;
                if (!s.paths.empty()) {
                    for (const auto &p : s.paths) {
                        needed += static_cast<uint32_t>((p.chain.size() + 1) * 2);
                    }
                } else {
                    // Fallback to the legacy single-chain rendering if the
                    // propagation result didn't expose per-path data.
                    needed += static_cast<uint32_t>((s.chain.size() + 1) * 2);
                }
            }
            if (needed > 0 &&
                bgfx::getAvailTransientVertexBuffer(
                    needed, Darkness::PosColorVertex::layout) >= needed) {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(
                    &tvb, needed, Darkness::PosColorVertex::layout);
                auto *verts = reinterpret_cast<Darkness::PosColorVertex *>(tvb.data);
                uint32_t n = 0;
                // One distinct hue per sub-source slot (ABGR; bright +
                // saturated for visibility on the dark engine palette).
                // Slot 0 (primary path) keeps the legacy cyan/yellow so
                // existing screenshots / muscle memory still read.
                constexpr uint32_t kSlot0     = 0xFF00FFFFu;  // yellow
                constexpr uint32_t kSlot1     = 0xFF00FF00u;  // green
                constexpr uint32_t kSlot2     = 0xFFFF8800u;  // orange
                constexpr uint32_t kSlot3     = 0xFFFF00FFu;  // magenta
                constexpr uint32_t kSlotOther = 0xFFFFAA00u;  // light blue (overflow)
                constexpr uint32_t kClean     = 0xFFFF00FFu;  // magenta (clean-threaded)
                Vector3 lst(state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
                auto emit = [&](const Vector3 &p, uint32_t c) {
                    verts[n].x = p.x;
                    verts[n].y = p.y;
                    verts[n].z = p.z;
                    verts[n].abgr = c;
                    ++n;
                };
                auto slotColor = [&](size_t slotIdx) -> uint32_t {
                    switch (slotIdx) {
                        case 0:  return kSlot0;
                        case 1:  return kSlot1;
                        case 2:  return kSlot2;
                        case 3:  return kSlot3;
                        default: return kSlotOther;
                    }
                };
                // Divergence overlay (Phase 1 of PLAN.CELL_GRAPH_PROPAGATION):
                //   • Cell-graph paths render as the existing solid polyline.
                //   • Steam-Audio paths (Phase 2) will render as dashed
                //     polyline segments — gap pattern hard-coded to skip
                //     every other segment so the two overlays don't
                //     overlap visually.
                //   • Per-voice colour gradient (cell-graph slot0):
                //       green ≤3 dB delta between backends
                //       yellow 3..8 dB
                //       red    >8 dB OR no secondary backend reached
                //     With no secondary backend (Phase 1 default), the
                //     overlay stays on the slot-default cyan/yellow palette.
                constexpr uint32_t kDelta_Green  = 0xFF00FF00u;
                constexpr uint32_t kDelta_Yellow = 0xFF00FFFFu;
                constexpr uint32_t kDelta_Red    = 0xFF0000FFu;
                auto emitChain = [&](const Vector3 &src, const Vector3 &lst,
                                      const std::vector<Vector3> &chain,
                                      uint32_t col, bool dashed) {
                    if (chain.empty()) {
                        emit(src, col);
                        emit(lst, col);
                        return;
                    }
                    // Dashed rendering: drop every other segment so the
                    // line shows as a gapped stipple, distinguishing
                    // Steam Audio's chain from the cell-graph chain
                    // when both are drawn together.
                    auto addSeg = [&](const Vector3 &a, const Vector3 &b,
                                       size_t idx) {
                        if (dashed && (idx & 1u)) return;
                        emit(a, col);
                        emit(b, col);
                    };
                    addSeg(src, chain.front(), 0);
                    for (size_t i = 1; i < chain.size(); ++i) {
                        addSeg(chain[i - 1], chain[i], i);
                    }
                    addSeg(chain.back(), lst, chain.size());
                };
                for (const auto &s : snapshots) {
                    if (!s.reached) continue;
                    // Per-voice divergence colour: applied to the
                    // primary chain's slot-0 entry so the operator can
                    // tell at a glance whether the two backends agree.
                    uint32_t primaryCol = kSlot0;
                    if (s.hasSecondaryBackend
                        && s.primaryEffectiveDistance > 0.0f
                        && s.secondaryEffectiveDistance > 0.0f) {
                        float deltaDb = 20.0f * std::log10(
                            s.secondaryEffectiveDistance
                                / s.primaryEffectiveDistance);
                        float absDb = std::fabs(deltaDb);
                        if (absDb <= 3.0f)      primaryCol = kDelta_Green;
                        else if (absDb <= 8.0f) primaryCol = kDelta_Yellow;
                        else                    primaryCol = kDelta_Red;
                    }

                    if (s.paths.empty()) {
                        // Legacy single-chain rendering (fallback if
                        // propagation didn't surface per-path data).
                        if (s.chain.empty()) {
                            emit(s.sourcePos, kClean);
                            emit(lst,         kClean);
                        } else {
                            emitChain(s.sourcePos, lst, s.chain,
                                       primaryCol, /*dashed=*/false);
                        }
                        continue;
                    }
                    for (size_t pi = 0; pi < s.paths.size(); ++pi) {
                        const auto &p = s.paths[pi];
                        uint32_t col = (pi == 0) ? primaryCol : slotColor(pi);
                        // Steam Audio backend (Phase 2) renders dashed;
                        // cell-graph renders solid.
                        bool dashed = (p.backend ==
                            Darkness::PropagationBackend::SteamAudio);
                        emitChain(s.sourcePos, lst, p.chain, col, dashed);
                    }
                }
                if (n > 0) {
                    float identity[16];
                    bx::mtxIdentity(identity);
                    bgfx::setTransform(identity);
                    bgfx::setVertexBuffer(0, &tvb, 0, n);
                    uint64_t lineState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                                       | BGFX_STATE_PT_LINES
                                       | BGFX_STATE_BLEND_ALPHA
                                       | BGFX_STATE_DEPTH_TEST_ALWAYS;
                    bgfx::setState(lineState);
                    float noFog[4] = {0, 0, 0, 0};
                    bgfx::setUniform(gpu.u_fogColor, noFog);
                    bgfx::setUniform(gpu.u_fogParams, noFog);
                    float opaqueParams[4] = {1.0f, 0.0f, 0.0f, 0.0f};
                    float whiteLight[4]   = {1.0f, 1.0f, 1.0f, 0.0f};
                    bgfx::setUniform(gpu.u_objectParams, opaqueParams);
                    bgfx::setUniform(gpu.u_objectLight, whiteLight);
                    bgfx::submit(2, gpu.flatProgram);
                }
            }
        }
    }

    // ── Room ID labels (screen-space text via bgfx::dbgTextPrintf) ──
    //
    // For each room center, project to clip space using the current view +
    // projection matrices. If the result lands inside the viewport and in
    // front of the camera, convert to the debug-text character grid and
    // print the room ID. The character grid is ~160×45 at 1280×720, so the
    // labels are coarse but immediately readable.
    //
    // (The room-ID label overlay used to live here, but the main render
    // loop calls `bgfx::dbgTextClear()` again after renderDebugOverlay
    // returns — to set up a clean text buffer for the frob hint and
    // console — which wiped everything we wrote. The label rendering
    // has been moved into the main loop after that final clear; see
    // the call to `renderRoomLabelsOverlay` there. Wireframe drawing
    // for show_rooms still happens above in this function, since lines
    // are written into the 3D view buffer and aren't affected by
    // dbgTextClear.)

    // ── Probe + listener-marker overlay ──
    // Independent of show_raycast. One bgfx draw per probe (using the
    // fallback cube). Both reflection and pathing probes are drawn as
    // cubes — they differ by tint and (per-class) size. Both share
    // the same camera-nearest-rooms cull as show_probe_radius so the
    // sphere overlay (when also on) appears/disappears in sync with
    // the cubes.
    if (state.showProbes
        && bgfx::isValid(gpu.fallbackCubeVBH)
        && bgfx::isValid(gpu.fallbackCubeIBH))
    {
        auto audioSvc = GET_SERVICE(Darkness::AudioService);

        // Tints — reflection probes are colored by ProbeFate (cyan if
        // Kept, plus warning palette for failure modes). Pathing probes
        // are colored by layer-2 occlusion state via the shared
        // classifier (matches show_probe_radius).
        //
        // ── Reflection-probe palette ──
        //   cyan    — Kept (in a room reachable from a mapper-placed
        //             object) AND the active (nearest) probe — the
        //             active one stands out by SIZE, not color
        //   red     — NoRoom (BSP void / unroomed cell)
        //   magenta — Unreachable (has a room, but isolated from playable
        //             space — would prune at bake-time even though IPL
        //             placed it on a real floor)
        //   yellow  — Isolated (graph-isolated; Steam Audio's visibility
        //             bake never formed an edge from this probe —
        //             populated from runtime evidence)
        //
        // ── Pathing-probe palette ── (same as show_probe_radius sphere)
        //   green   — Visible (listener inside influence sphere, ray
        //             clears the acoustic mesh — probe contributes)
        //   red     — Occluded (listener inside sphere, ray hits the
        //             acoustic mesh — Steam Audio's checkOcclusion
        //             would drop this probe at runtime)
        //   gray    — OutOfRange (listener outside influence sphere)
        //
        // Reflection fate classification comes from
        // AudioService::getProbeFates(); if empty or wrong-length we
        // fall back to cyan-only.
        if (audioSvc) {
            const auto &reflProbes  = audioSvc->getProbePositions();
            const auto &fates       = audioSvc->getProbeFates();
            const bool  fatesValid  = fates.size() == reflProbes.size();
            const auto  pathingViz  = audioSvc->getPathingProbeViz();
            Darkness::Vector3 listenerPos = audioSvc->getListenerPos();

            // Camera-nearest-rooms cull (shared with show_probe_radius
            // and show_rooms). Build a set of room IDs to keep so the
            // per-probe filter is O(1). Empty keepRooms → no cull
            // (debug_room_max_count was 0 or ≥ #rooms).
            Darkness::Vector3 camPos(state.cam.pos[0],
                                      state.cam.pos[1],
                                      state.cam.pos[2]);
            std::vector<size_t> nearestRoomIdx =
                selectClosestRooms(state, camPos);
            std::unordered_set<int32_t> keepRooms;
            keepRooms.reserve(nearestRoomIdx.size());
            for (size_t idx : nearestRoomIdx) {
                if (idx < state.roomDebug.size()) {
                    keepRooms.insert(state.roomDebug[idx].roomID);
                }
            }
            const bool roomCullEnabled = !keepRooms.empty();

            // Need RoomService to look up each reflection probe's
            // enclosing room ID for the cull. Pathing probes carry
            // their roomID inside PathingProbeViz.
            Darkness::RoomServicePtr roomSvc =
                GET_SERVICE(Darkness::RoomService);

            int   nearestIdx  = -1;
            float nearestDistSq = std::numeric_limits<float>::max();
            for (size_t i = 0; i < reflProbes.size(); ++i) {
                Darkness::Vector3 d = reflProbes[i] - listenerPos;
                float ds = glm::dot(d, d);
                if (ds < nearestDistSq) {
                    nearestDistSq = ds;
                    nearestIdx = static_cast<int>(i);
                }
            }

            uint64_t probeState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                                | BGFX_STATE_CULL_CW;
            float opaque[4]      = {1.0f, 0.0f, 0.0f, 0.0f};
            float keptTint[4]    = {0.0f, 0.8f, 1.0f, 0.0f};  // cyan
            float noRoomTint[4]  = {1.0f, 0.0f, 0.0f, 0.0f};  // red
            float unreachTint[4] = {0.9f, 0.0f, 0.7f, 0.0f};  // magenta
            float isolatedTint[4]= {1.0f, 0.85f, 0.0f, 0.0f}; // yellow

            // Reflection probes: shrunk to 25% of probeMarkerSize so
            // the dense bake grid stops filling the view. The active
            // (nearest-to-listener) probe stays at 50% (2x the new
            // shrunk default) so it pops by SIZE rather than by a
            // color swap — color stays the same cyan so the active
            // indicator doesn't read as a different class.
            const float kReflShrink     = 0.25f;
            const float kReflActiveMul  = 2.0f;
            const float reflSize        = state.probeMarkerSize * kReflShrink;
            const float reflActiveSize  = reflSize * kReflActiveMul;
            for (size_t i = 0; i < reflProbes.size(); ++i) {
                const auto &p = reflProbes[i];
                // Camera-nearest-rooms cull. Active probe is exempt
                // so the listener can always find their nearest probe
                // even when standing in an edge-of-set region.
                if (roomCullEnabled && static_cast<int>(i) != nearestIdx) {
                    int32_t rid = -1;
                    if (roomSvc) {
                        Darkness::Room *r = roomSvc->roomFromPoint(p);
                        if (r) rid = r->getRoomID();
                    }
                    if (rid < 0 || keepRooms.find(rid) == keepRooms.end())
                        continue;
                }
                const bool isActive = (static_cast<int>(i) == nearestIdx);
                float sz = isActive ? reflActiveSize : reflSize;
                float mtx[16];
                bx::mtxSRT(mtx, sz, sz, sz, 0.0f, 0.0f, 0.0f,
                           p.x, p.y, p.z);
                bgfx::setTransform(mtx);
                bgfx::setVertexBuffer(0, gpu.fallbackCubeVBH);
                bgfx::setIndexBuffer(gpu.fallbackCubeIBH);
                bgfx::setState(probeState);
                float noFog[4] = {0, 0, 0, 0};
                bgfx::setUniform(gpu.u_fogColor,  noFog);
                bgfx::setUniform(gpu.u_fogParams, noFog);
                bgfx::setUniform(gpu.u_objectParams, opaque);

                // Pick tint from ProbeFate. Active probe keeps the
                // Kept (cyan) tint — pop-by-size, not color swap.
                float *tint = keptTint;
                if (!isActive && fatesValid) {
                    switch (fates[i]) {
                        case Darkness::AudioService::ProbeFate::NoRoom:
                            tint = noRoomTint; break;
                        case Darkness::AudioService::ProbeFate::Unreachable:
                            tint = unreachTint; break;
                        case Darkness::AudioService::ProbeFate::Isolated:
                            tint = isolatedTint; break;
                        case Darkness::AudioService::ProbeFate::Kept:
                        default:
                            tint = keptTint; break;
                    }
                }
                bgfx::setUniform(gpu.u_objectLight, tint);
                bgfx::submit(2, gpu.flatProgram);
            }

            // Pathing-batch probe overlay. Uses the same layer-2
            // occlusion classification as show_probe_radius (shared
            // file-scope helper classifyPathingProbe +
            // probeStateToRGBA) so a probe's cube here and its sphere
            // there always have the same color. Cube size unchanged
            // (full probeMarkerSize) so pathing cubes still read as
            // distinct from the shrunk reflection cubes by size alone.
            if (!pathingViz.empty()) {
                const size_t numAcousticTris =
                    state.acousticIndices.size() / 3;
                const float *aVerts = state.acousticVerts.data();
                const int32_t *aIdx = state.acousticIndices.data();
                const float pathSize = state.probeMarkerSize;
                for (const auto &v : pathingViz) {
                    // Camera-nearest-rooms cull (same as reflection
                    // probes). PathingProbeViz already carries the
                    // probe's roomID so no per-probe roomFromPoint
                    // call is needed.
                    if (v.roomID < 0) continue;  // BSP void
                    if (roomCullEnabled
                        && keepRooms.find(v.roomID) == keepRooms.end()) {
                        continue;
                    }
                    ProbeOccState s = classifyPathingProbe(
                        listenerPos, v.position, v.radiusFt,
                        aVerts, aIdx, numAcousticTris);
                    float pathTint[4];
                    probeStateToRGBA(s, pathTint);
                    float mtx[16];
                    bx::mtxSRT(mtx, pathSize, pathSize, pathSize,
                               0.0f, 0.0f, 0.0f,
                               v.position.x, v.position.y, v.position.z);
                    bgfx::setTransform(mtx);
                    bgfx::setVertexBuffer(0, gpu.fallbackCubeVBH);
                    bgfx::setIndexBuffer(gpu.fallbackCubeIBH);
                    bgfx::setState(probeState);
                    float noFog[4] = {0, 0, 0, 0};
                    bgfx::setUniform(gpu.u_fogColor,  noFog);
                    bgfx::setUniform(gpu.u_fogParams, noFog);
                    bgfx::setUniform(gpu.u_objectParams, opaque);
                    bgfx::setUniform(gpu.u_objectLight, pathTint);
                    bgfx::submit(2, gpu.flatProgram);
                }
            }
        }

        // Listener marker: a yellow cube at the listener while any
        // player-emitted voice (footstep / land) is sounding. The marker
        // visibly flashes on each step, pinned to where you're standing —
        // so dips in audible reverb correlate directly with how close
        // the flash is to the nearest (orange) probe.
        bool markerVisibleThisFrame = false;
        if (audioSvc && audioSvc->isPlayerEmittedVoiceActive())
        {
            Darkness::Vector3 lp = audioSvc->getListenerPos();
            float sz = state.probeMarkerSize * 2.0f;  // slightly larger than probe markers
            float mtx[16];
            bx::mtxSRT(mtx, sz, sz, sz, 0.0f, 0.0f, 0.0f, lp.x, lp.y, lp.z);
            bgfx::setTransform(mtx);
            bgfx::setVertexBuffer(0, gpu.fallbackCubeVBH);
            bgfx::setIndexBuffer(gpu.fallbackCubeIBH);
            uint64_t markerState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                                 | BGFX_STATE_CULL_CW;
            bgfx::setState(markerState);
            float noFog[4]  = {0, 0, 0, 0};
            float opaque[4] = {1.0f, 0.0f, 0.0f, 0.0f};
            float yellow[4] = {1.0f, 1.0f, 0.0f, 0.0f};
            bgfx::setUniform(gpu.u_fogColor,  noFog);
            bgfx::setUniform(gpu.u_fogParams, noFog);
            bgfx::setUniform(gpu.u_objectParams, opaque);
            bgfx::setUniform(gpu.u_objectLight,  yellow);
            bgfx::submit(2, gpu.flatProgram);
            markerVisibleThisFrame = true;
        }

        // HUD readout. Lives here (not inside the showRaycast HUD block
        // above, which incorrectly nests the acoustic-mesh and other
        // overlays under show_raycast). Always tells you whether positions
        // are loaded; otherwise it's too easy to think "the overlay is
        // broken" when really the sidecar hasn't been written yet.
        bgfx::setDebug(BGFX_DEBUG_TEXT);
        int probeCount = audioSvc
            ? static_cast<int>(audioSvc->getProbePositions().size())
            : 0;
        bool footActive = audioSvc && audioSvc->isPlayerEmittedVoiceActive();
        const uint8_t labelAttr = 0x0F;  // white on black
        const uint8_t valAttr   = 0x0A;  // green on black
        const uint8_t warnAttr  = 0x0C;  // red on black
        if (probeCount > 0) {
            // Break the count down by reachability bucket so the user can see
            // at a glance how many probes a future filter would prune without
            // having to scrub the stderr log.
            int kept = 0, noRoom = 0, unreach = 0, isolated = 0;
            if (audioSvc) {
                const auto &fates = audioSvc->getProbeFates();
                if (static_cast<int>(fates.size()) == probeCount) {
                    for (auto f : fates) {
                        switch (f) {
                            case Darkness::AudioService::ProbeFate::Kept:        ++kept;     break;
                            case Darkness::AudioService::ProbeFate::NoRoom:      ++noRoom;   break;
                            case Darkness::AudioService::ProbeFate::Unreachable: ++unreach;  break;
                            case Darkness::AudioService::ProbeFate::Isolated:    ++isolated; break;
                        }
                    }
                }
            }
            bgfx::dbgTextPrintf(2, 14, valAttr,
                "Probes: %d refl  (cyan=%d kept (2x size=active), red=%d noRoom, "
                "magenta=%d unreach, yellow=%d isolated, size=%.2fft) | "
                "pathing cubes share show_probe_radius coloring "
                "(green=LOS, red=occluded, gray=out-of-range)",
                probeCount, kept, noRoom, unreach, isolated,
                state.probeMarkerSize * 0.25f);
        } else {
            bgfx::dbgTextPrintf(2, 14, warnAttr,
                "Probes: 0 positions loaded — run `bake_probes on` "
                "in console to regenerate the sidecar");
        }
        bgfx::dbgTextPrintf(2, 15,
            markerVisibleThisFrame ? valAttr : labelAttr,
            "Listener marker: %s (playerEmittedActive=%s)",
            markerVisibleThisFrame ? "VISIBLE this frame" : "hidden (no active player voice)",
            footActive ? "yes" : "no");
    }

    // ── Pathing-probe overlay (wireframe influence spheres) ──
    //
    // Separate from show_probes (cubes for both batches). Draws each
    // pathing probe as a wireframe sphere sized to its adaptive
    // influence radius so the Voronoi-ish placement is visible at a
    // glance. Filtered by the camera-nearest rooms (shared with the
    // room/portal overlay via debug_room_max_count) — without this
    // filter the sphere mesh count tanks framerate in dense missions.
    //
    // Robust against early frames: getPathingProbeViz returns empty
    // when the audio service isn't ready or the batch is empty; the
    // roomID==-1 sentinel auto-filters out-of-room probes.
    if (state.showProbeRadius) {
        auto audioSvc = GET_SERVICE(Darkness::AudioService);
        if (audioSvc) {
            std::vector<Darkness::AudioService::PathingProbeViz> probes =
                audioSvc->getPathingProbeViz();
            if (!probes.empty()) {
                // Resolve the camera-nearest-N room set to gate which
                // probes are drawn. The room overlay uses size_t indices
                // into state.roomDebug; we need real room IDs to match
                // PathingProbeViz::roomID, so build that set first.
                Darkness::Vector3 camPos(state.cam.pos[0],
                                          state.cam.pos[1],
                                          state.cam.pos[2]);
                std::vector<size_t> nearest = selectClosestRooms(state, camPos);
                std::unordered_set<int32_t> keepRooms;
                keepRooms.reserve(nearest.size());
                for (size_t idx : nearest) {
                    if (idx < state.roomDebug.size()) {
                        keepRooms.insert(state.roomDebug[idx].roomID);
                    }
                }

                // Build line vertices: 3 orthogonal great circles per
                // probe (XY, XZ, YZ), kSphereSegments segments each.
                // Each circle = kSphereSegments line segments =
                // 2×kSphereSegments verts. Per probe: 3 circles ×
                // 2 × kSphereSegments verts.
                constexpr uint32_t kSphereSegments = 32;
                constexpr uint32_t kVertsPerCircle = kSphereSegments * 2;
                constexpr uint32_t kVertsPerProbe  = kVertsPerCircle * 3;

                // Filter probes first so the vertex budget is exact.
                std::vector<const Darkness::AudioService::PathingProbeViz *> drawList;
                drawList.reserve(probes.size());
                for (const auto &p : probes) {
                    if (p.roomID < 0) continue;       // BSP void
                    if (!keepRooms.empty()
                        && keepRooms.find(p.roomID) == keepRooms.end()) {
                        continue;  // out of camera-nearest rooms
                    }
                    drawList.push_back(&p);
                }

                const uint32_t kMaxProbes = 256;  // 256 × 192 verts ≈ 49k verts
                if (drawList.size() > kMaxProbes) drawList.resize(kMaxProbes);
                const uint32_t needVerts =
                    static_cast<uint32_t>(drawList.size()) * kVertsPerProbe;

                // Hoisted out of the buffer-alloc block so the HUD
                // line further down can read them even when the
                // transient vertex buffer was rejected (low-budget
                // frame). Updated in the per-probe classification
                // loop inside the buffer block.
                int nVisible = 0, nOccluded = 0, nOutOfRange = 0;
                if (needVerts > 0
                    && bgfx::getAvailTransientVertexBuffer(
                            needVerts, Darkness::PosColorVertex::layout) >= needVerts) {
                    bgfx::TransientVertexBuffer tvb;
                    bgfx::allocTransientVertexBuffer(
                        &tvb, needVerts, Darkness::PosColorVertex::layout);
                    auto *verts = reinterpret_cast<Darkness::PosColorVertex *>(tvb.data);
                    uint32_t numVerts = 0;

                    // Per-probe layer-2 occlusion classification via
                    // the shared file-scope helpers near the top of
                    // this file (classifyPathingProbe +
                    // probeStateToABGR). show_probes uses the same
                    // helpers (with probeStateToRGBA) so the cube and
                    // sphere for the same probe agree on color.
                    // Counters live in the outer scope so the HUD line
                    // below can read them even when the buffer alloc
                    // failed.
                    const Darkness::Vector3 listenerPos(state.cam.pos[0],
                                                         state.cam.pos[1],
                                                         state.cam.pos[2]);
                    const size_t numAcousticTris =
                        state.acousticIndices.size() / 3;
                    const float *aVerts = state.acousticVerts.data();
                    const int32_t *aIdx = state.acousticIndices.data();
                    std::vector<uint32_t> probeColors;
                    probeColors.reserve(drawList.size());
                    for (const auto *p : drawList) {
                        ProbeOccState s = classifyPathingProbe(
                            listenerPos, p->position, p->radiusFt,
                            aVerts, aIdx, numAcousticTris);
                        switch (s) {
                            case ProbeOccState::Visible:    ++nVisible;    break;
                            case ProbeOccState::Occluded:   ++nOccluded;   break;
                            case ProbeOccState::OutOfRange: ++nOutOfRange; break;
                        }
                        probeColors.push_back(probeStateToABGR(s));
                    }

                    const float twoPi = 6.28318530717958647692f;

                    auto emitCircle = [&](const Darkness::Vector3 &c,
                                          float r,
                                          int planeAxis,
                                          uint32_t color) {
                        // planeAxis: 0=XY (z fixed), 1=XZ (y fixed),
                        // 2=YZ (x fixed)
                        for (uint32_t i = 0; i < kSphereSegments; ++i) {
                            float t0 = (twoPi * i) / kSphereSegments;
                            float t1 = (twoPi * (i + 1)) / kSphereSegments;
                            float c0 = std::cos(t0), s0 = std::sin(t0);
                            float c1 = std::cos(t1), s1 = std::sin(t1);
                            Darkness::Vector3 a = c;
                            Darkness::Vector3 b = c;
                            if (planeAxis == 0) {
                                a.x += r * c0; a.y += r * s0;
                                b.x += r * c1; b.y += r * s1;
                            } else if (planeAxis == 1) {
                                a.x += r * c0; a.z += r * s0;
                                b.x += r * c1; b.z += r * s1;
                            } else {
                                a.y += r * c0; a.z += r * s0;
                                b.y += r * c1; b.z += r * s1;
                            }
                            verts[numVerts++] = { a.x, a.y, a.z, color };
                            verts[numVerts++] = { b.x, b.y, b.z, color };
                        }
                    };

                    for (size_t pi = 0; pi < drawList.size(); ++pi) {
                        const auto *p = drawList[pi];
                        uint32_t color = probeColors[pi];
                        emitCircle(p->position, p->radiusFt, 0, color);
                        emitCircle(p->position, p->radiusFt, 1, color);
                        emitCircle(p->position, p->radiusFt, 2, color);
                    }

                    // Draw via view 2 (debug overlay), depth-disabled
                    // so spheres remain visible through walls — matches
                    // the existing show_probes cube overlay convention.
                    float identity[16];
                    bx::mtxIdentity(identity);
                    bgfx::setTransform(identity);
                    bgfx::setVertexBuffer(0, &tvb, 0, numVerts);
                    uint64_t lineState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                                       | BGFX_STATE_PT_LINES;
                    bgfx::setState(lineState);
                    float opaque[4]    = {1.0f, 0.0f, 0.0f, 0.0f};
                    float whiteLight[4]= {1.0f, 1.0f, 1.0f, 0.0f};
                    float noFog[4]     = {0, 0, 0, 0};
                    bgfx::setUniform(gpu.u_fogColor,    noFog);
                    bgfx::setUniform(gpu.u_fogParams,   noFog);
                    bgfx::setUniform(gpu.u_objectParams, opaque);
                    bgfx::setUniform(gpu.u_objectLight,  whiteLight);
                    bgfx::submit(2, gpu.flatProgram);
                }

                // HUD lines: probe counts + layer-2 occlusion
                // classification breakdown. Colors match the legend:
                // green = visible to listener, red = occluded by
                // acoustic mesh (would be dropped by Steam Audio's
                // `checkOcclusion`), gray = out of influence sphere
                // (layer 1 would cull before layer 2 ran).
                bgfx::setDebug(BGFX_DEBUG_TEXT);
                bgfx::dbgTextPrintf(2, 16, 0x0F,
                    "Pathing probes: %zu total, %zu drawn (room filter: "
                    "%zu rooms via debug_room_max_count)",
                    probes.size(), drawList.size(), keepRooms.size());
                bgfx::dbgTextPrintf(2, 17, 0x0F,
                    "  layer-2 occl: %d visible (green), %d occluded "
                    "(red), %d out-of-range (gray)",
                    nVisible, nOccluded, nOutOfRange);
            }
        }
    }

    // ── Pathing-graph EQ-activity overlay (Capability C) ──
    //
    // Layers 1 + 2: static probe-visibility graph (dim gray topology)
    // tinted in active-voice neighborhoods by per-voice eqCoeffs.mid.
    // Layer 3: per-voice source→listener arrow (separate toggle).
    //
    // HONESTY NOTE: this is NOT a claim that Steam Audio used edge
    // (A,B) for voice X. The adjacency was computed by Darkness using
    // the same raycast algorithm Steam Audio's pathing baker uses
    // (visRange + numVisSamples). Layer-2 edge coloring is a
    // neighborhood-activity heatmap: across active voices within
    // visRadius of either endpoint, edge_block = max(1 - eqCoeffs.mid).
    // The public Steam Audio C API does not expose per-voice
    // visited-probe sets, and the BFS-inference approach was
    // explicitly rejected (PLAN.PROBE_DEBUG_TOOLING.md Capability C)
    // due to divergence risk. Per feedback_no_hacks.
    if (state.showPathingGraph || state.showVoiceArrows) {
        auto audioSvc = GET_SERVICE(Darkness::AudioService);
        if (audioSvc) {
            // Smoothing state — keep the previous frame's edge_block
            // array keyed by edge index. Linear interp toward the new
            // value at a fixed rate so colour changes are smooth
            // (per feedback_smooth_transitions; ~120 ms time constant
            // at 60 fps). Static so it persists across frames.
            static std::vector<float> sEdgeBlockPrev;

            std::vector<Darkness::AudioService::PathingEdgeViz> edges;
            int nGreen = 0, nRed = 0, nNeighbor = 0;
            int activeVoices = 0;
            if (state.showPathingGraph) {
                edges = audioSvc->getPathingEdgeViz();

                // Resize smoothing buffer if edge count changed (level
                // reload, fresh bake). Resets to 0 (background).
                if (sEdgeBlockPrev.size() != edges.size()) {
                    sEdgeBlockPrev.assign(edges.size(), 0.0f);
                }
                constexpr float kBlend = 0.20f;  // ~5-frame ease-in
                for (size_t i = 0; i < edges.size(); ++i) {
                    float target = edges[i].hasNeighbor
                        ? edges[i].edgeBlock : 0.0f;
                    sEdgeBlockPrev[i] += (target - sEdgeBlockPrev[i]) * kBlend;
                    edges[i].edgeBlock = sEdgeBlockPrev[i];
                    // Reclassify after smoothing — an edge tagged
                    // hasNeighbor=true at target=0 stays as a yellow
                    // sliver until the EWMA decays, which is the
                    // desired smooth-fade behaviour.
                    if (edges[i].hasNeighbor) {
                        if (edges[i].edgeBlock > 0.05f) {
                            ++nNeighbor;
                            if (edges[i].edgeBlock > 0.66f) ++nRed;
                            else if (edges[i].edgeBlock < 0.20f) ++nGreen;
                        }
                    }
                }
            }
            std::vector<Darkness::AudioService::VoiceArrowViz> arrows;
            if (state.showVoiceArrows) {
                arrows = audioSvc->getVoiceArrowViz();
            }
            // For HUD: count audio voices with any pathing data. Mirrors
            // the per-frame iteration getPathingEdgeViz already does.
            if (state.showPathingGraph) {
                for (const auto &a : audioSvc->getVoiceArrowViz()) {
                    if (a.everSolved) ++activeVoices;
                }
            }

            // Worst-case verts: 2 per edge + 2 per arrow.
            uint32_t needVerts = static_cast<uint32_t>(edges.size()) * 2u
                              + static_cast<uint32_t>(arrows.size()) * 2u;
            if (needVerts > 0
                && bgfx::getAvailTransientVertexBuffer(
                       needVerts, Darkness::PosColorVertex::layout) >= needVerts)
            {
                bgfx::TransientVertexBuffer tvb;
                bgfx::allocTransientVertexBuffer(
                    &tvb, needVerts, Darkness::PosColorVertex::layout);
                auto *verts = reinterpret_cast<Darkness::PosColorVertex *>(tvb.data);
                uint32_t n = 0;

                // Color helpers. ABGR (bgfx native order for the
                // PosColorVertex layout used elsewhere in this file).
                //   Dim gray   = 0x80808080  background topology
                //   Green      = 0xFF22FF22  clear path
                //   Yellow     = 0xFF22EEFFu  partial
                //   Red        = 0xFF2222FF  fully blocked (sentinel)
                // Layer-3 arrow uses the same green→yellow→red ramp
                // applied to the raw eqMid coefficient (1=clear→green).
                auto eqColor = [](float block) -> uint32_t {
                    // block ∈ [0,1] — invert to clarity for the ramp
                    if (block <= 0.0f) return 0xFF22FF22u;        // green
                    if (block >= 0.95f) return 0xFF2222FFu;       // red
                    if (block < 0.33f) {
                        // green → yellow
                        return 0xFF22EEFFu;
                    }
                    if (block < 0.66f) return 0xFF22DDEEu;        // amber
                    return 0xFF2244FFu;                            // orange-red
                };

                // Layer 1 — every edge in the adjacency (dim gray when
                // no neighbor voice). Layer 2 overrides the tint when
                // an active voice is in the endpoint neighborhood.
                if (state.showPathingGraph) {
                    for (const auto &e : edges) {
                        uint32_t c = 0x80808080u;  // dim gray default
                        if (e.hasNeighbor) c = eqColor(e.edgeBlock);
                        verts[n++] = { e.posA.x, e.posA.y, e.posA.z, c };
                        verts[n++] = { e.posB.x, e.posB.y, e.posB.z, c };
                    }
                }

                // Layer 3 — per-voice arrows. Color by eqMid directly
                // (no smoothing pass — voices come and go and a smooth
                // fade on a brand-new voice would lag perceptibly).
                if (state.showVoiceArrows) {
                    for (const auto &a : arrows) {
                        float block = a.everSolved
                            ? (1.0f - std::max(0.0f, std::min(a.eqMid, 1.0f)))
                            : 0.0f;
                        // Unsolved voices get a faded cyan so they read
                        // as "still resolving" rather than green-clear.
                        uint32_t c = a.everSolved ? eqColor(block)
                                                  : 0x88FFFF00u;  // semi-transp cyan
                        verts[n++] = { a.source.x,   a.source.y,   a.source.z,   c };
                        verts[n++] = { a.listener.x, a.listener.y, a.listener.z, c };
                    }
                }

                if (n > 0) {
                    float identity[16];
                    bx::mtxIdentity(identity);
                    bgfx::setTransform(identity);
                    bgfx::setVertexBuffer(0, &tvb, 0, n);
                    uint64_t lineState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                                       | BGFX_STATE_PT_LINES
                                       | BGFX_STATE_BLEND_ALPHA
                                       | BGFX_STATE_DEPTH_TEST_ALWAYS;
                    bgfx::setState(lineState);
                    float noFog[4]      = {0, 0, 0, 0};
                    float opaqueParams[4] = {1.0f, 0.0f, 0.0f, 0.0f};
                    float whiteLight[4]   = {1.0f, 1.0f, 1.0f, 0.0f};
                    bgfx::setUniform(gpu.u_fogColor,    noFog);
                    bgfx::setUniform(gpu.u_fogParams,   noFog);
                    bgfx::setUniform(gpu.u_objectParams, opaqueParams);
                    bgfx::setUniform(gpu.u_objectLight,  whiteLight);
                    bgfx::submit(2, gpu.flatProgram);
                }
            }

            // HUD line. Anchored at row 19 so it doesn't collide with
            // the show_probes (rows 14-15) or show_probe_radius
            // (rows 16-17) blocks. Surfaces the no-pathing-batch state
            // explicitly (per feedback_no_silent_fallbacks).
            bgfx::setDebug(BGFX_DEBUG_TEXT);
            const uint8_t labelAttr = 0x0F;  // white on black
            const uint8_t valAttr   = 0x0A;  // green
            const uint8_t warnAttr  = 0x0C;  // red
            const auto &pp = audioSvc->getPathingProbePositions();
            if (pp.empty()) {
                bgfx::dbgTextPrintf(2, 19, warnAttr,
                    "Pathing graph: NO PATHING BATCH (run bake_probes)");
            } else if (edges.empty() && state.showPathingGraph) {
                // Adjacency hasn't been built yet (raycaster wasn't
                // wired at load time and rebuild never fired) — direct
                // the user to the [VIZ_FALLBACK] stderr line.
                bgfx::dbgTextPrintf(2, 19, warnAttr,
                    "Pathing graph: %zu probes loaded, adjacency NOT BUILT "
                    "(see [VIZ_FALLBACK] in stderr)", pp.size());
            } else {
                bgfx::dbgTextPrintf(2, 19, valAttr,
                    "Pathing graph: %zu probes / %zu edges | %d edges "
                    "carrying sound (green) / %d blocked (red) | "
                    "neighborhood-tinted=%d | active voices = %d",
                    pp.size(), edges.size(),
                    nGreen, nRed, nNeighbor, activeVoices);
            }
            if (state.showVoiceArrows) {
                bgfx::dbgTextPrintf(2, 20, labelAttr,
                    "Voice arrows: %zu (green=clear, red=blocked, "
                    "cyan=unresolved)", arrows.size());
            }
        }
    }
}

// ── Object rendering ──
// Two-pass rendering for correct transparency compositing:
//   Pass 1: all opaque submeshes (matTrans == 0 AND renderAlpha == 1.0)
//   Pass 2: all translucent submeshes (matTrans > 0 OR renderAlpha < 1.0)
// A single object (e.g. a window) can have both opaque submeshes (frame)
// and translucent submeshes (glass pane), so transparency is per-submesh.
//
// Final alpha = (1.0 - material.trans) * object.renderAlpha
// where material.trans is from .bin mat_extra (0=opaque, 0.3=glass)
// and object.renderAlpha is from P$RenderAlp property (1.0=opaque)
static void renderObjects(
    const Darkness::FrameContext &fc,
    const Darkness::GPUResources &gpu,
    const Darkness::MissionData &mission,
    const Darkness::RuntimeState &state)
{
    if (!state.showObjects) return;

    // Alpha-blend state for translucent submeshes
    uint64_t translucentState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                              | BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS
                              | BGFX_STATE_CULL_CW
                              | BGFX_STATE_BLEND_ALPHA;

    // Helper: set fog uniforms (bgfx clears uniforms after each submit)
    float opaqueParams[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    float whiteLight[4]   = { 1.0f, 1.0f, 1.0f, 0.0f };
    auto setFogOn = [&]() {
        bgfx::setUniform(gpu.u_fogColor, fc.fogColorArr);
        bgfx::setUniform(gpu.u_fogParams, fc.fogOnArr);
        bgfx::setUniform(gpu.u_objectParams, opaqueParams);
        bgfx::setUniform(gpu.u_objectLight, whiteLight);
    };

    // Helper: draw one object's submeshes, filtering by opacity.
    // When opaquePass=true, draws only opaque submeshes with fc.renderState.
    // When opaquePass=false, draws only translucent submeshes with translucentState.
    auto drawObjectSubmeshes = [&](size_t oi, bool opaquePass) {
        const auto &obj = mission.objData.objects[oi];
        if (!obj.hasPosition) return;

        // Portal culling: skip objects in non-visible cells, unless the
        // object's world-space AABB still intersects the rendering frustum.
        // This prevents large objects from popping when their center-of-mass
        // cell passes behind the camera but part of the mesh is still visible.
        if (state.portalCulling && oi < mission.objCellIDs.size()) {
            int32_t objCell = mission.objCellIDs[oi];
            if (objCell >= 0 && fc.visibleCells.count(static_cast<uint32_t>(objCell)) == 0) {
                // Cell not visible — check if model AABB intersects frustum
                std::string mname(obj.modelName);
                auto mit = mission.parsedModels.find(mname);
                if (mit == mission.parsedModels.end()) return;
                const auto &mesh = mit->second;

                // Compute conservative world-space AABB from model bbox + position.
                // Ignores rotation for speed — the resulting box is larger than
                // the true oriented bbox, so we never cull something visible.
                // Use runtime position if available (moving doors/platforms).
                float cx = obj.x, cy = obj.y, cz = obj.z;
                float csx = obj.scaleX, csy = obj.scaleY, csz = obj.scaleZ;
                const Darkness::ObjectState *cullState = state.objectStates
                    ? state.objectStates->tryGet(obj.objID) : nullptr;
                if (cullState) {
                    cx = cullState->position.x; cy = cullState->position.y; cz = cullState->position.z;
                    csx = cullState->scale.x; csy = cullState->scale.y; csz = cullState->scale.z;
                }
                float halfX = (mesh.bboxMax[0] - mesh.bboxMin[0]) * 0.5f * csx;
                float halfY = (mesh.bboxMax[1] - mesh.bboxMin[1]) * 0.5f * csy;
                float halfZ = (mesh.bboxMax[2] - mesh.bboxMin[2]) * 0.5f * csz;
                float extent = std::max({halfX, halfY, halfZ});  // sphere-ish bound
                if (!fc.objFrustum.testAABB(
                        cx - extent, cy - extent, cz - extent,
                        cx + extent, cy + extent, cz + extent))
                    return;
            }
        }

        // Compute per-object model matrix. If the object has a runtime state
        // override (door opening, platform moving, tweq animating), use that
        // transform instead of the static P$Position from the .mis file.
        float objMtx[16];
        const Darkness::ObjectState *objState = state.objectStates
            ? state.objectStates->tryGet(obj.objID) : nullptr;
        if (objState && !(objState->flags & Darkness::kObjStateDestroyed)) {
            if (objState->flags & Darkness::kObjStateHidden)
                return;  // skip hidden objects (blink tweq, etc.)
            if (objState->hasMatrix) {
                // Use pre-built matrix directly — avoids lossy Euler extraction
                std::memcpy(objMtx, objState->modelMatrix, sizeof(objMtx));
            } else {
                // Build matrix from runtime state angles (already in radians)
                const float negH = -objState->heading;
                const float negP = -objState->pitch;
                const float negB = -objState->bank;
                bx::mtxRotateXYZ(objMtx, negB, negP, negH);
                objMtx[ 0] *= objState->scale.x; objMtx[ 1] *= objState->scale.x; objMtx[ 2] *= objState->scale.x;
                objMtx[ 4] *= objState->scale.y; objMtx[ 5] *= objState->scale.y; objMtx[ 6] *= objState->scale.y;
                objMtx[ 8] *= objState->scale.z; objMtx[ 9] *= objState->scale.z; objMtx[10] *= objState->scale.z;
                objMtx[12] = objState->position.x;
                objMtx[13] = objState->position.y;
                objMtx[14] = objState->position.z;
            }
        } else {
            buildModelMatrix(objMtx, obj.x, obj.y, obj.z,
                             obj.heading, obj.pitch, obj.bank,
                             obj.scaleX, obj.scaleY, obj.scaleZ);
        }

        std::string modelName(obj.modelName);

        // Check for tweq model override (model cycling animation)
        if (objState && !objState->modelNameOverride.empty()) {
            modelName = objState->modelNameOverride;
        }

        // Model isolation mode: skip objects that don't match the isolated model
        if (state.isolateModelIdx >= 0 && state.isolateModelIdx < (int)state.sortedModelNames.size()) {
            if (modelName != state.sortedModelNames[state.isolateModelIdx])
                return;
        }

        auto it = gpu.objModelGPU.find(modelName);

        if (it != gpu.objModelGPU.end() && it->second.valid) {
            const auto &gpuModel = it->second;

            // Live world-space position for object lighting. Moving objects
            // (doors/platforms/tweqs) need their current position, not the
            // P$Position baseline; stationary objects fall through to obj.x/y/z.
            float litX = obj.x, litY = obj.y, litZ = obj.z;
            if (objState) {
                litX = objState->position.x;
                litY = objState->position.y;
                litZ = objState->position.z;
            }
            // Cell hint: stationary objects have a precomputed cell in
            // objCellIDs (built once at load); moving objects fall back to
            // the cell-find inside ObjectIlluminator.
            int32_t litCell = -1;
            if (!objState && oi < mission.objCellIDs.size())
                litCell = mission.objCellIDs[oi];

            // Object radius (used to offset the virtual sun position) is
            // the bounding sphere radius from the .bin header, scaled by
            // the object's per-axis scale (taking the largest scale to
            // stay conservative — the sun should be outside the object).
            const auto *parsed = mission.parsedModels.count(modelName)
                ? &mission.parsedModels.at(modelName) : nullptr;
            float litRadius = 1.0f;
            if (parsed) {
                float maxScale = std::max({obj.scaleX, obj.scaleY, obj.scaleZ});
                if (objState) {
                    maxScale = std::max({objState->scale.x,
                                         objState->scale.y,
                                         objState->scale.z});
                }
                litRadius = parsed->sphereRadius * std::max(maxScale, 1.0f);
            }

            // Two lighting paths share the same visibility/sun/cache logic
            // inside ObjectIlluminator; they only differ in what they emit
            // and which programs/uniforms the renderer feeds:
            //
            //   - Per-vertex (default when enabled): buildLightArray() packs
            //     the cell's visible lights into a GPU array. Vertex shader
            //     applies cos(angle)/dist per vertex → front/back contrast.
            //
            //   - Scalar fallback: compute() returns one RGB value, the
            //     fragment shader multiplies it across the whole object.
            //
            // When `objectLightingEnabled` is off we pass white through the
            // scalar path so the look matches the pre-lighting historical
            // appearance (a no-op multiply).
            const bool useScalarPath =
                !state.objectLightingEnabled ||
                !state.perVertexObjectLightingEnabled;

            Darkness::Vector3 litRGB(1.0f, 1.0f, 1.0f);
            if (useScalarPath) {
                if (state.objectLightingEnabled) {
                    litRGB = state.objectIlluminator.compute(
                        obj.objID,
                        Darkness::Vector3(litX, litY, litZ),
                        litRadius,
                        litCell);
                }
            } else {
                state.objectIlluminator.buildLightArray(
                    obj.objID,
                    Darkness::Vector3(litX, litY, litZ),
                    litRadius,
                    litCell,
                    state.gpuLightScratch);
            }
            float litArr[4] = { litRGB.x, litRGB.y, litRGB.z, 0.0f };

            for (const auto &sm : gpuModel.subMeshes) {
                if (sm.indexCount == 0) continue;

                // Held objects render fully opaque; visibility past them
                // comes from CarryParams::heightOffset placing the object
                // below the crosshair so the player can see over it.
                bool isTranslucent = (sm.matTrans > 0.0f) || (obj.renderAlpha < 1.0f);
                if (opaquePass == isTranslucent) continue;  // wrong pass

                // Compute final alpha: (1 - matTrans) * renderAlpha
                // matTrans convention: 0=opaque, 0.3=30% transparent glass
                float finalAlpha = (1.0f - sm.matTrans) * obj.renderAlpha;
                // Frob highlight: additive brightness for the targeted object
                float highlight = (obj.objID == state.frobHighlightObjID)
                    ? state.frobHighlightLevel : 0.0f;
                // Per-material self-illumination: additive floor consumed by
                // the per-vertex shader so a lamp's casing glows even when its
                // outward-facing normals leave the interior light below
                // Lambertian's half-space cutoff. The scalar shaders ignore
                // this slot — they get the same effect via the load-time
                // baked vertex tint. Future enhancement: multiply by
                // P$SelfIllum per-object scale (default 1.0).
                float objAlpha[4] = { finalAlpha, highlight, sm.matIllum, 0.0f };

                uint64_t drawState = opaquePass ? fc.renderState : translucentState;

                bgfx::setUniform(gpu.u_fogColor, fc.fogColorArr);
                bgfx::setUniform(gpu.u_fogParams, fc.fogOnArr);
                bgfx::setUniform(gpu.u_objectParams, objAlpha);
                bgfx::setUniform(gpu.u_objectLight, litArr);

                if (!useScalarPath) {
                    // Per-vertex: feed the GPU light array. The shader reads
                    // only [0, count) so leftover entries from prior draws
                    // are harmless. SoA layout means each .loc/.dir/.bright
                    // array is a contiguous run of vec4s — exactly what
                    // bgfx::setUniform expects for an array uniform.
                    const auto &arr = state.gpuLightScratch;
                    float countV[4]   = { (float)arr.count, 0.0f, 0.0f, 0.0f };
                    float ambientV[4] = { arr.ambient.x, arr.ambient.y,
                                          arr.ambient.z, 0.0f };
                    bgfx::setUniform(gpu.u_objectLightCount,  countV);
                    bgfx::setUniform(gpu.u_objectAmbient,     ambientV);
                    if (arr.count > 0) {
                        bgfx::setUniform(gpu.u_objectLightLoc,
                            arr.loc, arr.count);
                        bgfx::setUniform(gpu.u_objectLightDir,
                            arr.dir, arr.count);
                        bgfx::setUniform(gpu.u_objectLightBright,
                            arr.bright, arr.count);
                    }
                }

                bgfx::setTransform(objMtx);
                bgfx::setVertexBuffer(0, gpuModel.vbh);
                bgfx::setIndexBuffer(gpuModel.ibh, sm.firstIndex, sm.indexCount);
                bgfx::setState(drawState);

                bgfx::ProgramHandle texturedProg = useScalarPath
                    ? gpu.texturedProgram : gpu.texturedPerVertexProgram;
                bgfx::ProgramHandle flatProg = useScalarPath
                    ? gpu.flatProgram : gpu.basicPerVertexProgram;

                if (sm.textured) {
                    auto texIt = gpu.objTextureHandles.find(sm.matName);
                    if (texIt != gpu.objTextureHandles.end()) {
                        bgfx::setTexture(0, gpu.s_texColor, texIt->second, fc.texSampler);
                        bgfx::submit(1, texturedProg);
                    } else {
                        bgfx::submit(1, flatProg);
                    }
                } else {
                    bgfx::submit(1, flatProg);
                }
            }
        } else if (state.showFallbackCubes && opaquePass) {
            // Fallback cubes are always opaque
            setFogOn();
            bgfx::setTransform(objMtx);
            bgfx::setVertexBuffer(0, gpu.fallbackCubeVBH);
            bgfx::setIndexBuffer(gpu.fallbackCubeIBH);
            bgfx::setState(fc.renderState);
            bgfx::submit(1, gpu.flatProgram);
        }
    };

    // Pass 1: opaque submeshes of all objects
    for (size_t oi = 0; oi < mission.objData.objects.size(); ++oi) {
        drawObjectSubmeshes(oi, true);
    }
    // Pass 2: translucent submeshes — rendered after all opaque geometry
    for (size_t oi = 0; oi < mission.objData.objects.size(); ++oi) {
        drawObjectSubmeshes(oi, false);
    }
}


// ── Update window title bar ──
// Displays current render mode, speed, culling stats, filter mode, and model isolation info.
static void updateTitleBar(SDL_Window *window, const Darkness::RuntimeState &state) {
    char title[512];
    const char *filterNames[] = { "point", "bilinear", "trilinear", "aniso" };
    const char *filterStr = filterNames[state.filterMode % 4];
    const char *lmNames[] = { "bilinear", "bicubic" };
    const char *lmStr = lmNames[state.lightmapFiltering % 2];

    // Build model isolation suffix if active
    char isoSuffix[128] = "";
    if (state.isolateModelIdx >= 0 && state.isolateModelIdx < (int)state.sortedModelNames.size()) {
        const auto &isoName = state.sortedModelNames[state.isolateModelIdx];
        auto cit = state.modelInstanceCounts.find(isoName);
        int cnt = (cit != state.modelInstanceCounts.end()) ? cit->second : 0;
        std::snprintf(isoSuffix, sizeof(isoSuffix),
            " [MODEL %d/%zu: %s x%d]",
            state.isolateModelIdx + 1, state.sortedModelNames.size(),
            isoName.c_str(), cnt);
    }

    // Physics mode shows "walk" / camera collision shows "clip" / default "noclip"
    const char *moveStr = state.physicsMode ? "walk" :
                          state.cameraCollision ? "clip" : "noclip";

    // In physics mode, show velocity and ground state for debugging
    char physSuffix[128] = "";
    if (state.physicsMode && state.physics) {
        Darkness::Vector3 vel = state.physics->getPlayerVelocity();
        float speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
        const char *groundStr = state.physics->isPlayerOnGround() ? "ground" : "air";
        // Show player mode + speed modifier
        const auto &pp = state.physics->getPlayerPhysics();
        static const char *modeNames[] = {
            "stand", "crouch", "swim", "climb", "carry", "slide", "jump", "dead"
        };
        int modeIdx = static_cast<int>(pp.getMode());
        const char *modeName = (modeIdx >= 0 && modeIdx < 8) ? modeNames[modeIdx] : "?";
        const char *speedMode = pp.isSneaking() ? "sneak" :
                                pp.isRunning()  ? "run" : modeName;
        std::snprintf(physSuffix, sizeof(physSuffix),
            " [%s %s %.1f u/s cell:%d %.0fHz]", groundStr, speedMode, speed,
            state.physics->getPlayerCell(), state.physics->getPhysicsHz());
    }

    int dimmedLights = state.objectIlluminator.dimmedLightCount();
    char dimSuffix[32] = "";
    if (dimmedLights > 0)
        std::snprintf(dimSuffix, sizeof(dimSuffix), " [dim:%d]", dimmedLights);

    if (state.portalCulling) {
        std::snprintf(title, sizeof(title),
            "darkness — %s [speed: %.1f] [cull: %u/%u cells] [%s] [lm:%s] [%s]%s%s%s",
            state.modeStr, state.moveSpeed, state.cullVisibleCells, state.cullTotalCells, filterStr, lmStr, moveStr, isoSuffix, physSuffix, dimSuffix);
    } else {
        std::snprintf(title, sizeof(title),
            "darkness — %s [speed: %.1f] [cull: OFF] [%s] [lm:%s] [%s]%s%s%s",
            state.modeStr, state.moveSpeed, filterStr, lmStr, moveStr, isoSuffix, physSuffix, dimSuffix);
    }
    SDL_SetWindowTitle(window, title);
}

// ── Register debug console settings ──
// Binds 15 runtime-changeable settings to the debug console (opened with backtick).
// Lambdas capture state by reference and update the title bar when relevant settings change.
// ── Progress-bar bake helper ──
// Runs bakeProbes() on a background thread while pumping a bgfx debug-text
// progress bar on the main thread (so the window stays responsive and the
// user can see the bake actually running). Used by both the first-run
// auto-bake and the console-triggered `bake_probes on` action; the SAME
// helper for both paths avoids the foot-gun where one path freezes the
// window. Returns true if the bake completed and probes were reloaded.
static bool runBakeWithProgressBar(
    SDL_Window *window,
    Darkness::RuntimeState &state,
    Darkness::AudioServicePtr audioSvc,
    const std::string &bakePath)
{
    if (!audioSvc) return false;
    (void)window;  // event-pumping uses SDL_PollEvent, not the window ptr

    std::atomic<float> bakeProgress{0.0f};
    std::atomic<bool>  bakeDone{false};
    bool bakeSuccess = false;

    std::thread bakeThread([&]() {
        bakeSuccess = audioSvc->bakeProbes(bakePath, &bakeProgress);
        bakeDone.store(true, std::memory_order_release);
    });

    // The bake runs 3 sequential stages, each reporting 0→1 progress.
    // We detect stage transitions when progress drops and map to overall
    // 0→100% with weighted allocation (stages 0+1 = 20%, stage 2 = 80%).
    int   bakeStage = 0;
    float lastProg  = 0.0f;
    const char *stageNames[] = {
        "Baking pathing visibility...",
        "Computing shortest paths...",
        "Baking reflection IRs (this takes a while)..."
    };
    const float stageStart[]  = {0.0f, 0.10f, 0.20f};
    const float stageWeight[] = {0.10f, 0.10f, 0.80f};

    while (!bakeDone.load(std::memory_order_acquire)) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) {
                state.running = false;
                break;
            }
        }
        if (!state.running) break;

        float rawProg = bakeProgress.load(std::memory_order_relaxed);
        if (rawProg < lastProg - 0.1f && bakeStage < 2) bakeStage++;
        lastProg = rawProg;

        float prog = stageStart[bakeStage] + rawProg * stageWeight[bakeStage];
        prog = std::min(prog, 1.0f);
        int pct = static_cast<int>(prog * 100.0f);

        // Render progress bar using bgfx debug text on view 0
        bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                           0x1a1a2eFF, 1.0f, 0);
        bgfx::setViewRect(0, 0, 0, 1280, 720);
        bgfx::touch(0);

        bgfx::setDebug(BGFX_DEBUG_TEXT);
        bgfx::dbgTextClear();

        bgfx::dbgTextPrintf(2, 10, 0x0f, "DARKNESS ENGINE");
        bgfx::dbgTextPrintf(2, 12, 0x07, "Baking acoustic probes (stage %d/3):",
                            bakeStage + 1);
        bgfx::dbgTextPrintf(2, 13, 0x07, "  %s", stageNames[bakeStage]);

        int barWidth = 40;
        int filled = static_cast<int>(prog * barWidth);
        char bar[64] = {};
        for (int i = 0; i < barWidth; ++i)
            bar[i] = (i < filled) ? '#' : '-';
        bar[barWidth] = '\0';

        bgfx::dbgTextPrintf(2, 15, 0x0a, "[%s] %d%%", bar, pct);
        bgfx::dbgTextPrintf(2, 17, 0x08, "Probe data will be cached in:");
        bgfx::dbgTextPrintf(2, 18, 0x08, "  %s", bakePath.c_str());

        bgfx::frame();
        SDL_Delay(50);  // ~20fps progress display
    }

    bakeThread.join();
    bgfx::setDebug(0);

    if (bakeSuccess && state.running) {
        audioSvc->loadProbes(bakePath);
        std::fprintf(stderr, "Probe baking complete — %d probes loaded\n",
                     audioSvc->getProbeCount());
        return true;
    }
    return false;
}

static void registerConsoleSettings(
    Darkness::DebugConsole &dbgConsole,
    Darkness::RuntimeState &state,
    SDL_Window *window,
    const std::string &misPath = "")
{
    // Helper lambda that captures window+state for title bar refresh
    auto refreshTitle = [window, &state]() { updateTitleBar(window, state); };

    // ── Rendering ──

    dbgConsole.setGroup("Rendering");

    // Naming mirrors graphics.texture_filter / graphics.lightmap_filter in YAML.
    // Cycling exposes all four texture modes including anisotropic.
    dbgConsole.addCategorical("texture_filter",
        {"point", "bilinear", "trilinear", "anisotropic"},
        [&state]() { return state.filterMode; },
        [&state, refreshTitle](int v) { state.filterMode = v; refreshTitle(); });

    dbgConsole.addCategorical("lightmap_filter",
        {"bilinear", "bicubic"},
        [&state]() { return state.lightmapFiltering; },
        [&state, refreshTitle](int v) { state.lightmapFiltering = v; refreshTitle(); });

    dbgConsole.addBool("portal_culling",
        [&state]() { return state.portalCulling; },
        [&state, refreshTitle](bool v) { state.portalCulling = v; refreshTitle(); },
        "BFS portal traversal culling (reduces draw calls)");

    dbgConsole.addBool("show_objects",
        [&state]() { return state.showObjects; },
        [&state](bool v) { state.showObjects = v; },
        "Render object meshes (.bin models from obj.crf)");

    dbgConsole.addBool("show_fallback_cubes",
        [&state]() { return state.showFallbackCubes; },
        [&state](bool v) { state.showFallbackCubes = v; },
        "Show colored cubes for objects with missing models");

    dbgConsole.addBool("show_raycast",
        [&state]() { return state.showRaycast; },
        [&state](bool v) { state.showRaycast = v; },
        "Debug ray visualization from camera center");

    dbgConsole.addBool("show_acoustic_mesh",
        [&state]() { return state.showAcousticMesh; },
        [&state](bool v) { state.showAcousticMesh = v; },
        "Cyan wireframe overlay of the acoustic scene geometry");

    dbgConsole.addBool("show_door_geometry",
        [&state]() { return state.showDoorGeometry; },
        [&state](bool v) { state.showDoorGeometry = v; },
        "Orange wireframe overlay of all doors registered with the audio scene (IPLInstancedMesh). Complements show_acoustic_mesh — doors are the only dynamic geometry contributing to Steam Audio's pathing graph and are NOT in the static acoustic mesh. Use to verify whether a probe's reported isolation correlates with a door OBB sitting between it and its neighbors.");

    dbgConsole.addBool("show_acoustic_hit",
        [&state]() { return state.showAcousticHit; },
        [&state](bool v) { state.showAcousticHit = v; },
        "Raycast from camera forward against the acoustic mesh; highlight the closest hit triangle in red. HUD shows hit distance + texture name. Use to scan for holes: if you look at a wall and no triangle highlights within range, the acoustic mesh is missing geometry there (likely a BSP-split non-rendered portal silently excluded from the mesh).");

    dbgConsole.addBool("show_rooms",
        [&state]() { return state.showRooms; },
        [&state](bool v) { state.showRooms = v; },
        "Wireframe overlay of room OBBs and portal polygons, with room ID labels. Current room marked with '*'.");

    dbgConsole.addBool("show_pos",
        [&state]() { return state.showPos; },
        [&state](bool v) { state.showPos = v; },
        "Show camera world-space (x, y, z) position on the HUD. Z-up Dark Engine coords, units = feet.");

    dbgConsole.addBool("show_portals",
        [&state]() { return state.showPortals; },
        [&state](bool v) { state.showPortals = v; },
        "Highlight portal polygons in light pink (independent of show_rooms). Useful for tracing BFS audio paths through the level geometry.");

    // Cap how many rooms participate in the show_rooms / show_portals
    // overlays, picked by camera distance to room center. Levels with
    // hundreds of rooms (the typical Thief 2 mission) become unreadable
    // when every wireframe + ID label is drawn at once; restricting to
    // a handful nearest the camera keeps the overlay legible while you
    // walk around. "all" disables the cull.
    {
        // Stored as int on RuntimeState; categorical maps an option
        // index in [0..options.size()) onto our cap value. "all" → 0
        // (the sentinel selectClosestRooms reads as "no limit").
        static const std::vector<std::string> kCountOptions = {
            "5", "10", "20", "50", "100", "all"
        };
        static const std::vector<int> kCountValues = {5, 10, 20, 50, 100, 0};

        dbgConsole.addCategorical("debug_room_max_count", kCountOptions,
            [&state]() -> int {
                for (size_t i = 0; i < kCountValues.size(); ++i) {
                    if (kCountValues[i] == state.debugRoomMaxCount)
                        return static_cast<int>(i);
                }
                return 2;  // fall back to "20" (the default)
            },
            [&state](int idx) {
                if (idx >= 0 && idx < static_cast<int>(kCountValues.size()))
                    state.debugRoomMaxCount = kCountValues[idx];
            },
            "Cap rooms drawn by show_rooms / show_portals overlays to the camera-nearest N. 'all' disables the cull. Reduces visual clutter in dense levels.");
    }

    dbgConsole.addBool("show_vpos",
        [&state]() { return state.showVPos; },
        [&state](bool v) { state.showVPos = v; },
        "Per-voice spatial overlay. Cyan: source -> virtualPosition (anchor). Yellow: virtualPosition -> listener (what Steam Audio sees as the source distance). Yellow segment jumping = chain switch causing the audible volume pulse.");

    dbgConsole.addBool("debug_anim_lightmaps",
        [&state]() { return state.debugAnimLightmaps; },
        [&state](bool v) { state.debugAnimLightmaps = v; },
        "Tint animated lightmap polygons magenta for visibility");

    dbgConsole.addBool("force_flicker",
        [&state]() { return state.forceFlicker; },
        [&state](bool v) { state.forceFlicker = v; },
        "Override all animated lights to flicker mode");

    dbgConsole.addBool("object_lighting",
        [&state]() { return state.objectLightingEnabled; },
        [&state](bool v) {
            state.objectLightingEnabled = v;
            // Drop the cache so the next compute() rebuilds visibility from
            // scratch (in case the toggle is being used to compare results).
            state.objectIlluminator.clear();
        },
        "Per-object illumination from cell light list (off = flat material color)");

    dbgConsole.addBool("per_vertex_object_lighting",
        [&state]() { return state.perVertexObjectLightingEnabled; },
        [&state](bool v) { state.perVertexObjectLightingEnabled = v; },
        "Per-vertex Lambertian object shading (off = single scalar tint per object)");

    // Model isolation: "all" = show everything, then one entry per loaded model name.
    // sortedModelNames is populated during init before this registration runs.
    {
        std::vector<std::string> modelOpts = {"all"};
        modelOpts.insert(modelOpts.end(),
                         state.sortedModelNames.begin(),
                         state.sortedModelNames.end());
        dbgConsole.addCategorical("isolate_model", modelOpts,
            [&state]() { return state.isolateModelIdx + 1; },  // -1 → 0 ("all")
            [&state, window](int v) {
                state.isolateModelIdx = v - 1;  // 0 → -1 ("all")
                if (state.isolateModelIdx >= 0 &&
                    state.isolateModelIdx < static_cast<int>(state.sortedModelNames.size())) {
                    const auto &isoName = state.sortedModelNames[state.isolateModelIdx];
                    auto cit = state.modelInstanceCounts.find(isoName);
                    int cnt = (cit != state.modelInstanceCounts.end()) ? cit->second : 0;
                    std::fprintf(stderr, "Isolating model [%d/%zu]: '%s' (%d instances)\n",
                                 state.isolateModelIdx + 1, state.sortedModelNames.size(),
                                 isoName.c_str(), cnt);
                } else {
                    std::fprintf(stderr, "Model isolation: OFF (showing all)\n");
                }
                updateTitleBar(window, state);
            });
    }

    // ── Movement ──

    dbgConsole.setGroup("Movement");

    dbgConsole.addBool("physics_mode",
        [&state]() { return state.physicsMode; },
        [&state, refreshTitle](bool v) {
            state.physicsMode = v;
            state.crouchToggled = false;
            if (v && state.physics) {
                Darkness::Vector3 bodyPos(state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
                state.physics->setPlayerPosition(bodyPos);
                state.physics->setPlayerYaw(state.cam.yaw);
            }
            refreshTitle();
        },
        "Walk mode (on) vs fly/noclip mode (off)");

    dbgConsole.addCategorical("physics_rate",
        {"12.5Hz (vintage)", "60Hz (modern)", "120Hz (ultra)"},
        [&state]() {
            if (!state.physics) return 1;
            float hz = state.physics->getPhysicsHz();
            if (hz < 20.0f) return 0;       // vintage
            if (hz > 100.0f) return 2;       // ultra
            return 1;                         // modern
        },
        [&state, refreshTitle](int v) {
            if (!state.physics) return;
            auto &pp = state.physics->getPlayerPhysics();
            if (v == 0) pp.setTimestep(Darkness::VINTAGE);
            else if (v == 2) pp.setTimestep(Darkness::ULTRA);
            else pp.setTimestep(Darkness::MODERN);
            refreshTitle();
        });

    dbgConsole.addBool("camera_collision",
        [&state]() { return state.cameraCollision; },
        [&state, refreshTitle](bool v) { state.cameraCollision = v; refreshTitle(); },
        "Sphere collision against world geometry (noclip when off)");

    dbgConsole.addFloat("move_speed", 1.0f, 500.0f,
        [&state]() { return state.moveSpeed; },
        [&state, refreshTitle](float v) { state.moveSpeed = v; refreshTitle(); },
        "Camera/fly movement speed (world units/sec)");

    dbgConsole.addBool("step_log",
        [&state]() { return state.physics ? state.physics->getPlayerPhysics().stepLogEnabled() : false; },
        [&state](bool v) { if (state.physics) state.physics->getPlayerPhysics().setStepLog(v); },
        "Log stair-step diagnostics to stderr ([STEP] prefix)");

    dbgConsole.addBool("audio_log",
        []() { return Darkness::gAudioLogVerbose; },
        [](bool v) { Darkness::gAudioLogVerbose = v; },
        "Audio/sound/schema log output (off by default)");

    dbgConsole.addBool("physics_log",
        [&state]() { return state.physics ? state.physics->getPlayerPhysics().isLogging() : false; },
        [&state](bool v) {
            if (!state.physics) return;
            auto &player = state.physics->getPlayerPhysics();
            if (v) player.startLog("physics_log.csv");
            else   player.stopLog();
        },
        "Write per-timestep physics data to physics_log.csv");

    dbgConsole.addBool("head_log",
        [&state]() { return state.headLog != nullptr; },
        [&state](bool v) {
            if (v) openHeadLog(state, "head_log.csv");
            else   closeHeadLog(state);
        },
        "Write per-render-frame head/viewport state to head_log.csv");

    // Auto-fly probe-tour toggle. Flipping on snapshots N-nearest probes
    // from current camera position and starts the deterministic shuffle.
    // Flipping off releases the camera back to manual fly control.
    dbgConsole.addBool("auto_fly",
        [&state]() { return state.autoFly.enabled; },
        [&state](bool v) { state.autoFly.enabled = v; },
        "Deterministic probe-tour flythrough (forces fly mode)");

    dbgConsole.addFloat("auto_fly_speed", 0.5f, 50.0f,
        [&state]() { return state.autoFly.speed; },
        [&state](float v) { state.autoFly.speed = v; },
        "Auto-fly travel speed (world units/sec, ft/s)");

    dbgConsole.addFloat("auto_fly_pause_sec", 0.0f, 30.0f,
        [&state]() { return state.autoFly.pauseAtWaypointSec; },
        [&state](float v) { state.autoFly.pauseAtWaypointSec = v; },
        "Auto-fly dwell time per waypoint (seconds; 0 = continuous)");

    // ── Water ──

    dbgConsole.setGroup("Water");

    dbgConsole.addFloat("wave_amplitude", 0.0f, 5.0f,
        [&state]() { return state.waveAmplitude; },
        [&state](float v) { state.waveAmplitude = v; },
        "Water vertex wave displacement (0=flat)");

    dbgConsole.addFloat("uv_distortion", 0.0f, 0.5f,
        [&state]() { return state.uvDistortion; },
        [&state](float v) { state.uvDistortion = v; },
        "Water UV texture wobble strength");

    dbgConsole.addFloat("water_rotation", 0.0f, 0.5f,
        [&state]() { return state.waterRotation; },
        [&state](float v) { state.waterRotation = v; },
        "Water UV rotation speed (rad/s)");

    dbgConsole.addFloat("water_scroll", 0.0f, 1.0f,
        [&state]() { return state.waterScrollSpeed; },
        [&state](float v) { state.waterScrollSpeed = v; },
        "Water UV scroll speed (world units/s)");

    // ── Audio ──

    dbgConsole.setGroup("Audio");

    dbgConsole.addBool("portal_routing",
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getPortalRoutingEnabled() : true;
        },
        [](bool v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setPortalRoutingEnabled(v);
        },
        "Route sound through doorways via portal graph (indirect paths)");

    // One-shot diagnostic: print the closest door's state, properties, the
    // audio-side blocking factor for its room pair, AND the portal topology
    // of both rooms with each portal's current blocking factor. If room1 has
    // an unblocked portal to room2 that bypasses this door, BFS will detour
    // around the door and `totalBlocking` will collapse to 0.
    dbgConsole.addBool("dump_nearest_door",
        []() { return false; },  // action, not a state
        [&state](bool v) {
            if (!v || !state.doorSystem) return;
            auto svc = GET_SERVICE(Darkness::AudioService);
            auto roomSvc = GET_SERVICE(Darkness::RoomService);
            const Darkness::Vector3 camPos(
                state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
            auto ids = state.doorSystem->getAllDoorIDs();
            int32_t bestID = 0;
            float   bestDist = std::numeric_limits<float>::infinity();
            for (int32_t id : ids) {
                const auto *d = state.doorSystem->getDoor(id);
                if (!d) continue;
                float dist = glm::length(d->basePosition - camPos);
                if (dist < bestDist) { bestDist = dist; bestID = id; }
            }
            if (bestID == 0) {
                std::fprintf(stderr, "[DOOR_DUMP] no doors in level\n");
                return;
            }
            const auto *d = state.doorSystem->getDoor(bestID);
            float openFrac = state.doorSystem->getOpenFraction(bestID);
            float audioBlock = svc
                ? svc->getBlockingFactor(d->room1, d->room2) : -1.0f;

            Darkness::Room *listenerRoom = roomSvc
                ? roomSvc->roomFromPoint(camPos) : nullptr;
            int listenerRoomID = listenerRoom ? listenerRoom->getRoomID() : -1;

            std::fprintf(stderr,
                "[DOOR_DUMP] obj=%d distFromCam=%.1f listenerRoom=%d\n"
                "  basePos=(%.1f,%.1f,%.1f) status=%d openFrac=%.2f\n"
                "  rooms=(%d,%d) sndBlock=%.2f visBlock=%d isBrush=%d\n"
                "  AudioService.getBlockingFactor(%d,%d) = %.3f\n",
                bestID, bestDist, listenerRoomID,
                d->basePosition.x, d->basePosition.y, d->basePosition.z,
                (int)d->status, openFrac,
                d->room1, d->room2, d->soundBlocking,
                (int)d->visionBlocking, (int)d->isBrushDoor,
                d->room1, d->room2, audioBlock);

            // Dump the portal topology of room1 and room2 so we can see if
            // there's an alternate (unblocked) edge between them that lets
            // the BFS bypass this door.
            if (roomSvc && svc) {
                auto dumpRoomPortals = [&](int32_t roomID) {
                    Darkness::Room *r = roomSvc->getRoomByID(roomID);
                    if (!r) {
                        std::fprintf(stderr, "  room %d: NOT FOUND in RoomService\n", roomID);
                        return;
                    }
                    uint32_t pc = r->getPortalCount();
                    std::fprintf(stderr, "  room %d has %u portals:\n", roomID, pc);
                    for (uint32_t i = 0; i < pc; ++i) {
                        Darkness::RoomPortal *p = r->getPortal(i);
                        if (!p) continue;
                        Darkness::Room *far = p->getFarRoom();
                        int32_t farID = far ? far->getRoomID() : -1;
                        float blk = svc->getBlockingFactor(roomID, farID);
                        std::fprintf(stderr,
                            "    [%u] -> room %d  blockingFactor=%.3f\n",
                            i, farID, blk);
                    }
                };
                dumpRoomPortals(d->room1);
                dumpRoomPortals(d->room2);
            }
        },
        "Print nearest door's state, blocking factor, and room portal topology");

    dbgConsole.addBool("probe_pathing",
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getProbePathingEnabled() : true;
        },
        [](bool v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setProbePathingEnabled(v);
        },
        "Baked probe diffraction/pathing (when probe data available)");

    dbgConsole.addBool("refl_enabled",
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getReflectionsEnabled() : false;
        },
        [](bool v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setReflectionsEnabled(v);
        },
        "Real-time convolution reverb (per-source room reflections)");

    dbgConsole.addFloat("refl_rays", 128.0f, 8192.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getRealtimeNumRays()) : 4096.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setRealtimeNumRays(static_cast<int>(v));
        },
        "Rays per realtime reflection sim step (background thread)");

    dbgConsole.addFloat("refl_bounces", 1.0f, 8.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getRealtimeNumBounces()) : 4.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setRealtimeNumBounces(static_cast<int>(v));
        },
        "Realtime bounces per ray (more = multi-room reverb propagation)");

    dbgConsole.addFloat("refl_duration", 0.5f, 4.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getRealtimeDuration() : 2.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setRealtimeDuration(v);
        },
        "Realtime reverb tail length in seconds (must exceed hybrid_transition_time)");

dbgConsole.addFloat("refl_throttle", 1.0f, 32.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getReflectionThrottle()) : 4.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setReflectionThrottle(static_cast<int>(v));
        },
        "Run reflection sim every Nth frame (higher = less CPU)");

    dbgConsole.addFloat("refl_tri_count", 0.0f, 999999.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getAcousticSceneTriCount()) : 0.0f;
        },
        [](float) { /* read-only */ },
        "Triangles in acoustic scene (read-only)");

    dbgConsole.addFloat("reverb_voices", 1.0f, 32.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getReverbVoices()) : 4.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setReverbVoices(static_cast<int>(v));
        },
        "Max active voices with reverb (realtime + baked combined)");

    dbgConsole.addFloat("refl_sample_rate", 0.0f, 48000.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getReflectionSampleRate()) : 48000.0f;
        },
        [](float) { /* read-only */ },
        "Reflection pipeline sample rate (read-only, set via YAML)");

    dbgConsole.addFloat("refl_ambi_order", 0.0f, 1.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getAmbisonicsOrder()) : 0.0f;
        },
        [](float) { /* read-only */ },
        "Ambisonics order: 0=omnidirectional 1=directional (read-only, set via YAML)");

    dbgConsole.addFloat("transmission_scale", 0.1f, 100.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getTransmissionScale() : 10.0f;
        },
        [](float) { /* read-only — set via YAML, requires scene rebuild */ },
        "Material transmission multiplier (1=physical, 10=audible through walls, set via YAML)");

    // ── Mixer gains (live A/B for direct vs. indirect levels) ──

    dbgConsole.addFloat("master_gain", 0.0f, 4.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getMasterGain() : 1.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setMasterGain(v);
        },
        "Global output multiplier (post-mix, applied to direct + indirect)");

    dbgConsole.addFloat("direct_gain", 0.0f, 4.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getDirectGain() : 1.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setDirectGain(v);
        },
        "Dry-bus multiplier — direct (line-of-sight) path only");

    dbgConsole.addFloat("reflection_gain", 0.0f, 4.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getReflectionGain() : 1.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setReflectionGain(v);
        },
        "Wet-bus multiplier — indirect (bounced) reverb path only");

    // ── Volumetric occlusion (direct-path attenuation) ──

    dbgConsole.addFloat("occlusion_radius", 0.3f, 30.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getOcclusionRadius() : 10.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setOcclusionRadius(v);
        },
        "Volumetric source sphere radius (engine ft) — larger = softer LOS occlusion");

    dbgConsole.addFloat("occlusion_samples", 4.0f, 64.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getOcclusionSamples()) : 16.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setOcclusionSamples(static_cast<int>(v));
        },
        "Ray samples for volumetric occlusion (more = smoother, more CPU)");

    // ── Door LPF / propagation (drives "around the corner" muffling) ──

    dbgConsole.addFloat("door_lpf_open_hz", 1000.0f, 24000.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getDoorLpfOpenHz() : 20000.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setDoorLpfOpenHz(v);
        },
        "LPF cutoff for sounds through open doors/portals (Hz, higher = brighter)");

    dbgConsole.addFloat("door_lpf_blocked_hz", 100.0f, 10000.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getDoorLpfBlockedHz() : 800.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setDoorLpfBlockedHz(v);
        },
        "LPF cutoff for sounds through closed doors (Hz, lower = more muffled)");

    dbgConsole.addFloat("prop_min_attenuation", 0.0f, 0.1f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getPropMinAttenuation() : 0.001f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setPropMinAttenuation(v);
        },
        "Floor on portal-routed path attenuation (higher = quieter through walls)");

    dbgConsole.addFloat("pathing_gain_scale", 0.1f, 10.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getPathingGainScale() : 1.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setPathingGainScale(v);
        },
        "Runtime multiplier on baked-pathing gain (1=identity, >1=louder through doorways)");

    dbgConsole.addFloat("pathing_update_interval", 0.0f, 1.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getPathingUpdateInterval() : 0.1f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setPathingUpdateInterval(v);
        },
        "Min seconds between Steam Audio pathing-sim updates (0=every frame, 0.1=10 Hz default)");

    // Bake probes: set to "on" to trigger re-baking.
    // The action runs on a background thread with a progress-bar overlay
    // (same helper as the first-run auto-bake) so the window stays
    // responsive instead of freezing for the duration of the bake.
    dbgConsole.addBool("bake_probes",
        []() { return false; },  // always shows "off" (it's an action, not a state)
        [misPath, &state, window](bool v) {
            if (!v) return;
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (!svc) return;
            std::string probePath = Darkness::AudioService::getProbeFilePath(misPath);
            std::fprintf(stderr, "Re-baking probes → %s\n", probePath.c_str());
            runBakeWithProgressBar(window, state, svc, probePath);
        },
        "Re-bake acoustic probes (~10-60s with progress bar). Auto-baked on first run.");

    // Re-run the reachability classification used by the probe overlay.
    // Auto-runs after every load/bake, but exposing a manual trigger lets
    // you tune the heuristic (e.g. after teleporting / spawning more
    // objects / poking RoomService) without restarting the binary.
    dbgConsole.addBool("classify_probes",
        []() { return false; },
        [](bool v) {
            if (!v) return;
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (!svc) return;
            size_t pruneCount = svc->classifyProbeReachability();
            std::fprintf(stderr,
                "Probe reachability reclassified: %zu would be pruned "
                "(see stderr above for breakdown).\n", pruneCount);
        },
        "Recompute the probe-reachability overlay (red=NoRoom, magenta=Unreachable).");

    dbgConsole.addFloat("probe_count", 0.0f, 99999.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getProbeCount()) : 0.0f;
        },
        [](float) { /* read-only */ },
        "Number of loaded acoustic probes (read-only)");

    // Bake-time grid parameters. Changes take effect on the next bake_probes
    // trigger — existing probes are not relocated. To test whether residual
    // footstep-reverb A/B variance is driven by probe sparsity, lower the
    // spacing (e.g. 2.5 ft) and re-bake. Disk + bake time scale with 1/spacing².
    dbgConsole.addFloat("probe_spacing_ft", 1.0f, 20.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getProbeSpacingFt() : 5.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setProbeSpacingFt(v);
        },
        "Probe grid spacing in feet (requires bake_probes to take effect)");

    dbgConsole.addFloat("probe_height_ft", 0.5f, 20.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getProbeHeightFt() : 5.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setProbeHeightFt(v);
        },
        "Probe height above floor in feet (requires bake_probes)");

    // ── Audio debug overlay ──
    // Scatter cubes at every baked probe (cyan; orange = nearest to listener)
    // and flash a yellow cube at the listener on each player-emitted voice
    // (footstep / land). Single toggle: the listener flash only makes sense
    // alongside the probe grid, since the whole point is correlating
    // listener-to-probe distance with perceived footstep loudness.
    dbgConsole.addBool("show_probes",
        [&state]() { return state.showProbes; },
        [&state](bool v) { state.showProbes = v; },
        "Probe grid (cyan; orange=nearest) + yellow listener flash on each footstep");

    dbgConsole.addBool("show_probe_radius",
        [&state]() { return state.showProbeRadius; },
        [&state](bool v) { state.showProbeRadius = v; },
        "Overlay each pathing probe's influence radius as a wireframe "
        "sphere. Companion to show_probes (cubes). Both overlays share "
        "the camera-nearest-rooms cull (debug_room_max_count) so cubes "
        "and spheres appear in sync; both also share the layer-2 "
        "occlusion coloring (red=ray-blocked, green=LOS, gray=out-of-"
        "range), so a pathing probe's cube and sphere always agree.");

    // Capability C — pathing-graph EQ-activity visualization.
    // Layer 1 (static adjacency, dim gray) + Layer 2 (per-edge EQ tint
    // in active-voice neighborhoods). Honesty note in the help text:
    // we are NOT querying Steam Audio's per-voice visited-probe set
    // (the public C API doesn't expose that); we replicate the
    // bake-time adjacency and aggregate neighborhood EQ activity.
    dbgConsole.addBool("show_pathing_graph",
        [&state]() { return state.showPathingGraph; },
        [&state](bool v) { state.showPathingGraph = v; },
        "Pathing-probe visibility graph + per-edge EQ-activity heatmap. "
        "Edges drawn from a Darkness-side replication of Steam Audio's "
        "bake-time visibility test (same visRange / numVisSamples). "
        "Edge tint = max(1 - eqCoeffs.mid) across active voices within "
        "visRadius of either endpoint (green=clear, red=blocked, "
        "yellow=partial; dim gray = no voice nearby). NOT a per-voice "
        "edge-visitation query — public Steam Audio API does not "
        "expose that. Companion: show_voice_arrows.");

    // Layer 3 — per-voice source→listener arrow colored by that
    // voice's mid-band EQ. Toggleable independently so a user can
    // disambiguate per-voice contribution from the neighborhood
    // heatmap.
    dbgConsole.addBool("show_voice_arrows",
        [&state]() { return state.showVoiceArrows; },
        [&state](bool v) { state.showVoiceArrows = v; },
        "Per-voice arrow from source to listener, colored by the "
        "voice's mid-band eqCoeffs (Layer 3 of show_pathing_graph). "
        "Useful when the graph heatmap is ambiguous about which "
        "voice is contributing to a hot edge.");

    dbgConsole.addFloat("probe_marker_size", 0.1f, 10.0f,
        [&state]() { return state.probeMarkerSize; },
        [&state](float v) { state.probeMarkerSize = std::max(0.1f, std::min(v, 10.0f)); },
        "Size in feet of probe/listener debug cubes");

    dbgConsole.addFloat("probe_render_radius", 0.0f, 10000.0f,
        [&state]() { return state.probeRenderRadius; },
        [&state](float v) { state.probeRenderRadius = std::max(0.0f, v); },
        "Listener-relative cull radius for probe overlay in feet (0 = draw all)");

    dbgConsole.addBool("record_audio",
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->isRecordingAudio() : false;
        },
        [](bool v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (!svc) return;
            if (v) svc->startAudioRecording();
            else   svc->stopAudioRecording();
        },
        "Record final audio output to WAV + position CSV for debugging");

    // ── Interaction ──

    dbgConsole.setGroup("Interaction");

    dbgConsole.addFloat("frob_distance", 1.0f, 30.0f,
        [&state]() {
            return state.frobSystem ? state.frobSystem->getFrobDistance()
                                    : Darkness::kDefaultFrobDistance;
        },
        [&state](float v) {
            if (state.frobSystem) state.frobSystem->setFrobDistance(v);
        },
        "Maximum interaction distance (original engine default: 8.0)");

    dbgConsole.addBool("toggle_platforms",
        []() { return false; },  // read as momentary (always shows "off")
        [&state](bool v) {
            if (v && state.movingTerrainSystem) {
                // Toggle all platforms: activate stopped ones, deactivate moving ones
                auto ids = state.movingTerrainSystem->getAllPlatformIDs();
                int activated = 0, deactivated = 0;
                for (int32_t id : ids) {
                    if (state.movingTerrainSystem->toggle(id)) {
                        const Darkness::Vector3 *vel =
                            state.movingTerrainSystem->getVelocity(id);
                        if (vel) ++activated; else ++deactivated;
                    }
                }
                std::fprintf(stderr, "toggle_platforms: %d activated, %d deactivated\n",
                             activated, deactivated);
            }
        },
        "Toggle all moving terrain (elevators/platforms) on/off");
}

// ── Event handling ──
// Process SDL events: quit, mouse look, scroll-wheel speed, keyboard shortcuts.
// Mutates camera, runtime flags, and model isolation state.
static void handleEvents(
    Darkness::RuntimeState &state,
    Darkness::DebugConsole &dbgConsole,
    SDL_Window *window)
{
    static constexpr float MOUSE_SENS = 0.002f;
    static constexpr float PI = 3.14159265f;

    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        // Debug console gets first crack at all events
        if (dbgConsole.handleEvent(ev)) continue;

        if (ev.type == SDL_QUIT) {
            state.running = false;
        } else if (ev.type == SDL_WINDOWEVENT) {
            // Track window focus to avoid rendering to an invalid Metal drawable
            // when the user Command+Tabs away (macOS) or Alt+Tabs (Windows/Linux).
            if (ev.window.event == SDL_WINDOWEVENT_FOCUS_LOST) {
                state.windowFocused = false;
            } else if (ev.window.event == SDL_WINDOWEVENT_FOCUS_GAINED) {
                state.windowFocused = true;
            }
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE) {
            state.running = false;
        } else if (ev.type == SDL_MOUSEMOTION) {
            state.cam.yaw   -= ev.motion.xrel * MOUSE_SENS;
            state.cam.pitch -= ev.motion.yrel * MOUSE_SENS;
            state.cam.pitch = std::max(-PI * 0.49f, std::min(PI * 0.49f, state.cam.pitch));
        } else if (ev.type == SDL_MOUSEWHEEL) {
            // Scroll wheel adjusts movement speed: 1.5x per tick
            if (ev.wheel.y > 0) {
                for (int i = 0; i < ev.wheel.y; ++i)
                    state.moveSpeed *= 1.5f;
            } else if (ev.wheel.y < 0) {
                for (int i = 0; i < -ev.wheel.y; ++i)
                    state.moveSpeed /= 1.5f;
            }
            state.moveSpeed = std::max(1.0f, std::min(500.0f, state.moveSpeed));
            updateTitleBar(window, state);
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_HOME) {
            // Teleport back to spawn point
            state.cam.pos[0] = state.spawnX;
            state.cam.pos[1] = state.spawnY;
            state.cam.pos[2] = state.spawnZ;
            state.cam.yaw = state.spawnYaw;
            state.cam.pitch = 0;
            // Also teleport physics player if physics mode is active
            if (state.physics) {
                Darkness::Vector3 spawnPos(state.spawnX, state.spawnY, state.spawnZ);
                state.physics->setPlayerPosition(spawnPos);
                state.physics->setPlayerYaw(state.spawnYaw);
            }
            std::fprintf(stderr, "Teleported to spawn (%.1f, %.1f, %.1f)\n",
                         state.spawnX, state.spawnY, state.spawnZ);
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_c
                   && !ev.key.repeat && state.physicsMode) {
            // C in physics mode: toggle crouch on/off (edge-triggered).
            // Without the !repeat filter, the OS sends KEYDOWN repeats every
            // ~30 ms while C is held, flipping the toggle 30×/sec — visible as
            // the player rapidly oscillating between Stand and Crouch.
            state.crouchToggled = !state.crouchToggled;
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_SPACE
                   && !ev.key.repeat && state.physicsMode && state.physics) {
            // Space in physics mode: jump (edge-triggered, not continuous).
            // The original Dark Engine fires jump once per key-press event.
            // SDL_KEYDOWN with key.repeat filtered out ensures we only jump
            // on the initial press, not on OS key-repeat while held.
            state.physics->playerJump();
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_p
                   && !ev.key.repeat) {
            // P: toggle simulation pause. Physics, doors, tweqs, and platforms
            // freeze while audio and rendering continue.
            Darkness::SimServicePtr simSvc = GET_SERVICE(Darkness::SimService);
            if (simSvc->isSimRunning()) {
                if (simSvc->isSimPaused()) {
                    simSvc->unPauseSim();
                    std::fprintf(stderr, "Simulation UNPAUSED\n");
                } else {
                    simSvc->pauseSim();
                    std::fprintf(stderr, "Simulation PAUSED\n");
                }
            }
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_f
                   && !ev.key.repeat) {
            // F: triple-purpose, ordered by priority:
            //   1. If holding something → throw with original engine's
            //      power=30 model (velocity = forward * 30 * coeff, where
            //      coeff = launcherMass / (objMass + launcherMass)).
            //      Heavy objects still move, just slower — matches Thief.
            //   2. Else if frob target has kFrobMove and is a dynamic body
            //      → grab it.
            //   3. Else → standard frob (doors, levers, scripts, etc.).
            // Drop is on R (uses power=0.05, basically zero — gravity wins).
            bool handled = false;
            if (state.grabSystem && state.grabSystem->isGrabbingAny()) {
                // Camera forward in Z-up Dark Engine convention.
                const float cosPitch = std::cos(state.cam.pitch);
                const Darkness::Vector3 fwd(
                    std::cos(state.cam.yaw) * cosPitch,
                    std::sin(state.cam.yaw) * cosPitch,
                    std::sin(state.cam.pitch));
                // Original engine's throw callback uses power=30.0 for player
                // throws (Dark Engine convention). Tunable via YAML in a follow-up.
                constexpr float kThrowPower = 30.0f;
                state.grabSystem->releaseAllWithPower(fwd, kThrowPower);
                handled = true;
            }
            if (!handled && state.grabSystem && state.frobSystem
                && state.frobSystem->hasTarget() && state.physics) {
                const auto &t = state.frobSystem->getTarget();
                const bool canMove = (t.frobActions & Darkness::kFrobMove) != 0;
                if (canMove && state.physics->hasDynamicBody(t.objID)) {
                    Darkness::GrabRequest req;
                    req.objID = t.objID;
                    req.gripPointWorld = t.hitPoint;
                    req.mode = Darkness::GrabMode::kCarry;
                    handled = state.grabSystem->grab(req, state.cam);
                }
            }
            if (!handled && state.frobSystem && state.frobSystem->hasTarget()) {
                state.frobSystem->executeFrob();
            }
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_r
                   && !ev.key.repeat) {
            // R: drop. Original engine routes drop through the same throw
            // path with power=0.05 — effectively zero forward velocity,
            // gravity dominates immediately.
            if (state.grabSystem && state.grabSystem->isGrabbingAny()) {
                const float cosPitch = std::cos(state.cam.pitch);
                const Darkness::Vector3 fwd(
                    std::cos(state.cam.yaw) * cosPitch,
                    std::sin(state.cam.yaw) * cosPitch,
                    std::sin(state.cam.pitch));
                constexpr float kDropPower = 0.05f;
                state.grabSystem->releaseAllWithPower(fwd, kDropPower);
            }
        }
    }
}

// ── Movement update ──
// WASD + vertical movement, suppressed while debug console is open.
// Two modes:
//   Fly mode (default): free camera with optional sphere collision (camera_collision)
//   Physics mode: gravity, ground detection, constraint collision (physics_mode)
static void updateMovement(
    float dt, Darkness::RuntimeState &state,
    const Darkness::MissionData &mission,
    const Darkness::DebugConsole &dbgConsole)
{
    // ── Auto-fly probe-tour ──
    // Runs unconditionally when enabled (even with the debug console open
    // — the whole point is unattended sweep iterations). Forces fly mode
    // because the physics integrator owns camera position otherwise.
    // Lazy activation: defer snapshotting probes until the first tick so
    // we read the live spawn camera position as the N-nearest "from" point.
    if (state.autoFly.enabled) {
        if (state.physicsMode) {
            state.physicsMode = false;
            std::fprintf(stderr,
                "[AUTO_FLY] forcing physics_mode off (auto-fly drives "
                "camera directly)\n");
        }
        if (!state.autoFly.active) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            const std::vector<Darkness::Vector3> empty;
            const auto &probes = svc
                ? svc->getPathingProbePositions() : empty;
            Darkness::Vector3 from(state.cam.pos[0], state.cam.pos[1],
                                    state.cam.pos[2]);
            state.autoFly.activate(from, probes);
            // activate() emits [FALLBACK] and clears `enabled` if no
            // probes are loaded — fall through to manual fly mode in
            // that case.
        }
        if (state.autoFly.active) {
            state.cam.roll = 0.0f;
            state.autoFly.tick(dt, state.cam.pos,
                                state.cam.yaw, state.cam.pitch);
            return;
        }
    } else if (state.autoFly.active) {
        // Toggled off via console — return camera to user control.
        state.autoFly.deactivate();
    }

    if (dbgConsole.isOpen()) return;

    const Uint8 *keys = SDL_GetKeyboardState(nullptr);

    // ── Physics mode: player walks with gravity and collision ──
    if (state.physicsMode && state.physics) {
        // Normalized movement input [-1, 1] for forward/strafe
        float forward = 0.0f, right = 0.0f;
        if (keys[SDL_SCANCODE_W]) forward += 1.0f;
        if (keys[SDL_SCANCODE_S]) forward -= 1.0f;
        if (keys[SDL_SCANCODE_D]) right   += 1.0f;
        if (keys[SDL_SCANCODE_A]) right   -= 1.0f;

        state.physics->setPlayerMovement(forward, right);
        state.physics->setPlayerYaw(state.cam.yaw);

        // Speed modes: run (Ctrl, 2x) and sneak (LShift, 0.5x).
        // Sneak takes priority — can't run and sneak simultaneously.
        state.physics->setPlayerRunning(
            keys[SDL_SCANCODE_LCTRL] || keys[SDL_SCANCODE_RCTRL]);
        state.physics->setPlayerSneaking(keys[SDL_SCANCODE_LSHIFT] != 0);

        // Jump: handled by SDL_KEYDOWN event (edge-triggered, not held).
        // The original Dark Engine fires jump on key-press events, not per-frame
        // polling. This prevents jump chaining when spacebar is held — the player
        // must release and re-press space to jump again after landing.

        // Crouch with C key (toggle on/off, handled in event loop)
        state.physics->setPlayerCrouching(state.crouchToggled);

        // Lean with Q/E — lateral camera offset, physics body stays in place.
        // In physics mode Q/E lean instead of moving vertically.
        int leanDir = 0;
        if (keys[SDL_SCANCODE_Q]) leanDir -= 1;
        if (keys[SDL_SCANCODE_E]) leanDir += 1;
        state.physics->getPlayerPhysics().setLeanDirection(leanDir);

        // Feed camera pitch to physics for diagnostic logging
        state.physics->getPlayerPhysics().setCameraPitch(state.cam.pitch);

        // Step the physics simulation (respects sim pause — when paused,
        // player movement input is accepted but not simulated, so the player
        // freezes in place). SimService applies flow coefficient for slow-mo.
        {
            Darkness::SimServicePtr simSvc = GET_SERVICE(Darkness::SimService);
            float simDt = dt;
            if (simSvc->isSimRunning()) {
                if (simSvc->isSimPaused())
                    simDt = 0.0f;
                else
                    simDt = dt * simSvc->getFlowCoeff();
            }
            state.physics->step(simDt);
        }

        // Read back eye position and lean tilt from physics into camera
        Darkness::Vector3 eye = state.physics->getPlayerEyePosition();

        state.cam.pos[0] = eye.x;
        state.cam.pos[1] = eye.y;
        state.cam.pos[2] = eye.z;
        state.cam.roll = state.physics->getPlayerPhysics().getLeanTilt();

        // Add view punch from object impacts (Source Engine spring-damper).
        // mPunchAngle is the absolute spring displacement from neutral, not a
        // per-frame delta — so we have to subtract last frame's value from
        // cam.pitch before adding this frame's, otherwise the offset would
        // integrate every render frame and pitch would run off to infinity
        // (a 1 rad punch + 120 fps = ~6800°/s drift). Roll has no equivalent
        // bookkeeping because cam.roll is overwritten each frame by the line
        // above before the punch is applied.
        Darkness::Vector3 punch = state.physics->getPlayerPhysics().getViewPunch();
        state.cam.pitch -= state.lastAppliedPunchPitch;
        state.cam.pitch += punch.x;
        state.lastAppliedPunchPitch = punch.x;
        state.cam.roll  += punch.z;

        // Per-render-frame head/viewport log — sampled after all camera mutations so
        // it captures the exact state that will render this frame. Counts how many
        // fixed physics steps occurred since the previous write so post-analysis can
        // detect render-rate-over-physics-rate aliasing.
        writeHeadLogRow(state);
        return;
    }

    // ── Fly mode: free camera with optional collision ──
    state.cam.roll = 0.0f;  // no lean tilt in fly mode
    float forward = 0, right = 0, up = 0;
    float speed = state.moveSpeed;
    if (keys[SDL_SCANCODE_LCTRL] || keys[SDL_SCANCODE_RCTRL])
        speed *= 3.0f;

    if (keys[SDL_SCANCODE_W]) forward += speed * dt;
    if (keys[SDL_SCANCODE_S]) forward -= speed * dt;
    if (keys[SDL_SCANCODE_D]) right   += speed * dt;
    if (keys[SDL_SCANCODE_A]) right   -= speed * dt;
    if (keys[SDL_SCANCODE_SPACE])  up += speed * dt;
    if (keys[SDL_SCANCODE_LSHIFT]) up -= speed * dt;
    if (keys[SDL_SCANCODE_Q]) up += speed * dt;
    if (keys[SDL_SCANCODE_E]) up -= speed * dt;

    // Save position before movement for collision revert
    float oldPos[3] = { state.cam.pos[0], state.cam.pos[1], state.cam.pos[2] };

    state.cam.move(forward, right, up);

    // Camera collision: constrain sphere within world cell planes
    if (state.cameraCollision) {
        applyCameraCollision(mission.wrData, oldPos, state.cam.pos);
    }
}

// ── Animated lightmap update ──
// Advance all light animation timers and re-blend changed lightmaps into
// the atlas CPU buffer, then upload the updated atlas to the GPU.
// debugTint: tint animated lightmap polygons magenta for visibility.
// forceFlicker: override all lights to flicker mode (saved/restored on toggle).
static void updateLightmaps(
    float dt, const Darkness::BuiltMeshes &meshes,
    Darkness::MissionData &mission, Darkness::GPUResources &gpu,
    Darkness::ObjectIlluminator &objectIlluminator,
    bool debugTint = false, bool forceFlicker = false,
    bool scriptLightDirty = false)
{
    // Track debug mode transitions so we can force a full re-blend
    static bool prevDebugTint = false;
    bool debugChanged = (debugTint != prevDebugTint);
    prevDebugTint = debugTint;

    // Handle force-flicker toggle: save/restore original light parameters
    static bool prevForceFlicker = false;
    static std::unordered_map<int16_t, Darkness::LightSource> savedLights;
    if (forceFlicker && !prevForceFlicker) {
        // Save original light state and override to flicker
        savedLights.clear();
        for (auto &[num, ls] : mission.lightSources) {
            savedLights[num] = ls;
            ls.mode = Darkness::ANIM_FLICKER;
            ls.inactive = false;
            ls.minBright = 0.0f;
            ls.maxBright = std::max(ls.maxBright, 1.0f);
            ls.brightenTime = 0.15f;
            ls.dimTime = 0.15f;
            ls.brightness = ls.maxBright;
            ls.countdown = 0.1f;
            ls.isRising = false;
            ls.prevIntensity = -1.0f;
        }
        debugChanged = true; // force full re-blend
        std::fprintf(stderr, "Force-flicker: ON (%zu lights)\n",
                     mission.lightSources.size());
    } else if (!forceFlicker && prevForceFlicker) {
        // Restore original light state
        for (auto &[num, ls] : mission.lightSources) {
            auto it = savedLights.find(num);
            if (it != savedLights.end()) ls = it->second;
            ls.prevIntensity = -1.0f;
        }
        savedLights.clear();
        debugChanged = true; // force full re-blend
        std::fprintf(stderr, "Force-flicker: OFF (restored original modes)\n");
    }
    prevForceFlicker = forceFlicker;

    bool anyLightChanged = false;
    std::unordered_map<int16_t, float> currentIntensities;

    for (auto &[lightNum, light] : mission.lightSources) {
        bool changed = Darkness::updateLightAnimation(light, dt);
        float intensity = (light.maxBright > 0.0f)
            ? light.brightness / light.maxBright : 0.0f;
        currentIntensities[lightNum] = intensity;
        if (changed) anyLightChanged = true;
    }

    // Check if LightScriptService changed any lights (script-driven on/off)
    if (scriptLightDirty) {
        anyLightChanged = true;
    }

    // Propagate animated intensities to per-object lighting. Each animated
    // light's static-light-table slot gets a multiplier of (current/max),
    // so when a torch animation drops to 0 (off via tweq, mid-flicker, dim
    // phase) any object whose cell.lightIndices references that slot dims
    // accordingly. Non-animated static lights stay at multiplier 1.0 from
    // the last reset (in setMissionData / objectIlluminator.clear()).
    objectIlluminator.resetLightMultipliers();
    for (const auto &[lightNum, idx] : mission.animLightToStaticIdx) {
        auto it = currentIntensities.find(lightNum);
        if (it == currentIntensities.end()) continue;
        objectIlluminator.setLightMultiplier(idx, it->second);
    }

    // Once-per-second verification tick: prove the multiplier chain is
    // actually animating. Per-vertex path (the default) has no other runtime
    // diagnostic, so without this you can't tell whether the chain is dead
    // or just running silently.
    //
    // The sampling deliberately PREFERS slots with animating modes (FLIP,
    // SMOOTH, RANDOM, BRIGHTEN, DIM, SEMI_RANDOM, FLICKER) — iteration order
    // surfaces static-mode lights first since MAX_BRIGHT is the editor
    // default, and they would falsely look like "nothing's moving" if shown.
    // Reports the global intensity min/max so even if all 3 sample slots
    // happen to be static, you can see whether any slot anywhere is moving.
    // Once at first tick, also dumps a mode-distribution histogram.
    {
        static int sTickFrame = 0;
        static int sFps = 60;
        static bool sHistogramDumped = false;
        // Estimate frame rate from dt to roughly hit one-per-second.
        if (dt > 1e-4f) sFps = std::max(1, std::min(240, (int)(1.0f / dt + 0.5f)));
        if (++sTickFrame >= sFps) {
            sTickFrame = 0;

            // First-tick mode histogram (subset of animLightToStaticIdx — the
            // actually-mapped lights, which is the set that matters for
            // object lighting). LightSource-level histogram from
            // parseAnimLightProperties() reports ALL parsed lights, but here
            // we want only those that survived position-matching.
            if (!sHistogramDumped) {
                sHistogramDumped = true;
                int counts[10] = {0};
                int inactiveMapped = 0;
                int badRange = 0;
                for (const auto &[lightNum, idx] : mission.animLightToStaticIdx) {
                    auto lsIt = mission.lightSources.find(lightNum);
                    if (lsIt == mission.lightSources.end()) continue;
                    int m = (int)lsIt->second.mode;
                    if (m >= 0 && m < 10) ++counts[m];
                    if (lsIt->second.inactive) ++inactiveMapped;
                    if (lsIt->second.maxBright - lsIt->second.minBright <= 0.0f)
                        ++badRange;
                }
                static const char *kModeNames[10] = {
                    "FLIP","SMOOTH","RANDOM","MINBRIGHT","MAXBRIGHT",
                    "ZERO","BRIGHTEN","DIM","SEMI_RANDOM","FLICKER"
                };
                std::fprintf(stderr,
                    "[OBJ-LIGHT-HIST] mapped=%zu inactive=%d badRange=%d",
                    mission.animLightToStaticIdx.size(),
                    inactiveMapped, badRange);
                for (int i = 0; i < 10; ++i)
                    if (counts[i] > 0)
                        std::fprintf(stderr, " %s=%d", kModeNames[i], counts[i]);
                std::fprintf(stderr, "\n");
            }

            int dimmed = objectIlluminator.dimmedLightCount();

            // Scan all mapped lights for intensity range (global min/max).
            float globalMin = 2.0f, globalMax = -1.0f;
            int activelyAnimating = 0;
            for (const auto &[lightNum, idx] : mission.animLightToStaticIdx) {
                auto cit = currentIntensities.find(lightNum);
                if (cit == currentIntensities.end()) continue;
                float v = cit->second;
                if (v < globalMin) globalMin = v;
                if (v > globalMax) globalMax = v;
                auto lsIt = mission.lightSources.find(lightNum);
                if (lsIt != mission.lightSources.end()) {
                    int m = (int)lsIt->second.mode;
                    bool isAnim = (m == ANIM_FLIP || m == ANIM_SMOOTH ||
                                   m == ANIM_RANDOM || m == ANIM_BRIGHTEN ||
                                   m == ANIM_DIM || m == ANIM_SEMI_RANDOM ||
                                   m == ANIM_FLICKER);
                    if (isAnim && !lsIt->second.inactive) ++activelyAnimating;
                }
            }
            if (globalMax < 0.0f) { globalMin = 0.0f; globalMax = 0.0f; }

            std::fprintf(stderr,
                "[OBJ-LIGHT-TICK] mapped=%zu animating=%d dimmed=%d "
                "intensity=[%.2f,%.2f]",
                mission.animLightToStaticIdx.size(),
                activelyAnimating, dimmed, globalMin, globalMax);

            // Prefer to sample slots whose source is actively animating.
            // Two passes: first only animated modes; fall back to any.
            auto pickSample = [&](bool animatedOnly, int &shown, int maxShow) {
                for (const auto &[lightNum, idx] : mission.animLightToStaticIdx) {
                    if (shown >= maxShow) return;
                    auto cit = currentIntensities.find(lightNum);
                    if (cit == currentIntensities.end()) continue;
                    auto lsIt = mission.lightSources.find(lightNum);
                    if (lsIt == mission.lightSources.end()) continue;
                    int m = (int)lsIt->second.mode;
                    bool isAnim = (m == ANIM_FLIP || m == ANIM_SMOOTH ||
                                   m == ANIM_RANDOM || m == ANIM_BRIGHTEN ||
                                   m == ANIM_DIM || m == ANIM_SEMI_RANDOM ||
                                   m == ANIM_FLICKER);
                    if (animatedOnly && (!isAnim || lsIt->second.inactive))
                        continue;
                    static const char *kModeNames[10] = {
                        "FLIP","SMOOTH","RANDOM","MINBRIGHT","MAXBRIGHT",
                        "ZERO","BRIGHTEN","DIM","SEMI_RANDOM","FLICKER"
                    };
                    const char *modeStr = (m >= 0 && m < 10) ? kModeNames[m] : "?";
                    std::fprintf(stderr,
                        " | ln=%d idx=%d mult=%.2f b=%.2f/%.2f mode=%s%s",
                        lightNum, idx, cit->second,
                        lsIt->second.brightness, lsIt->second.maxBright,
                        modeStr, lsIt->second.inactive ? "/inactive" : "");
                    ++shown;
                }
            };
            int shown = 0;
            pickSample(true, shown, 3);
            if (shown == 0)
                pickSample(false, shown, 3);
            std::fprintf(stderr, "\n");
        }
    }

    // Atlas blend only runs when lightmapped rendering is active and the
    // GPU atlas is ready. Per-object lighting (above) is independent and
    // must update every frame regardless — gating it on the atlas would
    // freeze object illumination at the moment the user e.g. flips a
    // light switch in a non-lightmapped configuration.
    if (!meshes.lightmappedMode || gpu.lmAtlasSet.atlases.empty()) return;

    // Force full re-blend when debug mode toggles
    bool forceAll = debugChanged;

    // Re-blend changed lightmaps into atlas CPU buffer, tracking dirty region
    if (anyLightChanged || forceAll) {
        const auto &atlas = gpu.lmAtlasSet.atlases[0];
        int dirtyX0 = atlas.size, dirtyY0 = atlas.size;
        int dirtyX1 = 0, dirtyY1 = 0;

        if (forceAll) {
            // Re-blend ALL animated polygons (debug toggle or first frame)
            std::unordered_set<uint64_t> visited;
            for (const auto &[lightNum, polys] : mission.animLightIndex) {
                for (const auto &[ci, pi] : polys) {
                    uint64_t key = (static_cast<uint64_t>(ci) << 32)
                                 | static_cast<uint32_t>(pi);
                    if (!visited.insert(key).second) continue;

                    const auto &entry = gpu.lmAtlasSet.entries[ci][pi];
                    Darkness::blendAnimatedLightmap(
                        gpu.lmAtlasSet.atlases[0], mission.wrData, ci, pi,
                        entry, currentIntensities, debugTint);

                    int x0 = std::max(0, entry.pixelX - 2);
                    int y0 = std::max(0, entry.pixelY - 2);
                    int x1 = std::min(atlas.size, entry.pixelX + entry.pixelW + 2);
                    int y1 = std::min(atlas.size, entry.pixelY + entry.pixelH + 2);
                    dirtyX0 = std::min(dirtyX0, x0);
                    dirtyY0 = std::min(dirtyY0, y0);
                    dirtyX1 = std::max(dirtyX1, x1);
                    dirtyY1 = std::max(dirtyY1, y1);
                }
            }
            // Reset prevIntensity so subsequent frames don't skip updates
            for (auto &[ln, light] : mission.lightSources)
                light.prevIntensity = -1.0f;
        } else {
            for (auto &[lightNum, light] : mission.lightSources) {
                float intensity = currentIntensities[lightNum];
                if (std::abs(intensity - light.prevIntensity) < 0.002f) continue;
                light.prevIntensity = intensity;

                auto it = mission.animLightIndex.find(lightNum);
                if (it == mission.animLightIndex.end()) continue;

                for (auto &[ci, pi] : it->second) {
                    const auto &entry = gpu.lmAtlasSet.entries[ci][pi];
                    Darkness::blendAnimatedLightmap(
                        gpu.lmAtlasSet.atlases[0], mission.wrData, ci, pi,
                        entry, currentIntensities, debugTint);

                    int x0 = std::max(0, entry.pixelX - 2);
                    int y0 = std::max(0, entry.pixelY - 2);
                    int x1 = std::min(atlas.size, entry.pixelX + entry.pixelW + 2);
                    int y1 = std::min(atlas.size, entry.pixelY + entry.pixelH + 2);
                    dirtyX0 = std::min(dirtyX0, x0);
                    dirtyY0 = std::min(dirtyY0, y0);
                    dirtyX1 = std::max(dirtyX1, x1);
                    dirtyY1 = std::max(dirtyY1, y1);
                }
            }
        }

        // Upload only the dirty sub-rectangle to GPU
        if (dirtyX1 > dirtyX0 && dirtyY1 > dirtyY0) {
            int dw = dirtyX1 - dirtyX0;
            int dh = dirtyY1 - dirtyY0;
            std::vector<uint8_t> sub(dw * dh * 4);
            for (int y = 0; y < dh; ++y) {
                std::memcpy(&sub[y * dw * 4],
                            &atlas.rgba[((dirtyY0 + y) * atlas.size + dirtyX0) * 4],
                            dw * 4);
            }
            const bgfx::Memory *mem = bgfx::copy(sub.data(), static_cast<uint32_t>(sub.size()));
            bgfx::updateTexture2D(gpu.lightmapAtlasHandles[0], 0, 0,
                static_cast<uint16_t>(dirtyX0), static_cast<uint16_t>(dirtyY0),
                static_cast<uint16_t>(dw), static_cast<uint16_t>(dh), mem);
        }
    }
}

// ── Frame preparation ──
// Compute per-frame matrices, fog uniforms, sampler flags, underwater state,
// portal culling, and bgfx view transforms. Returns a FrameContext consumed
// by each render pass within a single frame.
static Darkness::FrameContext prepareFrame(
    Darkness::RuntimeState &state,
    const Darkness::MissionData &mission)
{
    using namespace Darkness;
    FrameContext fc{};

    // Projection matrix (shared by sky and world views)
    bx::mtxProj(fc.proj, 60.0f,
                 float(WINDOW_WIDTH) / float(WINDOW_HEIGHT),
                 0.1f, 5000.0f,
                 bgfx::getCaps()->homogeneousDepth,
                 bx::Handedness::Right);

    // Texture sampler flags for the current filtering mode (shared by both views).
    // UINT32_MAX = use texture's baked POINT flags (default).
    // Other modes override with explicit sampler flags per draw call.
    // Two variants: MIRROR wrap (world/object textures), CLAMP wrap (skybox).
    switch (state.filterMode) {
    case 0: // Point: use texture's baked POINT flags
        fc.texSampler = UINT32_MAX;
        fc.skySampler = UINT32_MAX;
        break;
    case 1: // Bilinear: linear min/mag, point mip
        fc.texSampler = BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR
                      | BGFX_SAMPLER_MIP_POINT;
        fc.skySampler = BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP
                      | BGFX_SAMPLER_MIP_POINT;
        break;
    case 2: // Trilinear: linear min/mag/mip
        fc.texSampler = BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR;
        fc.skySampler = BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP;
        break;
    case 3: // Anisotropic: aniso min/mag, linear mip
        fc.texSampler = BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR
                      | BGFX_SAMPLER_MIN_ANISOTROPIC | BGFX_SAMPLER_MAG_ANISOTROPIC;
        fc.skySampler = BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP
                      | BGFX_SAMPLER_MIN_ANISOTROPIC | BGFX_SAMPLER_MAG_ANISOTROPIC;
        break;
    default:
        fc.texSampler = UINT32_MAX;
        fc.skySampler = UINT32_MAX;
        break;
    }

    // ── Underwater detection ──
    // Check if the camera is inside a water cell (mediaType==2).
    // When submerged, override fog with short-range blue-green water fog.
    fc.underwater = (getCameraMediaType(mission.wrData, state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]) == 2);

    // Water fog defaults: dark blue-green tint, short visibility range
    static constexpr float waterFogR = 0.10f, waterFogG = 0.18f, waterFogB = 0.25f;
    static constexpr float waterFogDist = 80.0f;

    // ── Fog uniform values (reused before every bgfx::submit) ──
    // bgfx uniforms are per-submit: cleared after each draw call.
    // We set them before every submit so all shaders receive fog data.
    // When underwater, override with water fog regardless of FOG chunk.
    if (fc.underwater) {
        fc.fogColorArr[0] = waterFogR; fc.fogColorArr[1] = waterFogG;
        fc.fogColorArr[2] = waterFogB; fc.fogColorArr[3] = 1.0f;
        fc.fogOnArr[0] = 1.0f; fc.fogOnArr[1] = waterFogDist;
        fc.fogOnArr[2] = 0.0f; fc.fogOnArr[3] = 0.0f;
    } else {
        fc.fogColorArr[0] = mission.fogParams.r; fc.fogColorArr[1] = mission.fogParams.g;
        fc.fogColorArr[2] = mission.fogParams.b; fc.fogColorArr[3] = 1.0f;
        fc.fogOnArr[0] = mission.fogParams.enabled ? 1.0f : 0.0f;
        fc.fogOnArr[1] = mission.fogParams.distance;
        fc.fogOnArr[2] = 0.0f; fc.fogOnArr[3] = 0.0f;
    }
    fc.fogOffArr[0] = 0.0f; fc.fogOffArr[1] = 1.0f;
    fc.fogOffArr[2] = 0.0f; fc.fogOffArr[3] = 0.0f;

    // Sky fog: underwater always fogs sky; otherwise respect SKYOBJVAR.fog
    fc.skyFogged = fc.underwater || (mission.fogParams.enabled && mission.skyParams.fog);
    fc.skyFogArr = fc.skyFogged ? fc.fogOnArr : fc.fogOffArr;

    // Update clear color per-frame for underwater tinting
    if (fc.underwater) {
        uint8_t cr = static_cast<uint8_t>(waterFogR * 255.0f);
        uint8_t cg = static_cast<uint8_t>(waterFogG * 255.0f);
        uint8_t cb = static_cast<uint8_t>(waterFogB * 255.0f);
        uint32_t waterClear = (uint32_t(cr) << 24) | (uint32_t(cg) << 16)
                            | (uint32_t(cb) << 8) | 0xFF;
        bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                            waterClear, 1.0f, 0);
    } else {
        bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                            state.skyClearColor, 1.0f, 0);
    }

    // View 1 transform (world + objects)
    state.cam.getViewMatrix(fc.view);
    bgfx::setViewTransform(1, fc.view, fc.proj);

    // Ensure view 1's depth clear always executes, even if portal culling
    // produces zero visible cells (e.g., camera between cells). Without this,
    // bgfx skips the clear for views with no submissions, leaving stale depth
    // values that cause far geometry to flash through near geometry.
    bgfx::touch(1);

    // ── Portal culling: determine visible cells ──
    // Build view-projection matrix and extract frustum planes for portal tests.
    // When culling is disabled, visibleCells remains empty (skip filtering).
    // Guard band (world units) expands the frustum slightly to prevent large
    // objects from popping at screen edges due to cell-based culling.
    if (state.portalCulling) {
        // Build a wider projection for culling than for rendering.
        // 120-degree vertical FOV (~160° horizontal at 16:9) gives generous
        // margin so large objects aren't culled while still partially visible.
        constexpr float CULL_FOV = 120.0f;
        float cullProj[16];
        bx::mtxProj(cullProj, CULL_FOV,
                     float(WINDOW_WIDTH) / float(WINDOW_HEIGHT),
                     0.1f, 5000.0f,
                     bgfx::getCaps()->homogeneousDepth,
                     bx::Handedness::Right);
        float vp[16];
        bx::mtxMul(vp, fc.view, cullProj);
        ViewFrustum frustum;
        frustum.extractFromVP(vp);

        int32_t camCell = findCameraCell(mission.wrData, state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
        // Build door-blocking callback: when a portal connects cells in different
        // rooms with a closed vision-blocking door, skip the portal traversal.
        PortalBlockedCallback doorBlocking = nullptr;
        if (!mission.cellToRoom.empty() && state.doorSystem) {
            doorBlocking = [&](uint32_t srcCell, uint32_t tgtCell) -> bool {
                if (srcCell >= mission.cellToRoom.size() ||
                    tgtCell >= mission.cellToRoom.size())
                    return false;
                int32_t srcRoom = mission.cellToRoom[srcCell];
                int32_t tgtRoom = mission.cellToRoom[tgtCell];
                if (srcRoom < 0 || tgtRoom < 0 || srcRoom == tgtRoom)
                    return false;
                // Check if a door between these rooms is closed and vision-blocking
                float openFrac = state.doorSystem->getOpenFractionForRooms(srcRoom, tgtRoom);
                return openFrac < 0.01f;  // fully closed = block
            };
        }

        fc.visibleCells = portalBFS(mission.wrData, mission.cellPortals, camCell, frustum,
                                     state.cam.pos[0], state.cam.pos[1], state.cam.pos[2],
                                     doorBlocking);
        state.cullVisibleCells = static_cast<uint32_t>(fc.visibleCells.size());

        // Build a tighter frustum from the actual rendering projection for
        // per-object AABB tests. Objects whose cells are culled by the wide
        // portal frustum still get rendered if their world-space AABB
        // intersects this rendering frustum.
        float renderVP[16];
        bx::mtxMul(renderVP, fc.view, fc.proj);
        fc.objFrustum.extractFromVP(renderVP);
    }

    // Opaque geometry render state (constant each frame)
    fc.renderState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                   | BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS
                   | BGFX_STATE_CULL_CW;

    return fc;
}





int main(int argc, char *argv[]) {
    if (argc < 2) {
        printHelp();
        return 1;
    }

    // Parse config: hardcoded defaults → YAML file → CLI overrides
    Darkness::RenderConfig cfg;

    // First CLI pass: extract --config path (and detect --help early)
    Darkness::CliResult cli = Darkness::applyCliOverrides(argc, argv, cfg);

    if (cli.helpRequested) {
        printHelp();
        return 0;
    }

    if (!cli.misPath) {
        std::fprintf(stderr, "Error: no mission file specified.\n\n");
        printHelp();
        return 1;
    }

    // Route all engine diagnostics to stdout so standard pipes and redirects work:
    //   ./darknessRender ... | grep FRICTION
    //   ./darknessRender ... > log.txt
    // The render binary is interactive (SDL window), so stdout carries no structured
    // data. Merging stderr into stdout makes all fprintf(stderr, ...) output
    // easily filterable. Line-buffering ensures output appears immediately.
    setvbuf(stdout, nullptr, _IOLBF, 0);
    dup2(fileno(stdout), fileno(stderr));

    // Load YAML config (defaults to ./darknessRender.yaml if no --config flag).
    //
    // The library returns false in two cases: (1) file doesn't exist —
    // fine, run with defaults; (2) file exists but YAML parse / type
    // conversion failed — abort, because everything past the bad line was
    // silently skipped, leaving us running with a half-applied config.
    // (1) and (2) are distinguished by whether the file is on disk.
    std::string configPath = cli.configPath.empty() ? "darknessRender.yaml" : cli.configPath;
    bool ok = Darkness::loadConfigFromYAML(configPath, cfg);
    if (!ok) {
        struct stat st{};
        bool fileExists = (stat(configPath.c_str(), &st) == 0);
        if (fileExists) {
            std::fprintf(stderr,
                "\nAborting: config file present but unreadable. "
                "Fix the YAML error above (or pass a different --config path) "
                "and re-run.\n");
            return 1;
        }
    }

    // Re-apply CLI so flags always win over YAML values
    cli = Darkness::applyCliOverrides(argc, argv, cfg);

    // All CPU-side parsed mission content and mutable runtime state.
    Darkness::MissionData mission;
    Darkness::RuntimeState state;

    // Unpack mutable config into state structs.
    // Resource paths: CLI flag overrides paths.* in YAML if both are present.
    // This lets the YAML carry a sensible default (e.g. mounted disc path)
    // while still allowing one-off overrides on the command line.
    const char *misPath    = cli.misPath;
    std::string resPath    = !cli.resPath.empty() ? cli.resPath : cfg.resPath;
    state.showObjects       = cfg.showObjects;
    state.showFallbackCubes = cfg.showFallbackCubes;
    state.portalCulling     = cfg.portalCulling;
    state.cameraCollision   = cfg.cameraCollision;
    state.filterMode        = cfg.filterMode;
    state.lightmapFiltering = cfg.lightmapFiltering;
    state.waveAmplitude    = cfg.waveAmplitude;
    state.uvDistortion     = cfg.uvDistortion;
    state.waterRotation    = cfg.waterRotation;
    state.waterScrollSpeed = cfg.waterScrollSpeed;

    // ── Mandatory game-resource paths ──
    // The renderer needs both snd.crf (sounds) and the schema directory
    // (.sch files mapping schema names to actual sample files). Without
    // either, audio fails silently in confusing ways (every ambient retries
    // load forever in the background). Treat both as required.
    if (resPath.empty()) {
        std::fprintf(stderr,
            "ERROR: a Thief 2 RES directory is required.\n"
            "       Set it via either:\n"
            "         --res <path>           (CLI override)\n"
            "         paths.res: <path>      (YAML; persists across runs)\n"
            "       The path must contain fam.crf, obj.crf, snd.crf — typically\n"
            "       the installed game's RES folder, e.g.\n"
            "       /Volumes/THIEF2_INSTALL_C/THIEF2/RES.\n");
        return 1;
    }
    {
        struct stat st;
        if (::stat(resPath.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
            std::fprintf(stderr,
                "ERROR: --res path '%s' does not exist or is not a directory.\n",
                resPath.c_str());
            return 1;
        }
    }
    mission.texturedMode = true;

    // ── Initialize logging (required before ServiceManager) ──
    // Register a stderr listener and run at INFO level so audio + service
    // init messages (miniaudio init, Steam Audio init, snd.crf load,
    // probe baking) are visible. Without this, LOG_INFO / LOG_ERROR have
    // no destination and audio-init failures are invisible.
    Darkness::Logger logger;
    Darkness::StdLog stdlog;
    logger.registerLogListener(&stdlog);
    logger.setLogLevel(Darkness::Logger::LOG_LEVEL_INFO);
    Darkness::ConsoleBackend console;

    // ── Initialize service stack, load database, construct IWorldQuery ──
    std::string scriptsDir = "scripts/thief2";
    auto worldQuery = initServiceStack(misPath, scriptsDir, cfg);

    // ── Load mission data: WR geometry, portals, spawn, lights, sky, fog, flow ──
    if (!loadMissionData(misPath, mission))
        return 1;

    // Build cell→room mapping for door visibility blocking.
    // Each cell center is tested against room bounding planes to find which
    // room it belongs to. Used by portalBFS to skip portals blocked by doors.
    {
        auto roomSvc = GET_SERVICE(Darkness::RoomService);
        if (roomSvc && roomSvc->isLoaded()) {
            mission.cellToRoom = buildCellToRoomMap(
                mission.wrData, roomSvc->getAllRooms());

            // BSP-aware line-of-sight callback. The room-graph BFS uses
            // this at chain reconstruction time to validate each bend
            // segment is unobstructed by world geometry; blocked
            // segments trigger a per-portal bend-refinement search and,
            // if no clear bend can be found, the path is rejected
            // outright. The renderer owns the BSP data, so the
            // callback lives here.
            auto losClear = [&mission](const Darkness::Vector3 &a,
                                       const Darkness::Vector3 &b) -> bool {
                Darkness::RayHit hit;
                return !Darkness::raycastWorld(mission.wrData, a, b, hit);
            };

            auto audioSvcForLos = GET_SERVICE(Darkness::AudioService);
            if (audioSvcForLos) {
                audioSvcForLos->setSoundPathLineOfSightFn(losClear);
            }
        }
    }

    // Create physics world from parsed WR cell geometry.
    // Must be created after loadMissionData() so mission.wrData is valid.
    // The physics instance lives for the duration of the program; wrData
    // must outlive it (both are in main's scope).
    // Map config physicsRate to PhysicsTimestep preset:
    //   <= 12 → VINTAGE (12.5Hz, 3 collision iters, authentic Dark Engine feel)
    //   >= 120 → ULTRA (120Hz, 1 iter, high-fidelity)
    //   else → MODERN (60Hz, 1 iter, default)
    auto physTimestep = (cfg.physicsRate <= 12)  ? Darkness::VINTAGE
                      : (cfg.physicsRate >= 120) ? Darkness::ULTRA
                      : Darkness::MODERN;
    state.physics = std::make_unique<Darkness::DarkPhysics>(mission.wrData, physTimestep);

    // ── Query player physics properties from archetype inheritance chain ──
    // The starting point object inherits P$PhysAttr/P$PhysDims from the
    // Avatar archetype in dark.gam. Values override PlayerPhysics defaults.
    if (mission.spawnInfo.found && mission.spawnInfo.objectID != 0) {
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        Darkness::PlayerPhysicsConfig cfg;

        // Read P$PhysAttr — may be 48 bytes (without poreSize) or 52 bytes
        Darkness::PropPhysAttr attr = {};
        size_t rawSize = 0;
        const uint8_t *rawData = Darkness::getPropertyRawData(
            propSvc.get(), "PhysAttr", mission.spawnInfo.objectID, rawSize);
        if (rawData && rawSize >= 20) {
            // Copy what's available (handles both 48 and 52-byte variants)
            std::memcpy(&attr, rawData, std::min(rawSize, sizeof(attr)));
            cfg.gravityScale = attr.gravity / 100.0f;
            cfg.mass = attr.mass;
            cfg.density = attr.density;
            cfg.elasticity = attr.elasticity;
            std::fprintf(stderr, "P$PhysAttr (obj %d): gravity=%.1f%% mass=%.1f "
                         "density=%.1f elasticity=%.2f friction=%.3f\n",
                         mission.spawnInfo.objectID, attr.gravity, attr.mass,
                         attr.density, attr.elasticity, attr.friction);
        } else {
            std::fprintf(stderr, "P$PhysAttr: not found for obj %d (using defaults)\n",
                         mission.spawnInfo.objectID);
        }

        // Read P$PhysDims (52 bytes) — sphere radii and submodel offsets
        Darkness::PropPhysDims dims = {};
        if (Darkness::getTypedProperty<Darkness::PropPhysDims>(
                propSvc.get(), "PhysDims", mission.spawnInfo.objectID, dims)) {
            if (dims.radius[0] > 0.0f) cfg.headRadius = dims.radius[0];
            if (dims.radius[1] > 0.0f) cfg.bodyRadius = dims.radius[1];
            cfg.headOffsetZ = dims.offset1Z;
            cfg.bodyOffsetZ = dims.offset2Z;
            std::fprintf(stderr, "P$PhysDims (obj %d): r0=%.2f r1=%.2f "
                         "off1=(%.2f,%.2f,%.2f) off2=(%.2f,%.2f,%.2f)\n",
                         mission.spawnInfo.objectID,
                         dims.radius[0], dims.radius[1],
                         dims.offset1X, dims.offset1Y, dims.offset1Z,
                         dims.offset2X, dims.offset2Y, dims.offset2Z);
        } else {
            std::fprintf(stderr, "P$PhysDims: not found for obj %d (using defaults)\n",
                         mission.spawnInfo.objectID);
        }

        state.physics->applyPlayerConfig(cfg);
    }

    // Enable audio logging if --audio-log was passed (off by default to reduce noise)
    if (cfg.audioLog) {
        Darkness::gAudioLogVerbose = true;
        std::fprintf(stderr, "Audio logging enabled (--audio-log)\n");
    }

    // Enable stair step diagnostics if developer.step_log is true
    if (cfg.stepLog && state.physics) {
        state.physics->getPlayerPhysics().setStepLog(true);
        std::fprintf(stderr, "Stair step logging enabled (developer.step_log)\n");
    }

    // Per-render-frame head log can be opened at runtime via the debug
    // console (`head_log` toggle, writes to ./head_log.csv).

    // ── Load motion capture data from motions.crf ──
    // motions.crf is present in ALL Dark Engine games (Thief 1/2, System Shock 2).
    // Currently loaded for future NPC animation (Phase 4 AI). The player camera uses
    // pre-scripted offset poses (matching the original engine's PlayerMotionTable),
    // NOT raw motion capture data — mocap is only for third-person creature animation.
    std::unique_ptr<Darkness::MotionService> motionSvc;
    motionSvc = std::make_unique<Darkness::MotionService>();
    if (motionSvc->loadFromCRF(resPath)) {
        std::fprintf(stderr, "Motion: loaded %d clips from motions.crf (for future NPC use)\n",
                     motionSvc->clipCount());
    } else {
        std::fprintf(stderr, "WARNING: failed to load motions.crf from %s\n"
                             "         NPC animation will not be available.\n",
                     resPath.c_str());
    }

    // ── Load sound resources from snd.crf + schema directory ──
    // Both snd.crf (in resPath) and the schema directory are required —
    // without schemas, ambient/event playback falls back to looking up
    // sample files by schema name, which never resolves and silently
    // floods the log with load failures. Hard-fail at startup instead.
    {
        Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);
        // CLI --schemas wins; otherwise paths.schemas from YAML; otherwise let
        // AudioService search the default locations next to RES.
        std::string schemasPath = !cli.schemasPath.empty() ? cli.schemasPath : cfg.schemasPath;
        if (!audioSvc->loadSoundResources(resPath, schemasPath)) {
            std::fprintf(stderr,
                "ERROR: audio resource initialization failed.\n"
                "       The RES path must contain snd.crf, and the schema directory\n"
                "       must be locatable. Set the schema directory via either:\n"
                "         --schemas <path>          (CLI override)\n"
                "         paths.schemas: <path>     (YAML; persists across runs)\n"
                "       e.g. /Volumes/THIEF2_CD2/EDITOR/SCHEMA. Alternatively,\n"
                "       place the schemas next to RES at <res>/../EDITOR/SCHEMA.\n");
            return 1;
        }
    }

    // Parse TXLIST early — needed by the acoustic mesh builder for material
    // keyword matching. The full texture loading (CRF I/O, GPU upload) happens
    // later in loadWorldTextures(), but we need the texture name strings now.
    if (mission.texturedMode && mission.txList.textures.empty()) {
        try {
            mission.txList = Darkness::parseTXList(misPath);
            std::fprintf(stderr, "TXLIST (early parse for acoustics): %zu textures\n",
                         mission.txList.textures.size());
        } catch (const std::exception &e) {
            std::fprintf(stderr, "Early TXLIST parse failed: %s\n", e.what());
        }
    }

    // ── Build Steam Audio acoustic scene from WR world geometry ──
    // Build once at load time, no per-frame rebuilds needed.
    {
        Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);

        const auto &wr = mission.wrData;
        Darkness::AcousticSceneData fullScene;

        // Vertex deduplication: hash map from quantized position to vertex index.
        // Adjacent cells share vertices at polygon edges — without dedup, each
        // polygon pushes its own copy, inflating the vertex buffer ~3x.
        // Quantize to 0.01 units to merge coincident vertices robustly.
        struct VertKey {
            int32_t x, y, z;
            bool operator==(const VertKey &o) const {
                return x == o.x && y == o.y && z == o.z;
            }
        };
        struct VertKeyHash {
            size_t operator()(const VertKey &k) const {
                // FNV-1a inspired hash for 3 ints
                size_t h = 0x811c9dc5u;
                h ^= static_cast<size_t>(k.x); h *= 0x01000193u;
                h ^= static_cast<size_t>(k.y); h *= 0x01000193u;
                h ^= static_cast<size_t>(k.z); h *= 0x01000193u;
                return h;
            }
        };
        std::unordered_map<VertKey, uint32_t, VertKeyHash> vertexMap;
        vertexMap.reserve(wr.numCells * 20);  // rough estimate

        // Helper: get or insert a deduplicated vertex, returns index
        auto getVertexIndex = [&](float x, float y, float z) -> uint32_t {
            // Quantize to 0.01 units for robust merging
            VertKey key{static_cast<int32_t>(std::round(x * 100.0f)),
                        static_cast<int32_t>(std::round(y * 100.0f)),
                        static_cast<int32_t>(std::round(z * 100.0f))};
            auto it = vertexMap.find(key);
            if (it != vertexMap.end())
                return it->second;
            uint32_t idx = static_cast<uint32_t>(fullScene.vertices.size() / 3);
            fullScene.vertices.push_back(x);
            fullScene.vertices.push_back(y);
            fullScene.vertices.push_back(z);
            vertexMap[key] = idx;
            return idx;
        };

        for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
            const auto &cell = wr.cells[ci];
            int numSolid = cell.numPolygons - cell.numPortals;

            for (int pi = 0; pi < cell.numPolygons; ++pi) {
                const auto &poly = cell.polygons[pi];

                bool isPortal = (pi >= numSolid);

                // The WR polygon ordering within each cell is:
                //   [0, numSolid)        — solid walls (textured, physical)
                //   [numSolid, numTex)   — rendered portals (water/glass boundaries)
                //   [numTex, numPolys)   — non-rendered portals (BSP splits)
                //
                // Non-rendered portals are BSP subdivision artifacts — invisible
                // boundaries created when the BSP compiler splits large cells.
                // In outdoor areas, these create a lattice of phantom surfaces
                // floating in mid-air. They must be EXCLUDED from the acoustic
                // mesh to prevent phantom reflections.
                bool isNonRenderedPortal = isPortal && (pi >= cell.numTextured);
                if (isNonRenderedPortal)
                    continue;

                // Skip sky polygons (BACKHACK_IDX = 249) — only solid polys have textures
                if (!isPortal && pi < cell.numTextured && cell.texturing[pi].txt == 249)
                    continue;

                // Also skip water boundary textures (WATERIN_IDX=247, WATEROUT_IDX=248)
                // — these are medium transitions, not acoustically reflective surfaces
                if (!isPortal && pi < cell.numTextured) {
                    uint8_t txtIdx = cell.texturing[pi].txt;
                    if (txtIdx >= 247)  // WATERIN, WATEROUT, BACKHACK — all non-physical
                        continue;
                }

                // Determine texture name for material lookup.
                // Rendered portal polygons (water/glass boundaries) use the _portal
                // material. Solid walls use their TXLIST texture for material matching.
                std::string texName = "generic";
                if (isPortal) {
                    texName = "_portal";
                } else if (pi < cell.numTextured) {
                    uint8_t txtIdx = cell.texturing[pi].txt;
                    if (txtIdx < mission.txList.textures.size()) {
                        const auto &entry = mission.txList.textures[txtIdx];
                        if (!entry.fullPath.empty())
                            texName = entry.fullPath;
                    }
                }

                // Skip degenerate polygons
                if (poly.count < 3)
                    continue;

                // Collect deduplicated vertex indices for this polygon
                std::vector<uint32_t> polyVerts(poly.count);
                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = cell.polyIndices[pi][vi];
                    const auto &v = cell.vertices[idx];
                    polyVerts[vi] = getVertexIndex(v.x, v.y, v.z);
                }

                // Fan-triangulate the polygon
                for (int t = 1; t < poly.count - 1; ++t) {
                    fullScene.indices.push_back(
                        static_cast<int32_t>(polyVerts[0]));
                    fullScene.indices.push_back(
                        static_cast<int32_t>(polyVerts[t + 1]));
                    fullScene.indices.push_back(
                        static_cast<int32_t>(polyVerts[t]));
                    fullScene.texNames.push_back(texName);
                }
            }
        }

        size_t numTris = fullScene.indices.size() / 3;
        size_t numVerts = fullScene.vertices.size() / 3;
        std::fprintf(stderr, "Acoustic mesh: %zu vertices, %zu triangles (%u cells)\n",
                     numVerts, numTris, wr.numCells);

        // Apply audio config before building the acoustic scene.
        // Subsections (audio.performance / .reflections / .occlusion / .propagation
        // / .spatialization / .ambient / .mixer / .dsp) all live as flat fields on
        // RenderConfig; the YAML loader fills them from the nested layout.

        // -- audio.performance (engine + sim throughput) --
        // Note: sample_rate / frame_size / sound_cache_mb take effect on the next
        // engine init (currently set during bootstrapFinished); change them via
        // YAML and restart for now.
        audioSvc->setAudioSampleRate(cfg.audioSampleRate);
        audioSvc->setAudioFrameSize(cfg.audioFrameSize);
        audioSvc->setSoundCacheMB(cfg.audioSoundCacheMB);
        audioSvc->setReflectionRateDivisor(cfg.reflectionRateDivisor);
        audioSvc->setMaxActiveVoices(cfg.maxActiveVoices);
        audioSvc->setReverbVoices(cfg.reverbVoices);
        audioSvc->setReverbVoicesRealtime(cfg.reverbVoicesRealtime);
        audioSvc->setReverbThreads(cfg.reverbThreads);
        audioSvc->setReverbThreadsConvShare(cfg.reverbThreadsConvShare);
        audioSvc->setReflectionThrottle(cfg.reflectionThrottle);
        audioSvc->setSimMaxOcclusionSamples(cfg.simMaxOcclusionSamples);
        audioSvc->setSceneType(cfg.sceneType);

        // -- audio.reflections --
        audioSvc->setReflectionsEnabled(cfg.realtimeReflections);
        audioSvc->setAmbisonicsOrder(cfg.ambisonicsOrder);
        // Reflection algorithm: HYBRID-only (Steam Audio's
        // `IPL_REFLECTIONEFFECTTYPE_HYBRID`). The legacy `type:` YAML key is
        // accepted but ignored with a [FALLBACK] warning — see RenderConfig.
        audioSvc->setHybridTransitionTime(cfg.hybridTransitionTime);
        audioSvc->setHybridOverlapPercent(cfg.hybridOverlapPercent);
        audioSvc->setRealtimeNumRays(cfg.realtimeNumRays);
        audioSvc->setRealtimeNumBounces(cfg.realtimeNumBounces);
        audioSvc->setRealtimeDuration(cfg.realtimeDuration);
        audioSvc->setRealtimeDiffuseSamples(cfg.realtimeDiffuseSamples);
        audioSvc->setBakeNumRays(cfg.bakeNumRays);
        audioSvc->setBakeNumBounces(cfg.bakeNumBounces);
        audioSvc->setBakeDuration(cfg.bakeDuration);
        audioSvc->setBakeDiffuseSamples(cfg.bakeDiffuseSamples);
        audioSvc->setBakeAmbisonicsOrder(cfg.bakeAmbisonicsOrder);
        audioSvc->setReflectionBakeSkip(cfg.reflectionBakeSkip);
        audioSvc->setForcePathingBake(cfg.forcePathingBake);

        // -- audio.probes --
        // Bake-time grid parameters. Applied before the auto-bake on first
        // run (so a denser YAML setting takes effect on the initial bake)
        // and read by manual bake_probes triggers from the debug console.
        audioSvc->setProbeSpacingFt(cfg.audioProbeSpacingFt);
        audioSvc->setProbeHeightFt(cfg.audioProbeHeightFt);
        audioSvc->setProbeElevations(cfg.audioProbeElevations);
        audioSvc->setProbeMinWallClearanceFt(cfg.audioProbeMinWallClearanceFt);
        audioSvc->setProbeElevationSparsityMul(cfg.audioProbeElevationSparsityMul);
        audioSvc->setProbeGlobalDedupRadiusFt(cfg.audioProbeGlobalDedupRadiusFt);
        audioSvc->setPathingProbeBatchEnabled(cfg.audioPathingProbesEnabled);
        audioSvc->setPathingDedupRadiusFt(cfg.audioPathingDedupRadiusFt);

        // -- audio.occlusion --
        audioSvc->setOcclusionRadius(cfg.occlusionRadius);
        audioSvc->setOcclusionSamples(cfg.occlusionSamples);
        audioSvc->setTransmissionScale(cfg.transmissionScale);
        audioSvc->setAbsorptionScale(cfg.absorptionScale);

        // -- audio.propagation --
        audioSvc->setPortalRoutingEnabled(cfg.portalRouting);
        audioSvc->setProbePathingEnabled(cfg.probePathing);
        audioSvc->setPropagationMaxDist(cfg.propagationMaxDist);
        audioSvc->setDoorLpfOpenHz(cfg.doorLpfOpenHz);
        audioSvc->setDoorLpfBlockedHz(cfg.doorLpfBlockedHz);
        audioSvc->setPropMinAttenuation(cfg.propMinAttenuation);
        audioSvc->setPropMaxPaths(cfg.propMaxPaths);
        audioSvc->setPropMaxPathDiff(cfg.propMaxPathDiff);
        audioSvc->setPathingGainScale(cfg.pathingGainScale);
        audioSvc->setPathingBlockingScale(cfg.pathingBlockingScale);
        audioSvc->setPathingUpdateInterval(cfg.pathingUpdateInterval);
        audioSvc->setPathingGainBandWeights(cfg.pathingGainWeightLow,
                                            cfg.pathingGainWeightMid,
                                            cfg.pathingGainWeightHigh);

        // -- audio.spatialization --
        audioSvc->setHRTFVolume(cfg.hrtfVolume);
        audioSvc->setHRTFInterpolation(cfg.hrtfInterpolation);
        audioSvc->setSpatialBlend(cfg.spatialBlend);

        // -- audio.ambient --
        audioSvc->setAmbHysteresisStartMul(cfg.ambHysteresisStartMul);
        audioSvc->setAmbHysteresisStopMul(cfg.ambHysteresisStopMul);
        audioSvc->setAmbDefaultPriority(cfg.ambDefaultPriority);
        audioSvc->setAmbEnvironmentalSpatialBlend(cfg.ambEnvironmentalSpatialBlend);
        audioSvc->setAmbGlobalVolumeScale(cfg.ambGlobalVolumeScale);

        // -- audio.mixer --
        audioSvc->setMasterGain(cfg.mixerMasterGain);
        audioSvc->setDirectGain(cfg.mixerDirectGain);
        audioSvc->setReflectionGain(cfg.mixerReflectionGain);
        audioSvc->setReflectionRampMs(cfg.reflectionRampMs);

        // -- audio.dsp (master bus chain: EQ → compressor → limiter; plus duck) --
        audioSvc->setDSPLimiterEnabled(cfg.dspLimiter);
        audioSvc->setDSPLimiterKnee(cfg.dspLimiterKnee);
        audioSvc->setDSPCompressorEnabled(cfg.dspCompressor);
        audioSvc->setDSPCompThreshold(cfg.dspCompThreshold);
        audioSvc->setDSPCompRatio(cfg.dspCompRatio);
        audioSvc->setDSPCompAttackMs(cfg.dspCompAttackMs);
        audioSvc->setDSPCompReleaseMs(cfg.dspCompReleaseMs);
        audioSvc->setDSPEQEnabled(cfg.dspEQ);
        audioSvc->setDSPEQFreq(cfg.dspEQFreq);
        audioSvc->setDSPEQGain(cfg.dspEQGain);
        audioSvc->setDSPEQQ(cfg.dspEQQ);
        audioSvc->setDSPDuckingEnabled(cfg.dspDucking);
        audioSvc->setDSPDuckAmount(cfg.dspDuckAmount);
        audioSvc->setDSPDuckAttackMs(cfg.dspDuckAttackMs);
        audioSvc->setDSPDuckReleaseMs(cfg.dspDuckReleaseMs);
        audioSvc->setDSPWetSaturationEnabled(cfg.dspWetSaturation);
        audioSvc->setDSPWetSaturationDrive(cfg.dspWetSaturationDrive);

        // Push audio-thread-side params (HRTF interp, door LPF, etc.) into
        // file-scope atomics so the next audio callback observes them.
        audioSvc->publishAudioThreadParams();

        // ── Open audio_perf.jsonl per-run artifact ──
        // PLAN.AUDIO_PROFILING.md §1.1/§1.2 — single emitter
        // (dumpAudioStatusPeriodic) fans every 5s window snapshot out to
        // BOTH stderr (live tail) AND this JSONL file. Open AFTER all
        // setters have run so the run.meta config snapshot matches what
        // the engine is actually using.
        {
            std::string cliArgvJoined;
            for (int i = 0; i < argc; ++i) {
                if (i) cliArgvJoined += ' ';
                cliArgvJoined += argv[i];
            }
            std::string audioCfgJson = serializeAudioConfigJson(cfg);
            std::string missionName  = deriveMissionName(misPath);
            audioSvc->openPerfJsonl(missionName, misPath ? misPath : "",
                                    cfg.perfLabel, cliArgvJoined, audioCfgJson);
        }

        if (!audioSvc->buildAcousticScene(fullScene)) {
            std::fprintf(stderr, "WARNING: failed to build acoustic scene\n"
                                 "         Steam Audio spatialization disabled.\n");
        }

        // Try to load baked probe data from the darkness install folder.
        // If no probes exist, auto-bake them (with a progress bar).
        // --no-probes skips baking entirely (no spatial audio, faster startup).
        if (!cfg.noProbes) {
            std::string probePath = Darkness::AudioService::getProbeFilePath(misPath);

            // ── [REFL_SKIP] startup banner ──
            //
            // When the user passed --skip-reflection-bake (or set
            // audio.reflections.bake_skip in YAML), the next bakeProbes()
            // call will carry the existing reflection section forward
            // verbatim instead of re-baking it. Emit a loud banner now so
            // the user sees what's about to happen — silently going from
            // "bake takes 5 minutes" to "bake takes 5 seconds" without
            // explanation would be a foot-gun (and per
            // feedback_no_silent_fallbacks every fallback must announce
            // itself). We inspect the existing file directly so we can
            // report probe count + file mtime (as a proxy for the
            // original bake time — the .probes format does not record a
            // bake timestamp).
            if (cfg.reflectionBakeSkip) {
                Darkness::ProbeFileHeader phdr;
                std::vector<Darkness::ProbeBatchRecord> precs;
                Darkness::ProbeFileStatus pst = Darkness::loadProbeFile(probePath, phdr, precs);
                int reflProbes = 0;
                bool haveRefl = false;
                for (const auto &rec : precs) {
                    if (rec.purpose == static_cast<uint32_t>(
                            Darkness::ProbePurpose::Reflections)) {
                        reflProbes = static_cast<int>(rec.probeCount);
                        haveRefl = true;
                        break;
                    }
                }
                if (pst == Darkness::ProbeFileStatus::Ok && haveRefl) {
                    // File mtime as best-effort bake-time proxy.
                    char timeBuf[64] = "unknown";
                    struct stat sb;
                    if (stat(probePath.c_str(), &sb) == 0) {
                        struct tm tmv;
                    #ifdef _WIN32
                        localtime_s(&tmv, &sb.st_mtime);
                    #else
                        localtime_r(&sb.st_mtime, &tmv);
                    #endif
                        std::strftime(timeBuf, sizeof(timeBuf),
                                      "%Y-%m-%d %H:%M", &tmv);
                    }
                    std::fprintf(stderr,
                        "[REFL_SKIP] reflection bake disabled — carrying forward existing reflection section\n"
                        "            from %s (%d probes, baked %s)\n"
                        "[REFL_SKIP] If you've moved geometry, audible reflections will be STALE until you\n"
                        "            rebuild with --skip-reflection-bake removed.\n",
                        probePath.c_str(), reflProbes, timeBuf);
                } else {
                    // No existing file (or no reflection section). The
                    // bake itself will hard-fail later with the same
                    // [FALLBACK] verbiage from ProbeManager; this early
                    // warning gives the user a chance to ^C before the
                    // bake even starts.
                    std::fprintf(stderr,
                        "[REFL_SKIP] reflection bake disabled but existing .probes file '%s' is missing\n"
                        "            or lacks a reflection section (status=%s, refl_section=%s).\n"
                        "[REFL_SKIP] bakeProbes() will REFUSE to produce a pathing-only file — re-bake\n"
                        "            with --skip-reflection-bake removed first.\n",
                        probePath.c_str(),
                        Darkness::probeFileStatusString(pst),
                        haveRefl ? "yes" : "no");
                }
            }

            if (!audioSvc->loadProbes(probePath)) {
                // No baked probes — need to bake.
                // This is done BEFORE the render loop starts, so we can use
                // a simple bgfx debug text progress bar during baking.
                state.probeBakePath = probePath;
                state.probeBakeNeeded = true;
            } else if (cfg.forcePathingBake) {
                // ── [PATHING_BAKE_FORCE] startup banner ──
                //
                // loadProbes() succeeded against the existing .probes file
                // BUT the user passed --force-pathing-bake — schedule a
                // bake anyway so the pathing section is regenerated fresh.
                // The bake invocation respects --skip-reflection-bake (via
                // mReflectionBakeSkip → ProbeBakeParams::bakeReflectionBatch),
                // so the canonical Sweep 2 Phase B composition
                //   --skip-reflection-bake --force-pathing-bake
                // carries reflection bytes forward and re-bakes pathing
                // fresh in seconds. See PLAN.AUDIO_PROFILING.md §4.3.
                //
                // Loud banner per feedback_no_silent_fallbacks — going
                // from "load probes (instant)" to "load probes then bake
                // for ~minutes (or ~seconds if --skip-reflection-bake)"
                // is a surprise without warning. The banner makes the
                // mode shift unmissable.
                std::fprintf(stderr,
                    "[PATHING_BAKE_FORCE] --force-pathing-bake: existing .probes "
                    "file '%s' loaded successfully, but the pathing section\n"
                    "                     will be re-baked anyway. Reflection "
                    "section will be %s.\n",
                    probePath.c_str(),
                    cfg.reflectionBakeSkip
                        ? "CARRIED FORWARD (--skip-reflection-bake)"
                        : "RE-BAKED (multi-minute; pass --skip-reflection-bake "
                          "to carry forward)");
                state.probeBakePath = probePath;
                state.probeBakeNeeded = true;
            }
        } else {
            std::fprintf(stderr, "Probe baking skipped (--no-probes)\n");
        }

        // Store acoustic mesh data for debug wireframe overlay AND the
        // per-frame `show_acoustic_hit` raycast highlighter. GPU buffer
        // is created later after bgfx::init; the CPU-side copy stays
        // resident for the lifetime of the mission so the raycast tool
        // can do Möller-Trumbore against every triangle each frame.
        // Memory is ~12 B/vertex × ~3·numTris + small per-tri name
        // string — single-digit MB on Thief 2 missions.
        state.acousticVerts = fullScene.vertices;
        state.acousticIndices = fullScene.indices;
        state.acousticTexNames = fullScene.texNames;
    }

    // Inject raycaster into the world query facade — enables ray-vs-world
    // queries (AI line-of-sight, physics, sound occlusion) via portal traversal
    worldQuery->setRaycaster(
        [&mission](const Darkness::Vector3 &from, const Darkness::Vector3 &to,
                  Darkness::RayHit &hit) {
            return Darkness::raycastWorld(mission.wrData, from, to, hit);
        });

    // Diagnostic raycaster for the [PATH] periodic|spike block in AudioService.
    // Lets the diagnostic answer "is the voice's nearest-probe LOS actually
    // blocked by BSP geometry?" — distinguishes a wall-embedded source from
    // a Steam Audio mesh-interpretation mismatch. Same WR data as the
    // worldQuery raycaster above; just exposes it to AudioService directly so
    // we don't need to round-trip through IWorldQuery for a one-off log.
    {
        auto audioSvcForRay = GET_SERVICE(Darkness::AudioService);
        if (audioSvcForRay) {
            audioSvcForRay->setRaycaster(
                [&mission](const Darkness::Vector3 &from,
                           const Darkness::Vector3 &to,
                           Darkness::RayHit &hit) {
                    return Darkness::raycastWorld(mission.wrData, from, to, hit);
                });
            // loadProbes ran BEFORE setRaycaster (above), so the first
            // pathing-adjacency build attempt was a no-op fallback. Now
            // that the raycaster is wired, rebuild so show_pathing_graph
            // has real edges. Cheap no-op when no pathing batch loaded.
            audioSvcForRay->rebuildPathingAdjacency();
        }
    }

    // Give the renderer access to the mutable object state map (owned by worldQuery)
    state.objectStates = &worldQuery->objectStates();

    // Bind the per-object illumination subsystem to the loaded WR data,
    // mission render parameters, and property service. Computation runs
    // per visible object per frame in the render loop.
    {
        Darkness::PropertyServicePtr illumPropSvc = GET_SERVICE(Darkness::PropertyService);

        // Synthesize the static-light-table bright field for animated
        // lights from anim_max + P$LightColor. Some torches were baked
        // in ANIM_MIN_BRIGHT mode at zero brightness, so their disk slot
        // has bright=(0,0,0) — without this overwrite the multiplier
        // mechanism scales nothing and switches don't visibly affect
        // those torches. Using anim_max as the base gives the multiplier
        // a meaningful 0..1 range to modulate against.
        constexpr float kBakeLightScale = 32.0f;
        int patched = 0;
        for (const auto &[lightNum, idx] : mission.animLightToStaticIdx) {
            auto lsIt = mission.lightSources.find(lightNum);
            if (lsIt == mission.lightSources.end()) continue;
            const auto &ls = lsIt->second;
            if (ls.maxBright <= 0.0f) continue;

            float hue = 0.0f, sat = 0.0f;  // default white
            Darkness::PropLightColor col{};
            if (ls.objectId != 0 &&
                Darkness::getTypedProperty<Darkness::PropLightColor>(
                    illumPropSvc.get(), "LightColo", ls.objectId, col)) {
                hue = col.hue;
                sat = col.saturation;
            }
            Darkness::Vector3 colorRgb = Darkness::hsbToRgb(hue, sat);
            Darkness::Vector3 effective =
                colorRgb * (ls.maxBright / kBakeLightScale);

            mission.wrData.staticLights[idx].bright = effective;
            ++patched;
        }
        std::fprintf(stderr,
            "  Synthesized bright for %d animated light slots\n", patched);

        state.objectIlluminator.setMissionData(&mission.wrData,
                                               &mission.renderParams,
                                               illumPropSvc.get());
        state.objectIlluminator.setDynamicLights(&state.dynamicLights);
    }

    // Declare DoorSystem early so frob/message systems can reference it.
    // Actual init is deferred until after loadObjectAssets (which populates
    // allPlacements with position/angle data for ALL concrete objects).
    Darkness::DoorSystem doorSystem;
    state.doorSystem = &doorSystem;

    Darkness::TweqSystem tweqSystem;
    Darkness::MovingTerrainSystem movingTerrainSystem;
    state.movingTerrainSystem = &movingTerrainSystem;

    Darkness::PressurePlateSystem pressurePlateSystem;
    Darkness::EdgeTriggerSystem edgeTriggerSystem;

    // ── Initialize frob system ──
    // Casts a short ray from the camera each frame to find the nearest frobbable
    // object (doors, switches, pickups). Right-click triggers the frob action.
    Darkness::FrobSystem frobSystem;
    {
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        Darkness::ObjectServicePtr objSvc = GET_SERVICE(Darkness::ObjectService);
        Darkness::ObjectCollisionWorld *ocw =
            state.physics ? state.physics->getObjectCollisionWorld() : nullptr;
        frobSystem.init(propSvc.get(), objSvc.get(), &doorSystem, ocw);
        // MessageDispatch connected below after creation
    }
    state.frobSystem = &frobSystem;

    // ── Initialize grab system ──
    // Carries the held object lifecycle (pickup, per-frame attach to camera,
    // drop, throw). Lives outside FrobSystem so future grip modes (force
    // drag, hinge drag) can be added without touching frob targeting.
    Darkness::GrabSystem grabSystem;
    if (state.physics) {
        grabSystem.init(state.physics.get(), state.objectStates);
    }
    state.grabSystem = &grabSystem;

    // ── Initialize message dispatch ──
    // Routes script messages (TurnOn, TurnOff, FrobWorldEnd) to handlers.
    // ControlDevice link traversal: frobbing a lever sends TurnOn to linked objects.
    Darkness::MessageDispatch messageDispatch;
    messageDispatch.init(worldQuery.get());

    // Register built-in door handler: doors respond to TurnOn/TurnOff/FrobWorldEnd
    messageDispatch.registerGlobalHandler("TurnOn", [&doorSystem](const Darkness::ScriptMessage &msg) {
        return doorSystem.activate(msg.to, Darkness::kDoorDoOpen);
    });
    messageDispatch.registerGlobalHandler("TurnOff", [&doorSystem](const Darkness::ScriptMessage &msg) {
        return doorSystem.activate(msg.to, Darkness::kDoorDoClose);
    });
    // Tweq handlers: TurnOn/TurnOff activate/halt tweqs. Return false to
    // avoid consuming the message — other systems (doors, scripts) may also respond.
    messageDispatch.registerGlobalHandler("TurnOn", [&tweqSystem](const Darkness::ScriptMessage &msg) {
        tweqSystem.activate(msg.to, Darkness::kTweqDoActivate);
        return false;  // never consume — multiple systems may respond
    });
    messageDispatch.registerGlobalHandler("TurnOff", [&tweqSystem](const Darkness::ScriptMessage &msg) {
        tweqSystem.activate(msg.to, Darkness::kTweqDoHalt);
        return false;
    });
    // Moving terrain handlers: TurnOn/TurnOff activate/deactivate platforms.
    // Return false to avoid consuming — other systems may also respond.
    messageDispatch.registerGlobalHandler("TurnOn", [&movingTerrainSystem](const Darkness::ScriptMessage &msg) {
        movingTerrainSystem.activate(msg.to);
        return false;
    });
    messageDispatch.registerGlobalHandler("TurnOff", [&movingTerrainSystem](const Darkness::ScriptMessage &msg) {
        movingTerrainSystem.deactivate(msg.to);
        return false;
    });

    messageDispatch.registerGlobalHandler("FrobWorldEnd",
        [&doorSystem](const Darkness::ScriptMessage &msg) {
            // Dark Engine convention: locked doors reject frob when closed.
            // Lock check only applies to frob — TurnOn/TurnOff from scripts/levers
            // can still open locked doors, matching original engine behavior.
            if (doorSystem.isDoor(msg.to) &&
                doorSystem.getStatus(msg.to) == Darkness::kDoorClosed) {
                auto propSvc = GET_SERVICE(Darkness::PropertyService);
                Darkness::PropLocked locked{};
                if (propSvc &&
                    Darkness::getTypedProperty<Darkness::PropLocked>(
                        propSvc.get(), "Locked", msg.to, locked) &&
                    locked.isLocked != 0) {
                    // Play reject sound: env_tag (Event Reject)(Operation OpenDoor)
                    // plus the door's ClassTags (e.g. DoorType Metal)
                    auto audioSvc = GET_SERVICE(Darkness::AudioService);
                    const auto *door = doorSystem.getDoor(msg.to);
                    if (audioSvc && door) {
                        std::vector<Darkness::SchemaTagValue> tags;
                        Darkness::SchemaTagValue evtTag;
                        evtTag.tagName = "Event";
                        evtTag.enumValues.push_back("Reject");
                        tags.push_back(evtTag);
                        Darkness::SchemaTagValue opTag;
                        opTag.tagName = "Operation";
                        opTag.enumValues.push_back("OpenDoor");
                        tags.push_back(opTag);
                        // Add ClassTags from the door object (e.g. DoorType Metal)
                        struct { uint32_t value; char text[252]; } classTags = {};
                        for (const char *pname : {"ClassTags", "Class Tags", "Class Tag"}) {
                            if (Darkness::getTypedProperty<decltype(classTags)>(
                                    propSvc.get(), pname, msg.to, classTags))
                                break;
                        }
                        std::string tagStr(classTags.text,
                            strnlen(classTags.text, sizeof(classTags.text)));
                        if (!tagStr.empty()) {
                            std::istringstream iss(tagStr);
                            std::string key, val;
                            while (iss >> key >> val) {
                                Darkness::SchemaTagValue t;
                                t.tagName = key;
                                t.enumValues.push_back(val);
                                tags.push_back(std::move(t));
                            }
                        }
                        // Play on both sides of the door (same as open/close sounds)
                        Darkness::Matrix3 doorRot(door->baseRotation);
                        Darkness::Vector3 doorNormal = doorRot[1];
                        audioSvc->playEnvSchema(tags,
                            door->basePosition + doorNormal * 0.5f, true);
                        audioSvc->playEnvSchema(tags,
                            door->basePosition - doorNormal * 0.5f, true);
                    }
                    std::fprintf(stderr, "Door %d: LOCKED, rejecting frob\n", msg.to);
                    return true;  // consumed — do not toggle
                }
            }
            return doorSystem.activate(msg.to, Darkness::kDoorToggle);
        });

    // Connect FrobSystem to MessageDispatch for ControlDevice link traversal
    frobSystem.setMessageDispatch(&messageDispatch);
    frobSystem.setWorldQuery(worldQuery.get());
    frobSystem.setObjectStates(state.objectStates);  // live-position source for cone path
    // frobSystem.setScriptManager wired below after ScriptManager declaration
    state.messageDispatch = &messageDispatch;

    // ── Initialize Script System (Phase 6) ──
    // ScriptManager reads P$Scripts from all concrete objects and instantiates
    // C++ script classes (StdDoor, StdLever, AnimLight, traps, triggers, etc.).
    // Scripts receive messages first; unhandled messages fall through to the
    // global MessageDispatch handlers above (backward compat).
    Darkness::ScriptManager scriptManager;
    Darkness::ObjectPushSystem objectPushSystem;

    // Script service wrappers — thin facades around existing OPDE services.
    // These must outlive the ScriptManager (scripts hold pointers to them).
    Darkness::LinkScriptService linkScriptSvc;
    Darkness::ObjectScriptService objectScriptSvc;
    Darkness::PropertyScriptService propertyScriptSvc;
    Darkness::SoundScriptService soundScriptSvc;
    Darkness::LightScriptService lightScriptSvc;
    lightScriptSvc.lightSources = &mission.lightSources;
    Darkness::DamageScriptService damageScriptSvc;
    Darkness::DataScriptService dataScriptSvc;
    Darkness::LockedScriptService lockedScriptSvc;
    Darkness::PhysicsScriptService physicsScriptSvc;
    Darkness::DoorScriptService doorScriptSvc;
    Darkness::ContainerScriptService containerScriptSvc;
    Darkness::DarkGameScriptService darkGameScriptSvc;
    Darkness::DarkUIScriptService darkUIScriptSvc;
    Darkness::IScriptServices scriptServices;  // aggregate — must outlive scripts

    // Wire service wrappers to backing OPDE services
    {
        auto propSvc = GET_SERVICE(Darkness::PropertyService);
        auto linkSvc = GET_SERVICE(Darkness::LinkService);
        auto objSvc = GET_SERVICE(Darkness::ObjectService);

        linkScriptSvc.linkSvc = linkSvc.get();
        objectScriptSvc.objSvc = objSvc.get();
        objectScriptSvc.worldQuery = worldQuery.get();
        propertyScriptSvc.propSvc = propSvc.get();
        {
            auto audioSvc = GET_SERVICE(Darkness::AudioService);
            soundScriptSvc.audioSvc = audioSvc.get();
            std::fprintf(stderr, "[SoundScriptService] audioSvc=%p\n",
                         (void*)soundScriptSvc.audioSvc);
        }
        soundScriptSvc.placements = &mission.objData.allPlacements;
        soundScriptSvc.propSvc = propSvc.get();
        damageScriptSvc.msgDispatch = &messageDispatch;
        lockedScriptSvc.propSvc = propSvc.get();
        lockedScriptSvc.msgDispatch = &messageDispatch;
        doorScriptSvc.doorSys = &doorSystem;
        containerScriptSvc.linkSvc = linkSvc.get();

        // Populate aggregate service struct for scripts
        scriptServices.propertyService = propSvc.get();
        scriptServices.linkService = linkSvc.get();
        scriptServices.objectService = objSvc.get();
        scriptServices.doorSystem = &doorSystem;
        scriptServices.tweqSystem = &tweqSystem;
        scriptServices.movingTerrainSystem = &movingTerrainSystem;
        scriptServices.messageDispatch = &messageDispatch;
        scriptServices.link = &linkScriptSvc;
        scriptServices.object = &objectScriptSvc;
        scriptServices.property = &propertyScriptSvc;
        scriptServices.sound = &soundScriptSvc;
        scriptServices.light = &lightScriptSvc;
        scriptServices.damage = &damageScriptSvc;
        scriptServices.data = &dataScriptSvc;
        scriptServices.locked = &lockedScriptSvc;
        scriptServices.physics = &physicsScriptSvc;
        scriptServices.door = &doorScriptSvc;
        scriptServices.container = &containerScriptSvc;
        scriptServices.darkGame = &darkGameScriptSvc;
        scriptServices.darkUI = &darkUIScriptSvc;

        // Initialize ScriptManager and instantiate scripts from P$Scripts
        scriptManager.init(propSvc.get(), linkSvc.get(), objSvc.get(),
                           &messageDispatch, &scriptServices);
        scriptManager.instantiateScripts();

        // ObjectPushSystem init deferred until after buildObjectCollision (below)
    }

    // Wire ScriptManager to frob system and state (after ScriptManager is declared)
    frobSystem.setScriptManager(&scriptManager);
    state.scriptManager = &scriptManager;

    // ── Load world textures: TXLIST, fam.crf textures, flow textures, skybox ──
    loadWorldTextures(misPath, resPath, mission);

    // ── Build per-texture friction table from P$Friction on texture archetypes ──
    // The original engine maps each TXLIST texture index to a "t_fam/<family>/<name>"
    // archetype in dark.gam, then reads P$Friction (float, default 1.0) from that
    // archetype (with MetaProp inheritance). We build a flat lookup table at load time
    // to avoid string lookups per physics step.
    //
    // TODO: The shipping Thief 2 gamesys only has 3 P$Friction records (city/roof,
    // city/rooftile at 0.15, and object 2 at 0.0). Ice/snow textures (vmawwin/iceclif1
    // etc.) have no friction override — verify whether the original engine has additional
    // friction sources beyond P$Friction (e.g. texture-name heuristics, per-mission
    // overrides, or Climbability affecting friction). May need a name-based fallback
    // or manual friction table for ice/snow surfaces if gameplay testing reveals issues.
    {
        Darkness::ObjectServicePtr objSvc = GET_SERVICE(Darkness::ObjectService);
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        size_t numTex = mission.txList.textures.size();
        mission.frictionTable.assign(numTex, 1.0f);

        int nonDefault = 0;
        for (size_t i = 0; i < numTex; ++i) {
            const auto &entry = mission.txList.textures[i];
            if (entry.family.empty() && entry.name.empty())
                continue;

            // Texture archetypes in dark.gam are named "t_fam/<family>/<name>"
            // with original case preserved (objectNamed is case-sensitive).
            std::string archName = "t_fam/" + entry.fullPath;

            int archID = objSvc->named(archName);
            if (archID == 0) continue;  // no archetype found for this texture

            float friction = 1.0f;
            if (Darkness::getTypedProperty<float>(propSvc.get(), "Friction", archID, friction)) {
                if (std::fabs(friction - 1.0f) > 0.001f) {
                    mission.frictionTable[i] = friction;
                    ++nonDefault;
                    std::fprintf(stderr, "[FRICTION] %s: %.2f (idx %zu)\n",
                                 archName.c_str(), friction, i);
                }
            }
        }
        std::fprintf(stderr, "[FRICTION] %d textures with non-default friction out of %zu loaded\n",
                     nonDefault, numTex);

        // Pass friction table to player physics
        if (state.physics) {
            state.physics->setFrictionTable(mission.frictionTable);
        }
    }

    // ── Build per-texture climbability table from P$Climbabil ──
    // Same pattern as friction table above. P$Climbabil is a single float
    // "factor" on texture archetypes. Non-zero values boost friction on steep
    // surfaces (helping the player walk up ramps) — this is NOT a climb-mode
    // trigger. The original engine disabled terrain wall climbing in shipping
    // Thief 2; only OBB objects with climbable_sides support actual climbing.
    {
        Darkness::ObjectServicePtr objSvc = GET_SERVICE(Darkness::ObjectService);
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        size_t numTex = mission.txList.textures.size();
        mission.climbabilityTable.assign(numTex, 0.0f);

        int nonDefault = 0;
        for (size_t i = 0; i < numTex; ++i) {
            const auto &entry = mission.txList.textures[i];
            if (entry.family.empty() && entry.name.empty())
                continue;

            std::string archName = "t_fam/" + entry.fullPath;
            int archID = objSvc->named(archName);
            if (archID == 0) continue;

            float climbability = 0.0f;
            if (Darkness::getTypedProperty<float>(propSvc.get(), "Climbabil", archID, climbability)) {
                if (climbability > 0.001f) {
                    mission.climbabilityTable[i] = climbability;
                    ++nonDefault;
                    std::fprintf(stderr, "[CLIMBABILITY] %s: %.2f (idx %zu)\n",
                                 archName.c_str(), climbability, i);
                }
            }
        }
        std::fprintf(stderr, "[CLIMBABILITY] %d textures with non-zero climbability out of %zu loaded\n",
                     nonDefault, numTex);

        // Pass climbability table to player physics
        if (state.physics) {
            state.physics->setClimbabilityTable(mission.climbabilityTable);
        }
    }

    // ── Build per-texture material keyword table for footstep sounds ──
    // Maps each TXLIST texture index → material keyword ("stone", "metal", etc.)
    // using substring matching on texture family/name paths.
    {
        Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);
        size_t numTex = mission.txList.textures.size();
        std::vector<std::string> materials(numTex);
        for (size_t i = 0; i < numTex; ++i) {
            const auto &entry = mission.txList.textures[i];
            if (!entry.fullPath.empty()) {
                materials[i] = Darkness::lookupAcousticMaterialKeyword(entry.fullPath);
            } else {
                materials[i] = "generic";
            }
        }
        audioSvc->setTextureMaterials(std::move(materials));

        // Register footstep and landing callbacks with physics
        if (state.physics) {
            state.physics->setFootstepCallback(
                [audioSvc](const Darkness::Vector3 &pos, float speed, int matIdx) {
                    audioSvc->playFootstep(pos, speed, matIdx);
                });
            state.physics->setLandingCallback(
                [audioSvc](const Darkness::Vector3 &pos, float fallSpeed, int matIdx) {
                    audioSvc->playLanding(pos, fallSpeed, matIdx);
                });
        }
    }

    // ── Load object assets: properties, .bin models, textures from obj.crf ──
    loadObjectAssets(misPath, resPath, cfg, mission, state);

    // ── Initialize door system (after loadObjectAssets so allPlacements is populated) ──
    // Build objID → placement map from ALL positioned objects (not just rendered
    // ones). Doors have RenderType=NotRendered and are filtered from
    // mission.objData.objects, but DoorSystem needs their raw angles.
    // NOTE: doorPlacements must outlive doorSystem — it stores a raw pointer.
    std::unordered_map<int32_t, Darkness::ObjPlacementInfo> doorPlacements;
    for (const auto &[id, obj] : mission.objData.allPlacements) {
        Darkness::ObjPlacementInfo pi = {
            obj.x, obj.y, obj.z,
            obj.heading, obj.pitch, obj.bank,
            obj.scaleX, obj.scaleY, obj.scaleZ,
            {}
        };
        std::memcpy(pi.modelName, obj.modelName, 16);
        doorPlacements[id] = pi;
    }
    {
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        Darkness::ObjectServicePtr objSvc = GET_SERVICE(Darkness::ObjectService);
        doorSystem.init(propSvc.get(), objSvc.get(), state.objectStates,
                        &mission.parsedModels, &doorPlacements);

        // Log locked doors
        int lockedCount = 0;
        for (int32_t did : doorSystem.getAllDoorIDs()) {
            Darkness::PropLocked locked{};
            if (Darkness::getTypedProperty<Darkness::PropLocked>(
                    propSvc.get(), "Locked", did, locked) &&
                locked.isLocked != 0) {
                ++lockedCount;
            }
        }
        if (lockedCount > 0) {
            std::fprintf(stderr, "  %d doors locked (P$Locked)\n", lockedCount);
        }
    }

    // ── Initialize tweq system ──
    // Scans for objects with tweq config properties (Rotate, Scale, Flicker, Models)
    // and creates per-object animation instances. Auto-starts tweqs with Sim flag.
    {
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        tweqSystem.init(propSvc.get(), state.objectStates, &doorPlacements,
                        &mission.parsedModels);
    }

    // ── Initialize moving terrain system ──
    // Scans for objects with P$MovingTerrain property and builds waypoint graphs
    // from TPathInit/TPath link chains. Platforms follow waypoint paths at
    // configured speeds, pausing at each waypoint.
    {
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        Darkness::ObjectServicePtr objSvc = GET_SERVICE(Darkness::ObjectService);
        Darkness::LinkServicePtr linkSvc = GET_SERVICE(Darkness::LinkService);
        movingTerrainSystem.init(propSvc.get(), objSvc.get(), linkSvc.get(),
                                 state.objectStates, &doorPlacements);
        // Waypoint callback wired below, after ScriptManager is declared.
    }

    // ── Initialize pressure plate system ──
    // Scans for objects with P$PhysPPlat property. Plates depress when the
    // player stands on them and send TurnOn/TurnOff via ControlDevice links.
    {
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        Darkness::ObjectServicePtr objSvc = GET_SERVICE(Darkness::ObjectService);
        pressurePlateSystem.init(propSvc.get(), objSvc.get(),
                                 state.objectStates, &doorPlacements);
        pressurePlateSystem.setMessageDispatch(&messageDispatch);
    }

    // ── Build object collision bodies from .bin bounding boxes ──
    // Creates OBB/sphere collision bodies for placed objects (crates, furniture,
    // doors) so the player can't walk through them. Uses P$PhysType properties
    // to determine shape type and P$PhysDims for optional dimension overrides.
    if (state.physics && !mission.objData.objects.empty()) {
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        state.physics->buildObjectCollision(
            mission.objData.objects, mission.parsedModels, propSvc.get());

        // Create ODE geoms + dynamic bodies. Kinematic objects (doors, platforms,
        // pressure plates) are excluded — they're driven by their own sim systems.
        state.physics->setObjectStates(state.objectStates);
        state.physics->buildODEGeoms(propSvc.get(),
            [&doorSystem, &movingTerrainSystem, &pressurePlateSystem](int32_t objID) {
                return doorSystem.isDoor(objID) ||
                       movingTerrainSystem.isPlatform(objID) ||
                       pressurePlateSystem.isPlate(objID);
            });
        // Build world geometry trimesh so dynamic objects collide with floors/walls
        state.physics->buildWorldTrimesh();

        // Initialize edge trigger system from collision bodies with isEdgeTrigger=true
        edgeTriggerSystem.init(state.physics->getObjectCollisionWorld());
        edgeTriggerSystem.setMessageDispatch(&messageDispatch);

        // Initialize ObjectPushSystem (Task 61) — must be after buildObjectCollision
        // so collision bodies exist for the pushability check.
        objectPushSystem.init(propSvc.get(), state.objectStates,
                              state.physics->getObjectCollisionWorld(), &doorSystem);
        objectPushSystem.autoRegisterPushable();

        // Promote pushable objects to ODE dynamic bodies. ODE handles gravity,
        // object-object collision, wall collision (via world trimesh), and settling.
        // This replaces ObjectPushSystem's kinematic Euler integration.
        {
            int activated = 0;
            for (int32_t objID : objectPushSystem.getPushableObjects()) {
                float mass = objectPushSystem.getMass(objID);
                float friction = objectPushSystem.getFriction(objID);
                (void)friction;  // retained for future tuning / informational lookups
                // P$PhysAttr.friction was a kinematic decel coefficient in the
                // original engine, not a Coulomb µ. Pass a reduced value into ODE
                // so the slide is visible while still settling within ~1-2 s.
                // At g=32 this gives ~1.6 u/s² Coulomb deceleration on flat
                // ground, comparable to the original engine's effective slide
                // friction.
                constexpr float kODEPushFriction = 0.20f;
                // Elasticity: low for crates/barrels (0.05), matching original engine's
                // 0.02 bounce dampening. Slightly higher than zero so objects don't
                // feel perfectly dead on impact.
                if (state.physics->makeDynamic(objID, mass, kODEPushFriction, 0.05f))
                    ++activated;
            }
            std::fprintf(stderr, "[ODE] Activated %d/%zu pushable objects as dynamic bodies\n",
                         activated, objectPushSystem.getPushableObjects().size());

            // One-shot snap: any dynamic body placed mid-air in the
            // level is dropped straight down to rest on the surface
            // below it. Mirrors what the original level editor's
            // drop-to-floor pass did at edit time — pure Z translation,
            // no rotation or bounce. Stacked props snap bottom-up so
            // upper boxes rest on the freshly-snapped lower ones.
            state.physics->snapDynamicBodiesToRest();
        }

        // Wire ODE dynamic body check so ObjectPushSystem skips ODE-handled objects
        objectPushSystem.setHasDynamicBodyCb([&state](int32_t objID) {
            return state.physics->hasDynamicBody(objID);
        });
        state.physics->setPushSystem(&objectPushSystem);

        // Build frob cache for objects without collision bodies (levers, switches, etc.)
        // Uses allPlacements for reliable position data (P$Position binary format).
        frobSystem.buildFrobCache(mission.objData.allPlacements);
    }

    // Wire door collision updates: when doors animate, update their collision
    // OBB position so the player can walk through open doors.
    {
        Darkness::ObjectCollisionWorld *ocw =
            state.physics ? state.physics->getObjectCollisionWorld() : nullptr;
        if (ocw) {
            doorSystem.setCollisionUpdateCallback(
                [ocw](int32_t objID, const Darkness::Matrix4 &worldMatrix) {
                    ocw->updateBodyTransform(objID, worldMatrix);
                });
            // Moving terrain also needs collision body updates
            movingTerrainSystem.setCollisionUpdateCallback(
                [ocw](int32_t objID, const Darkness::Matrix4 &worldMatrix) {
                    ocw->updateBodyTransform(objID, worldMatrix);
                });
            // Pressure plates also need collision body updates
            pressurePlateSystem.setCollisionUpdateCallback(
                [ocw](int32_t objID, const Darkness::Matrix4 &worldMatrix) {
                    ocw->updateBodyTransform(objID, worldMatrix);
                });
        }
    }

    // Wire platform riding: PlayerPhysics queries platform velocity to carry
    // the player when standing on a moving elevator/platform.
    if (state.physics) {
        state.physics->getPlayerPhysics().setPlatformVelocityCallback(
            [&movingTerrainSystem](int32_t objID) -> const Darkness::Vector3 * {
                return movingTerrainSystem.getVelocity(objID);
            });
    }

    // Wire pressure plate weight detection: checks if player is standing on
    // a plate's collision body via ground contact object IDs.
    if (state.physics) {
        pressurePlateSystem.setPlayerOnObjectCallback(
            [&state](int32_t objID) -> bool {
                return state.physics->getPlayerPhysics().isStandingOnObject(objID);
            });
    }

    // Wire edge trigger player position query for volume entry/exit detection.
    if (state.physics) {
        edgeTriggerSystem.setPlayerPositionCallback(
            [&state]() -> Darkness::Vector3 {
                return state.physics->getPlayerPosition();
            });
    }

    // ── SDL2 + bgfx init ──
    Darkness::GPUResources gpu;
    Darkness::BuiltMeshes meshes;

    // state.skyClearColor set by initWindow
    SDL_Window *window = initWindow(mission.fogParams, state.skyClearColor);
    if (!window) return 1;

    // ── Create all GPU resources: shaders, lightmap atlas, world/object/sky buffers ──
    float camX, camY, camZ;
    if (!createGPUResources(mission, cfg, state.showObjects, meshes, gpu,
                            camX, camY, camZ)) {
        shutdownWindow(window);
        return 1;
    }

    // ── Create acoustic mesh debug wireframe buffer (after bgfx init) ──
    if (!state.acousticIndices.empty()) {
        size_t numTris = state.acousticIndices.size() / 3;
        std::vector<Darkness::PosColorVertex> lineVerts;
        lineVerts.reserve(numTris * 6);
        uint32_t wireColor = 0x80FFFF00;  // ABGR: semi-transparent cyan
        for (size_t t = 0; t < numTris; ++t) {
            int32_t i0 = state.acousticIndices[t * 3 + 0];
            int32_t i1 = state.acousticIndices[t * 3 + 1];
            int32_t i2 = state.acousticIndices[t * 3 + 2];
            float x0 = state.acousticVerts[i0*3], y0 = state.acousticVerts[i0*3+1], z0 = state.acousticVerts[i0*3+2];
            float x1 = state.acousticVerts[i1*3], y1 = state.acousticVerts[i1*3+1], z1 = state.acousticVerts[i1*3+2];
            float x2 = state.acousticVerts[i2*3], y2 = state.acousticVerts[i2*3+1], z2 = state.acousticVerts[i2*3+2];
            lineVerts.push_back({x0,y0,z0,wireColor}); lineVerts.push_back({x1,y1,z1,wireColor});
            lineVerts.push_back({x1,y1,z1,wireColor}); lineVerts.push_back({x2,y2,z2,wireColor});
            lineVerts.push_back({x2,y2,z2,wireColor}); lineVerts.push_back({x0,y0,z0,wireColor});
        }
        state.acousticLineCount = static_cast<uint32_t>(lineVerts.size());
        const bgfx::Memory *mem = bgfx::copy(lineVerts.data(),
            static_cast<uint32_t>(lineVerts.size() * sizeof(Darkness::PosColorVertex)));
        state.acousticVBH = bgfx::createVertexBuffer(mem, Darkness::PosColorVertex::layout);
        std::fprintf(stderr, "Acoustic debug mesh: %zu tris, %u line verts\n",
                     numTris, state.acousticLineCount);
        // Note: the CPU-side acousticVerts / acousticIndices /
        // acousticTexNames are intentionally retained for the
        // `show_acoustic_hit` raycast tool. Cost is single-digit MB on
        // Thief 2 missions and survives only for the mission's lifetime.

        // (acousticHitVBH is unused — show_acoustic_hit allocates a
        // transient VB each frame instead, matching the show_raycast
        // pattern. State field kept for ABI stability with any external
        // tools; safe to remove in a follow-up.)
    }

    // ── Build per-room debug geometry (room/portal wireframe overlay) ──
    //
    // For each Room we compute the corners of its bounding polytope (the
    // half-space intersection of its 6 stored planes), the edges between
    // those corners, and the polygon boundary of each of its portals.
    // Geometry is stored PER ROOM so the renderer can sort by camera
    // distance each frame and emit only the closest N into a transient
    // vertex buffer — the room database can run into the hundreds, and
    // drawing every wireframe + ID label at once is unreadable.
    //
    // Build is one-shot — the room database is static for the life of the
    // mission. Toggle via the `show_rooms` / `show_portals` booleans;
    // adjust the cull cap via `debug_room_max_count`.
    {
        Darkness::RoomServicePtr roomSvc = GET_SERVICE(Darkness::RoomService);
        if (roomSvc && roomSvc->isLoaded()) {
            const auto &rooms = roomSvc->getAllRooms();
            state.roomDebug.clear();
            state.roomDebug.reserve(rooms.size());

            // Uniform light-pink color for portals — the BFS treats
            // every ROOM_DB portal as a potentially-traversable opening
            // and runs per-bend BSP validation at chain reconstruction,
            // so there's no "phantom portal" status to colour here.
            constexpr uint32_t kLightPink  = 0xFFB4B4FFu;  // ABGR

            size_t totalRoomVerts = 0;
            size_t totalPortalVerts = 0;
            size_t totalLabels = 0;

            for (const auto &rp : rooms) {
                if (!rp) continue;
                Darkness::Room *room = rp.get();
                int16_t roomID = room->getRoomID();
                uint32_t color = Darkness::colorFromRoomID(roomID);

                Darkness::RuntimeState::PerRoomDebug prd;
                prd.center = room->getCenter();
                prd.roomID = roomID;

                // Room polytope edges.
                const Plane *planes = room->getBoundingPlanes();
                auto corners = Darkness::computeRoomCorners(planes);
                auto edges = Darkness::computeRoomEdges(corners);
                prd.roomLines.reserve(edges.size() * 2);
                for (const auto &e : edges) {
                    const auto &a = corners[e.a].pos;
                    const auto &b = corners[e.b].pos;
                    prd.roomLines.push_back({a.x, a.y, a.z, color});
                    prd.roomLines.push_back({b.x, b.y, b.z, color});
                }

                // Portal polygons. The show_rooms overlay paints them in
                // the same per-room color so they read as "openings owned
                // by this room" (a portal appears once from each side, so
                // doorways naturally render in two overlaid colors). The
                // show_portals overlay paints the SAME polygons in
                // uniform light pink so they stand out from world
                // architecture — used to trace which polygons the BFS
                // chain is threading.
                uint32_t pc = room->getPortalCount();
                for (uint32_t pi = 0; pi < pc; ++pi) {
                    Darkness::RoomPortal *portal = room->getPortal(pi);
                    if (!portal) continue;
                    auto poly = Darkness::computePortalPolygon(*portal);
                    if (poly.size() < 2) continue;
                    uint32_t portalColor = kLightPink;
                    // Closed line strip: emit consecutive pairs, including
                    // the wrap-around segment from last to first vertex.
                    for (size_t v = 0; v < poly.size(); ++v) {
                        const auto &a = poly[v];
                        const auto &b = poly[(v + 1) % poly.size()];
                        prd.roomLines.push_back({a.x, a.y, a.z, color});
                        prd.roomLines.push_back({b.x, b.y, b.z, color});
                        prd.portalLines.push_back({a.x, a.y, a.z, portalColor});
                        prd.portalLines.push_back({b.x, b.y, b.z, portalColor});
                    }
                }

                // Record the room ID label at the center (used to mark
                // the listener's current room with a "*") and at every
                // polytope corner. Per-corner labels are essential when
                // visually-overlapping subdivisions share a screen area:
                // the center label may be hidden inside geometry or
                // collide with another room's label, but corner labels
                // appear at the actual wireframe vertices so each
                // wireframe is reliably tagged.
                prd.labels.reserve(1 + corners.size());
                prd.labels.push_back({room->getCenter(), roomID, true});
                for (const auto &c : corners) {
                    prd.labels.push_back({c.pos, roomID, false});
                }

                totalRoomVerts   += prd.roomLines.size();
                totalPortalVerts += prd.portalLines.size();
                totalLabels      += prd.labels.size();
                state.roomDebug.push_back(std::move(prd));
            }

            std::fprintf(stderr,
                "Room debug overlay: %zu rooms, %zu room line verts, "
                "%zu portal line verts, %zu labels (toggle with "
                "`show_rooms` / `show_portals`; cap with "
                "`debug_room_max_count`)\n",
                state.roomDebug.size(), totalRoomVerts,
                totalPortalVerts, totalLabels);
        }
    }

    // ── Initialize runtime state: mode string, model isolation, spawn/camera ──
    initRuntimeState(mission, meshes, gpu, camX, camY, camZ, state);

    // Initialize physics player at spawn position.
    // Use the camera position (eye level) directly as initial body center.
    // Gravity will pull the player down to the floor within a fraction of a
    // second, settling them at the correct standing height.  Subtracting
    // HEAD_OFFSET here would risk placing the body center below the floor
    // (outside all WR cells), which disables collision entirely.
    if (state.physics) {
        Darkness::Vector3 bodyPos(state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
        state.physics->setPlayerPosition(bodyPos);
        state.physics->setPlayerYaw(state.cam.yaw);
        std::fprintf(stderr, "Physics: player spawned at body (%.1f, %.1f, %.1f)\n",
                     bodyPos.x, bodyPos.y, bodyPos.z);
    }

    // ── Debug console for runtime settings management ──
    // Opened with backtick (`), provides tab-completion and value editing.
    Darkness::DebugConsole dbgConsole;
    registerConsoleSettings(dbgConsole, state, window, misPath);

    updateTitleBar(window, state);

    // NOTE: auto-bake is intentionally deferred past door registration
    // below — the bake reads mDoorAudioInstances (registered immediately
    // after the LoopService block) to (1) classify which portals are
    // doors so it can emit pair-probes flanking them, and (2) reject
    // any candidate probe whose position falls inside a door OBB. Both
    // become silent no-ops if the bake runs before doors are registered,
    // which is the easy regression to introduce when moving these blocks
    // around. See the corresponding `if (state.probeBakeNeeded)` block
    // below the door-registration block.

    // ── Set up LoopService for priority-ordered frame dispatch ──
    //
    // Execution order per frame (by LoopClient priority):
    //   1. InputClient     (priority 1)    — SDL events, movement, listener xform
    //   2. SimService      (priority 50)   — sim time, pause, dispatches to SimListeners
    //   3. AudioService    (priority 100)  — voice management, Steam Audio sim kick
    //   4. RenderClient    (priority 1024) — lightmaps, all render passes, bgfx::frame
    //
    // SimService and AudioService register themselves during bootstrapFinished().
    // We create the LoopMode, register our two FunctionalLoopClients, then drive
    // the loop with loopService->step().
    Darkness::LoopServicePtr loopSvc = GET_SERVICE(Darkness::LoopService);
    Darkness::SimServicePtr simSvc = GET_SERVICE(Darkness::SimService);

    {
        Darkness::LoopModeDefinition gameMode;
        gameMode.id = 1;
        gameMode.mask = LOOPMODE_INPUT | LOOPMODE_RENDER;
        gameMode.name = "GameRunning";
        loopSvc->createLoopMode(gameMode);
        loopSvc->requestLoopMode(gameMode.id);
        // Mode activation happens on the first step() call — step() now
        // processes mNewModeRequested before dispatching to clients.
    }

    // Wire continuous audio blocking updates during door animation.
    // Capture AudioServicePtr once (shared_ptr, outlives the lambda).
    {
        Darkness::AudioServicePtr audioForBlocking = GET_SERVICE(Darkness::AudioService);
        doorSystem.setAudioBlockingCallback(
            [audioForBlocking](int32_t room1, int32_t room2, float factor) {
                if (audioForBlocking)
                    audioForBlocking->setBlockingFactor(room1, room2, factor);
            });

        // Register every door's audio geometry with the acoustic scene and
        // hook the per-frame transform callback. This is the geometry-aware
        // door blocking pipeline: Steam Audio's pathing validation rejects
        // baked path edges that intersect a closed door's OBB. Done after
        // buildAcousticScene + DoorSystem::init so the scene exists and
        // every door has its world transform settled.
        if (audioForBlocking) {
            auto doorAudioGeom = doorSystem.getAudioGeometryInventory();
            audioForBlocking->registerDoorGeometry(doorAudioGeom);
            doorSystem.setAudioMeshUpdateCallback(
                [audioForBlocking](int32_t objID, const Darkness::Matrix4 &xform) {
                    audioForBlocking->setDoorTransform(objID, xform);
                });
        }
    }

    // ── Auto-bake probes if needed (with progress bar) ──
    // Intentionally placed AFTER door registration: AudioService::bakeProbes
    // reads mDoorAudioInstances to (1) classify portals as door-flanking
    // (which pushes a pair of probes ±portal-normal instead of one at the
    // centroid) and (2) reject any candidate probe whose final position
    // falls inside a door's world-AABB (the "probe-in-door-OBB" sound-
    // bypass pathology). Both checks are silent no-ops if mDoorAudioInstances
    // is empty, so reordering this block above registerDoorGeometry above
    // regresses the bake topology without any error message.
    if (state.probeBakeNeeded) {
        Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);
        runBakeWithProgressBar(window, state, audioSvc, state.probeBakePath);
        state.probeBakeNeeded = false;
    }

    // Apply initial blocking for all doors that start closed.
    // Also log every door so we can see why some don't contribute to the
    // blocking map (status != closed, soundBlocking == 0, or rooms invalid).
    // This pass runs once at startup so unconditional fprintf is OK.
    {
        Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);
        int applied = 0, rejected = 0;
        for (int32_t id : doorSystem.getAllDoorIDs()) {
            const auto *door = doorSystem.getDoor(id);
            if (!door) continue;
            const bool statusOk    = (door->status == Darkness::kDoorClosed);
            const bool blockOk     = (door->soundBlocking > 0.0f);
            const bool roomsOk     = (door->room1 >= 0 && door->room2 >= 0 &&
                                       door->room1 != door->room2);
            if (statusOk && blockOk && roomsOk) {
                audioSvc->setBlockingFactor(door->room1, door->room2,
                                             door->soundBlocking);
                ++applied;
            } else {
                std::fprintf(stderr,
                    "[DOOR_INIT_BLOCK] reject obj=%d status=%d sndBlock=%.2f "
                    "rooms=(%d,%d) reason=%s%s%s\n",
                    id, (int)door->status, door->soundBlocking,
                    door->room1, door->room2,
                    statusOk ? "" : "not-closed ",
                    blockOk  ? "" : "zero-block ",
                    roomsOk  ? "" : "bad-rooms");
                ++rejected;
            }
        }
        std::fprintf(stderr,
            "[DOOR_INIT_BLOCK] %d/%zu doors contributed initial blocking "
            "(%d rejected)\n",
            applied, doorSystem.getAllDoorIDs().size(), rejected);
    }

    // Register DoorSystem as a SimListener so it receives simStep() calls.
    // Priority 10 = before physics (which would be 20+ when registered).
    simSvc->registerListener(&doorSystem, 10);
    simSvc->registerListener(&movingTerrainSystem, 12);  // after doors (10)
    simSvc->registerListener(&pressurePlateSystem, 13);  // after moving terrain (12)
    simSvc->registerListener(&edgeTriggerSystem, 14);    // after pressure plates (13)
    simSvc->registerListener(&tweqSystem, 15);            // after edge triggers (14)
    simSvc->registerListener(&objectPushSystem, 16);      // after tweqs (15)
    simSvc->registerListener(&scriptManager, 60);         // after physics, before render

    // Hook TweqSystem completion events → ScriptManager TweqComplete messages
    tweqSystem.setEventCallback([&scriptManager](int32_t objID, Darkness::eTweqType type,
                                                  int action) {
        Darkness::ScriptMessage msg;
        msg.to = objID;
        msg.name = "TweqComplete";
        msg.from = objID;
        msg.data = Darkness::Variant(static_cast<int>(type));   // tweq type
        msg.data2 = Darkness::Variant(action);                   // direction/halt action
        scriptManager.sendMessage(msg);
    });

    // Hook MovingTerrainSystem waypoint arrival → ScriptManager MovingTerrainWaypoint.
    // StdElevator listens for this to deactivate at the destination waypoint;
    // without it, platforms loop continuously. The callback also gives us a
    // place to fan out to peer subsystems (audio schemas, particles) later.
    movingTerrainSystem.setEventCallback([&scriptManager](int32_t platformID,
                                                          int32_t waypointID,
                                                          int waypointIdx) {
        Darkness::ScriptMessage msg;
        msg.to = platformID;
        msg.name = "MovingTerrainWaypoint";
        msg.from = waypointID;  // StdElevator's "Call" path keys off msg.from
        msg.data = Darkness::Variant(waypointID);
        msg.data2 = Darkness::Variant(waypointIdx);
        scriptManager.sendMessage(msg);
    });

    // Set up door event callback: update audio blocking and log status changes.
    // When a door starts opening, remove sound blocking. When it finishes
    // closing, apply full blocking. During animation, blocking scales with
    // the open fraction (updated per-frame below).
    {
    Darkness::AudioServicePtr audioForEvents = GET_SERVICE(Darkness::AudioService);
    Darkness::PropertyServicePtr propSvcForDoors = GET_SERVICE(Darkness::PropertyService);
    Darkness::ScriptManager *scriptMgrPtr = &scriptManager;
    doorSystem.setEventCallback([audioForEvents, propSvcForDoors, scriptMgrPtr](int32_t objID, Darkness::DoorStatus status,
                                    Darkness::DoorStatus oldStatus,
                                    const Darkness::DoorState &door) {
        // Send door state message through ScriptManager for script handlers
        // (StdDoor, TrigDoorOpen, etc.)
        {
            static const char *msgNames[] = {"DoorClose", "DoorOpen", "DoorClosing", "DoorOpening", "DoorHalt"};
            if (status >= 0 && status <= 4) {
                Darkness::ScriptMessage doorMsg;
                doorMsg.to = objID;
                doorMsg.name = msgNames[status];
                doorMsg.from = objID;
                doorMsg.data = Darkness::Variant(static_cast<int>(status));
                doorMsg.data2 = Darkness::Variant(static_cast<int>(oldStatus));
                scriptMgrPtr->sendMessage(doorMsg);
            }
        }
        // State name strings matching Dark Engine schema env_tag convention
        const char *stateNames[] = {"Closed", "Open", "Closing", "Opening", "Halted"};
        const char *statusName = (status >= 0 && status <= 4) ? stateNames[status] : "?";
        const char *oldName = (oldStatus >= 0 && oldStatus <= 4) ? stateNames[oldStatus] : "?";
        AUDIO_LOG("Door %d: %s (was %s, room %d<->%d)\n",
                     objID, statusName, oldName, door.room1, door.room2);

        // Play door sound via general-purpose env_tag schema matching.
        // Tags follow the Dark Engine's StdDoor script convention:
        //   (Event StateChange) (OpenState <new>) (OldOpenState <old>)
        if (audioForEvents) {
            Darkness::SchemaTagValue eventTag;
            eventTag.tagName = "Event";
            eventTag.enumValues.push_back("StateChange");

            Darkness::SchemaTagValue openTag;
            openTag.tagName = "OpenState";
            openTag.enumValues.push_back(statusName);

            Darkness::SchemaTagValue oldTag;
            oldTag.tagName = "OldOpenState";
            oldTag.enumValues.push_back(oldName);

            Darkness::SchemaTagValue creatureTag;
            creatureTag.tagName = "CreatureType";
            creatureTag.enumValues.push_back("Player");

            // Read DoorType from the object's "ClassTags" property.
            // On-disk format: uint32 + char[252] containing tag pairs
            // like "DoorType Wood1sm". Resolved through archetype inheritance.
            std::vector<Darkness::SchemaTagValue> tags = {
                eventTag, openTag, oldTag, creatureTag};
            {
                struct { uint32_t value; char text[252]; } classTags = {};
                // The property is "Class Tag" in the pldef (label "ClassTags",
                // chunk "P$Class Tag"). Try all known names.
                bool found = false;
                for (const char *pname : {"ClassTags", "Class Tags", "Class Tag"}) {
                    if (propSvcForDoors &&
                        Darkness::getTypedProperty<decltype(classTags)>(
                            propSvcForDoors.get(), pname, objID, classTags)) {
                        found = true;
                        break;
                    }
                }
                if (found) {
                    // Parse tag pairs from the string (e.g., "DoorType Wood1sm")
                    std::string tagStr(classTags.text,
                        strnlen(classTags.text, sizeof(classTags.text)));
                    AUDIO_LOG("  Door %d ClassTags: \"%s\"\n",
                                 objID, tagStr.c_str());
                    // Split into key-value pairs
                    std::istringstream iss(tagStr);
                    std::string key, val;
                    while (iss >> key >> val) {
                        Darkness::SchemaTagValue t;
                        t.tagName = key;
                        t.enumValues.push_back(val);
                        tags.push_back(std::move(t));
                    }
                } else {
                    static int warnCount = 0;
                    if (warnCount < 3) {
                        AUDIO_LOG("  Door %d: ClassTags property not found\n", objID);
                        ++warnCount;
                    }
                }
            }
            // The door's P$Position sits at the portal plane, embedded in frame
            // geometry. Play on BOTH sides of the door (offset along the door's
            // face normal) so the sound is always audible regardless of which
            // side the player is on. Steam Audio naturally occludes the far-side
            // source through the frame geometry.
            Darkness::Matrix3 doorRot(door.baseRotation);
            Darkness::Vector3 doorNormal = doorRot[1];  // local Y = thin axis (face normal)
            Darkness::Vector3 posA = door.basePosition + doorNormal * 0.5f;
            Darkness::Vector3 posB = door.basePosition - doorNormal * 0.5f;
            auto h = audioForEvents->playEnvSchema(tags, posA, true);
            audioForEvents->playEnvSchema(tags, posB, true);
            if (h == Darkness::SOUND_HANDLE_INVALID) {
                AUDIO_LOG("  Door %d: no schema matched env_tags "
                             "(OpenState=%s OldOpenState=%s)\n",
                             objID, statusName, oldName);
            }
        }

        // Update audio blocking on door state transitions
        if (door.room1 >= 0 && door.room2 >= 0 && door.room1 != door.room2 &&
            audioForEvents) {
                if (status == Darkness::kDoorClosed) {
                    audioForEvents->setBlockingFactor(door.room1, door.room2,
                                                       door.soundBlocking);
                } else if (status == Darkness::kDoorOpen) {
                    audioForEvents->setBlockingFactor(door.room1, door.room2, 0.0f);
                }
        }
    });
    }

    // Start simulation time — SimListeners will begin receiving simStep()
    simSvc->startSim();

    // InputClient: SDL events, player movement, audio listener transform.
    // Runs first (priority 1) so camera position is current before audio/render.
    Darkness::FunctionalLoopClient inputClient(
        LOOPCLIENT_ID_INPUT, "InputClient",
        LOOPMODE_INPUT | LOOPMODE_RENDER, LOOPCLIENT_PRIORITY_INPUT,
        [&](float dt) {
            state.waterElapsed += dt;

            handleEvents(state, dbgConsole, window);

            // When unfocused, skip movement but still process events
            // (so we detect refocus). Render client handles the idle case.
            if (!state.windowFocused)
                return;

            updateMovement(dt, state, mission, dbgConsole);

            // Update frob target — cast ray from camera to find nearest frobbable object
            if (state.frobSystem) {
                state.frobSystem->update(state.cam);
            }

            // Drive any held objects toward their carry target. Run after
            // movement (so the camera position is final for this frame) but
            // before any physics step that consumes the held geom transform.
            if (state.grabSystem) {
                state.grabSystem->update(state.cam, dt);
            }

            // Update frob highlight fade (Dark Engine: ~130ms fade in/out)
            {
                constexpr float kHighlightMax = 0.10f;   // subtle white overlay; 0.47 was the original engine default
                constexpr float kFadeTime = 0.129f;       // 129ms fade
                int32_t targetID = state.frobSystem && state.frobSystem->hasTarget()
                    ? state.frobSystem->getTarget().objID : 0;

                if (targetID != state.frobHighlightObjID) {
                    // Target changed — fade out old, start fading in new
                    state.frobHighlightObjID = targetID;
                    state.frobHighlightLevel = 0.0f;
                }

                if (targetID != 0) {
                    // Fade in
                    state.frobHighlightLevel = std::min(kHighlightMax,
                        state.frobHighlightLevel + (kHighlightMax / kFadeTime) * dt);
                } else {
                    // Fade out
                    state.frobHighlightLevel = std::max(0.0f,
                        state.frobHighlightLevel - (kHighlightMax / kFadeTime) * dt);
                }
            }

            // Set audio listener position before AudioService::loopStep runs
            Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);
            audioSvc->setListenerTransform(
                Darkness::Vector3(state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]),
                state.cam.yaw, state.cam.pitch);

            if (state.physics) {
                audioSvc->setPlayerInWater(
                    state.physics->getPlayerPhysics().isInWater());
            }
        });

    // RenderClient: lightmaps, all render passes, bgfx::frame().
    // Runs last (priority 1024).
    Darkness::FunctionalLoopClient renderClient(
        LOOPCLIENT_ID_RENDERER, "RenderClient",
        LOOPMODE_RENDER, LOOPCLIENT_PRIORITY_RENDERER,
        [&](float dt) {
            // When the window loses focus (Command+Tab on macOS, Alt+Tab elsewhere),
            // skip rendering entirely. The Metal drawable is invalid while backgrounded
            // and submitting draw calls would segfault. We still call bgfx::frame() with
            // no submissions to keep the internal state ticking, and sleep briefly to
            // avoid burning CPU in a tight poll loop.
            if (!state.windowFocused) {
                bgfx::frame();
                SDL_Delay(16);  // ~60Hz idle polling rate
                return;
            }

            bool scriptDirty = lightScriptSvc.dirty;
            if (scriptDirty) lightScriptSvc.dirty = false;
            updateLightmaps(dt, meshes, mission, gpu,
                            state.objectIlluminator,
                            state.debugAnimLightmaps, state.forceFlicker,
                            scriptDirty);

            // ── Prepare frame: matrices, fog, samplers, culling ──
            auto fc = prepareFrame(state, mission);
            updateTitleBar(window, state);

            // ── View 0: Sky pass ──
            renderSky(fc, meshes, gpu, mission, state);

            // ── View 1: World geometry ──
            renderWorld(fc, meshes, gpu, mission, state);

            // ── Object meshes ──
            // Reset the dynamic-light list once per frame before drawing
            // objects. Gameplay systems (player flashlight / fire arrows /
            // mage spells / creature glow) are responsible for re-adding
            // their lights each frame; absent any caller, the list stays
            // empty and only static-light + ambient illumination apply.
            state.dynamicLights.reset();
            renderObjects(fc, gpu, mission, state);

            // ── Water surfaces ──
            renderWater(fc, meshes, gpu, state);

            // ── Debug raycast visualization (view 2) ──
            renderDebugOverlay(fc, gpu, mission, state);

            // Frob target indicator + debug console overlay.
            // Both use bgfx debug text, which requires BGFX_DEBUG_TEXT.
            // Enable it, clear the text buffer, draw frob hint, then console
            // (console draws on top if open, otherwise text buffer stays).
            bgfx::setDebug(BGFX_DEBUG_TEXT);
            bgfx::dbgTextClear();

            if (state.frobSystem && state.frobSystem->hasTarget()) {
                const auto &target = state.frobSystem->getTarget();
                // Center of screen: 160 cols (1280/8), 45 rows (720/16)
                // Place hint just below center
                int col = 80 - static_cast<int>(target.name.size()) / 2 - 2;
                if (col < 0) col = 0;
                bgfx::dbgTextPrintf(col, 24, 0x0f, "[ %s ]", target.name.c_str());
            }

            // Camera position HUD (only when show_pos is on). Row 0 so
            // it sits above the show_rooms / show_raycast lines on row 1.
            if (state.showPos) {
                bgfx::dbgTextPrintf(2, 0, 0x0F,
                    "POS — (`show_pos) cam = (%.2f, %.2f, %.2f)",
                    state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
            }

            // Room-ID label overlay (only when show_rooms is on).
            // Must happen AFTER the dbgTextClear above; otherwise the
            // labels are written and then immediately wiped.
            renderRoomLabelsOverlay(fc, state);

            // Debug console overlay (draws on top of frob hint when open)
            dbgConsole.render();

            bgfx::frame();
        });

    loopSvc->addLoopClient(&inputClient);
    loopSvc->addLoopClient(&renderClient);

    // Note: AudioService already registered itself as a LoopClient during
    // bootstrapFinished() (priority 100). It receives loopStep(dt) from
    // LoopService — the manual audioSvc->updateAudio(dt) call is removed.
    // SimService registered itself in init() (priority 50).

    // Auto-toggle all platforms if --toggle-platforms was passed
    if (cfg.togglePlatforms) {
        auto ids = movingTerrainSystem.getAllPlatformIDs();
        for (int32_t id : ids)
            movingTerrainSystem.activate(id);
        std::fprintf(stderr, "--toggle-platforms: activated %zu platforms\n", ids.size());
    }

    // Seed auto-fly probe-tour from CLI flags. Lazy activation happens on
    // the first updateMovement() tick (needs a live camera position to do
    // N-nearest probe selection, and the probe set may not be loaded yet
    // here when --skip-reflection-bake is off and the bake is still in
    // progress). Tick the auto_fly console toggle to interactively start
    // / stop without restarting.
    state.autoFly.speed             = cfg.autoFlySpeed;
    state.autoFly.waypointCount     = cfg.autoFlyWaypoints;
    state.autoFly.seed              = cfg.autoFlySeed;
    state.autoFly.pauseAtWaypointSec = cfg.autoFlyPauseSec;
    state.autoFly.enabled           = cfg.autoFly;
    if (cfg.autoFly) {
        std::fprintf(stderr,
            "--auto-fly: enabled (waypoints=%d, speed=%.1f ft/s, "
            "seed=0x%08x, pause=%.2f s) — activates on first input tick\n",
            cfg.autoFlyWaypoints, cfg.autoFlySpeed,
            cfg.autoFlySeed, cfg.autoFlyPauseSec);
    }

    // ── Main loop — LoopService drives frame dispatch ──
    //
    // --exit-after-seconds N (PLAN.AUDIO_PROFILING.md §1.4): if set, exit
    // the loop after N seconds of wall-clock from main() start. Lets
    // tools/perf_sweep.sh run unattended. The JSONL sink closes cleanly
    // through AudioService::shutdown() at program exit.
    auto mainLoopStart = std::chrono::steady_clock::now();
    bool exitAfterFired = false;
    while (state.running && !loopSvc->isTerminationRequested()) {
        loopSvc->step();
        if (cfg.exitAfterSeconds > 0.0f && !exitAfterFired) {
            float elapsed = std::chrono::duration<float>(
                std::chrono::steady_clock::now() - mainLoopStart).count();
            if (elapsed >= cfg.exitAfterSeconds) {
                std::fprintf(stderr,
                    "--exit-after-seconds: %.1f s elapsed (limit %.1f s) "
                    "— requesting clean exit\n",
                    elapsed, cfg.exitAfterSeconds);
                state.running = false;
                exitAfterFired = true;
            }
        }
    }

    // Clean up LoopClients and SimListeners before state is destroyed
    loopSvc->removeLoopClient(&inputClient);
    loopSvc->removeLoopClient(&renderClient);
    simSvc->unregisterListener(&doorSystem);
    simSvc->unregisterListener(&tweqSystem);
    simSvc->unregisterListener(&movingTerrainSystem);
    simSvc->unregisterListener(&pressurePlateSystem);
    simSvc->unregisterListener(&edgeTriggerSystem);
    simSvc->endSim();

    // Destroy runtime GPU resources not in GPUResources struct
    if (bgfx::isValid(state.acousticVBH))
        bgfx::destroy(state.acousticVBH);
    if (bgfx::isValid(state.acousticHitVBH))
        bgfx::destroy(state.acousticHitVBH);

    // Flush and close the head log before destroying physics / GPU resources.
    closeHeadLog(state);

    destroyGPUResources(gpu);
    shutdownWindow(window);

    std::fprintf(stderr, "Clean shutdown.\n");
    return 0;
}
