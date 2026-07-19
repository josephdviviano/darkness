/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *
 *    Packed C++ struct definitions matching Dark Engine property on-disk formats.
 *    Each struct corresponds to a dtype definition in scripts/thief2/t2-types.dtype.
 *    All structs use #pragma pack(push, 1) to match the exact byte layout on disk.
 *
 *    Usage:
 *      #include "TypedProperty.h"
 *      #include "DarkPropertyDefs.h"
 *      PropPosition pos;
 *      if (getTypedProperty<PropPosition>(propSvc, "Position", objID, pos)) { ... }
 *
 *    The property name passed to getTypedProperty() is the property label (not the
 *    chunk name). For example, "Position" not "P$Position".
 *
 *****************************************************************************/

#ifndef __DARKPROPERTYDEFS_H
#define __DARKPROPERTYDEFS_H

#include <cstdint>

namespace Darkness {

#pragma pack(push, 1)

// ============================================================================
// Rendering properties (Phase 0 — already used by ObjectPropParser)
// ============================================================================

// P$ModelName — dtype: ModelName, inheritor: always, 16 bytes
struct PropModelName {
    char name[16];
};

// P$Position — dtype: Position, inheritor: never (cached), 22 bytes
// Disk order: bank, pitch, heading (angvec tx, ty, tz)
// Angles are binary radians: 65536 = 360 degrees
struct PropPosition {
    float x, y, z;          // world position
    int16_t cell;            // BSP cell index
    int16_t zero;            // padding
    int16_t bank;            // rotation around X (angvec tx)
    int16_t pitch;           // rotation around Y (angvec ty)
    int16_t heading;         // rotation around Z (angvec tz)
};

// P$Scale — dtype: Scale, inheritor: always (but has kPropertyNoInherit flag), 12 bytes
struct PropScale {
    float x, y, z;
};

// P$RenderAlpha — dtype: RenderAlp, inheritor: always, 4 bytes
struct PropRenderAlpha {
    float alpha;             // 0.0 = invisible, 1.0 = opaque
};

// P$RenderType — dtype: RenderTyp, inheritor: always, 4 bytes
// Values: 0=Normal, 1=NotRendered, 2=NoLightmap, 3=EditorOnly
struct PropRenderType {
    uint32_t mode;
};

// P$SelfIllum — dtype: SelfIllum, inheritor: always, 4 bytes
struct PropSelfIllum {
    float brightness;
};

// P$SelfLit — dtype: SelfLit, inheritor: always, 4 bytes.
// Schema label "DynamicLight". Flags an object as a true per-frame dynamic
// light source: its contribution is NOT baked into the lightmap, instead
// added to the DynamicLightList every render frame. Brightness is in the
// same raw light units as static-light brightness (0..1023 range
// historically; reference engine treats the int32 as the radiance scalar
// passed straight into add_dynamic_light). The light position is the
// object's current world position; if P$LightColor is also set on the
// object the dynamic light is tinted with that hue/sat, otherwise white.
// Radius is a fixed engine constant (hardware-render value 10.0), NOT
// stored in the property.
struct PropSelfLit {
    int32_t brightness;
};

// P$ExtraLigh — dtype: ExtraLigh, inheritor: always, 8 bytes
// Per-object lighting modifier. `factor` is in [-1, 1]; when `isAdditive` is
// nonzero, factor is added to the ambient term (clamped so factor + frob
// highlight stays ≤ 1.0). When zero, factor overrides the ambient term and
// also forces ambient-only mode (suppresses contribution from cell lights).
struct PropExtraLight {
    float    factor;
    uint32_t isAdditive;     // BOOL32: nonzero = additive, zero = override
};
static_assert(sizeof(PropExtraLight) == 8);

// ============================================================================
// Light properties
// ============================================================================

// P$Light — dtype: Light, inheritor: always, 28 bytes
// brightness(4) + offset(12) + radius(4) + quadlight(4) + inner(4)
struct PropLight {
    float brightness;
    float offsetX, offsetY, offsetZ;
    float radius;
    uint32_t quadLight;      // bool32
    float innerRadius;
};

// P$LightColor — dtype: LightColo, inheritor: always, 8 bytes
struct PropLightColor {
    float hue;
    float saturation;
};

// P$AnimLight — dtype: AnimLight, inheritor: archetype, 76 bytes
// Sole on-disk representation used by all consumers (LightingSystem,
// DarknessHeadless prop-dump). Read via getTypedProperty<PropAnimLight>(...)
// so layout drift can't desync parsers reading the same bytes.
struct PropAnimLight {
    int32_t unk1;
    float offsetX, offsetY, offsetZ;
    int32_t unk2;            // default 1
    int16_t cellIndex;
    int16_t hitCells;
    int16_t lightNum;        // default -1
    uint16_t mode;           // animlightmode enum (0-9), default 4
    int32_t brightenTime;
    int32_t dimTime;
    float minBrightness;
    float maxBrightness;
    int32_t unk4;
    uint32_t rising;         // bool32
    int32_t countdown;
    uint32_t inactive;       // bool32
    float radius;
    int32_t unk5;
    uint32_t quadLit;        // bool32
    float innerRadius;
};
static_assert(sizeof(PropAnimLight) == 76,
    "PropAnimLight must match the 76-byte on-disk layout exactly — "
    "every consumer memcpy's directly from PropertyService raw bytes");

// ============================================================================
// Physics properties (Phase 2)
// ============================================================================

// P$PhysType — dtype: PhysType, inheritor: always, 16 bytes
// physmodeltype: 0=OBB, 1=Sphere, 2=SphereHat, 3=None
struct PropPhysType {
    uint32_t type;
    int32_t submodels;       // default 6
    uint32_t removeOnSleep;  // bool32
    uint32_t special;        // bool32
};

// P$PhysState — dtype: PhysState, inheritor: always, 48 bytes
struct PropPhysState {
    float locX, locY, locZ;
    float facX, facY, facZ;
    float velX, velY, velZ;
    float rotVelX, rotVelY, rotVelZ;
};

// P$PhysDims — dtype: PhysDims, inheritor: always, 52 bytes
struct PropPhysDims {
    float radius[2];
    float offset1X, offset1Y, offset1Z;
    float offset2X, offset2Y, offset2Z;
    float sizeX, sizeY, sizeZ;
    uint32_t pointTerrain;   // bool32
    uint32_t pointSpecial;   // bool32
};

// P$PhysControl — dtype: PhysContr, inheritor: always, 40 bytes
struct PropPhysControl {
    uint32_t flags;          // physcontrolflags bitfield
    float transX, transY, transZ;
    float velX, velY, velZ;
    float rotX, rotY, rotZ;
};

// P$PhysAttr — dtype: PhysAttr, inheritor: always, 52 bytes
struct PropPhysAttr {
    float gravity;
    float mass;
    float density;
    float elasticity;
    float friction;
    float cogX, cogY, cogZ;  // center of gravity offset
    uint32_t rotationAxes;   // physaxisflags bitfield
    uint32_t restAxes;       // physorientationflags
    uint32_t climbable;      // physorientationflags
    uint32_t edgeTrigger;    // bool32
    float poreSize;
};

// ============================================================================
// Door properties (Phase 2)
// ============================================================================

// P$RotDoor — dtype: RotDoor, inheritor: always
// int32 zero + 3*float + int32 axis + uint32 state + bool32 + float blocksound +
// bool32 blockvision + float pushmass + 3*vector + shortvec + float + int32[2] +
// bool32 clockwise + 2*shortvec
struct PropRotDoor {
    int32_t zero;
    float closedAngle;
    float openAngle;
    float speed;
    int32_t axis;            // physaxistype: 0=X, 1=Y, 2=Z
    uint32_t status;         // doorstate: 0=closed, 1=open, 2=closing, 3=opening, 4=halted
    uint32_t hardLimits;     // bool32
    float blockSound;        // sound blocking factor when closed
    uint32_t blockVision;    // bool32 — blocks AI vision when closed
    float pushMass;
    float closedPosX, closedPosY, closedPosZ;
    float openPosX, openPosY, openPosZ;
    float startPosX, startPosY, startPosZ;
    int16_t startFacBank, startFacPitch, startFacHeading;
    float unknown;
    int32_t room1, room2;
    uint32_t clockwise;      // bool32
    int16_t haltCloseBank, haltClosePitch, haltCloseHeading;
    int16_t haltOpenBank, haltOpenPitch, haltOpenHeading;
};

// P$TransDoor — dtype: TransDoor, inheritor: always
struct PropTransDoor {
    int32_t zero;
    float closed;
    float open;
    float speed;
    int32_t axis;            // physaxistype
    uint32_t status;         // doorstate
    uint32_t hardLimits;     // bool32
    float blockSound;
    uint32_t blockVision;    // bool32
    float pushMass;
    float closedPosX, closedPosY, closedPosZ;
    float openPosX, openPosY, openPosZ;
    float startPosX, startPosY, startPosZ;
    int16_t startFacBank, startFacPitch, startFacHeading;
    float unknown;
    int32_t room1, room2;
};

// ============================================================================
// AI properties (Phase 4)
// ============================================================================

// P$AI — dtype: AI, inheritor: always, 32 bytes
struct PropAI {
    char behaviorSet[32];
};

// P$AI_Alertness — dtype: AI_Alertn, inheritor: always, 8 bytes
struct PropAIAlertness {
    uint32_t level;
    uint32_t peak;
};

// ============================================================================
// Script properties
// ============================================================================

// P$Scripts — dtype: Scripts, inheritor: always, 132 bytes
struct PropScripts {
    char script0[32];
    char script1[32];
    char script2[32];
    char script3[32];
    uint32_t dontInherit;    // bool32
};

// P$TrapFlags — dtype: TrapFlags, inheritor: always, 4 bytes
// Bits: 1=Once, 2=Invert, 4=NoOn, 8=NoOff
struct PropTrapFlags {
    uint32_t flags;
};

// ============================================================================
// Audio properties (Phase 3)
// ============================================================================

// P$SchemaPlayParams — dtype: SchPlayPa, inheritor: always, 20 bytes
struct PropSchemaPlayParams {
    uint16_t flags;          // schemaplayflags bitfield
    uint16_t audioClass;     // schemaaudioclass enum
    int32_t volume;          // default -1
    int32_t pan;
    uint32_t delay;
    int32_t fade;
};

// P$SchemaLoopParams — dtype: SchLoopPa, inheritor: always, 8 bytes
struct PropSchemaLoopParams {
    uint8_t flags;           // schemaloopflags: 1=Poly, 2=AutoHalt
    uint8_t maxSamples;
    int16_t loopCount;
    int16_t minInterval;
    int16_t maxInterval;
};

// P$SchemaAttenuationFactor — dtype: SchAttFac, 4 bytes.
// Per-schema attenuation-factor divisor (default 1.0). At runtime, the
// volume formula is:
//   volume_centibels = gain - (d/r) * (5000 + gain) / attenuation
// So values > 1.0 make the schema fall off LESS aggressively (volume
// drops to e.g. -25 dB at radius instead of -50 dB for attenuation=2.0).
// Set per-archetype on the schema's object record in dark.gam.
struct PropSchAttFac {
    float attenuation;
};

// P$SchemaPriority — dtype: SchPriori, 4 bytes.
// Per-schema priority value used by the voice manager to decide which
// sound to drop when the active-voice budget is exceeded. Range 0–255
// per original engine; some schemas (e.g. M06 ambient archetype) use
// 255 to stay above ordinary sounds.
struct PropSchPriori {
    int32_t priority;
};

// P$SchemaMessage — dtype: SchMsg, 16 bytes.
// Optional message label fired to scripts when the schema is triggered.
// E.g. AI alerts use this to announce "MetalImpact" when an arrow hits
// armor. Null-terminated char[16].
struct PropSchMsg {
    char label[16];
};

// P$SchemaLastSample — dtype: SchLastSa, 4 bytes. Runtime-mutable: index
// of the last sample played from this schema. Used by SCH_NO_REPEAT to
// avoid replaying the same sample twice in a row. Not authored — written
// at runtime as the schema plays.
struct PropSchLastSa {
    int32_t value;
};

// P$SpotlightAndAmbient — dtype: SpotAmb, 12 bytes. RENDERER property.
//
// Despite the audio-sounding "Ambient" name and the field names below, this
// is a *lighting* property in the original engine — the spotlight-and-ambient
// entry in the renderer's light-property table, paired with P$Light + P$Spotlight
// to encode a spotlight that ALSO emits an omnidirectional ambient term.
// The field names were lifted from an OPDE-era dtype guess that confused
// this with sound data; semantically it is the same 3-float vector the
// engine reads as a spotlight cone with an ambient brightness override.
//
// Field meanings (per the original engine's CalcSpotlightVec consumer):
//   • innerAngleDeg     — inner cone half-angle (degrees); fully lit inside
//   • outerAngleDeg     — outer cone half-angle (degrees); dark outside
//   • ambientBrightness — overrides the base light's brightness for the
//                         omnidirectional (ambient) component when this
//                         spotlight is the "SpotlightAndAmbient" variant
//
// TODO(lighting): SpotlightAndAmbient is not yet implemented on the renderer
// side. The struct sits here as a stub so callers parse the bytes correctly
// once the feature lands. See TASKS.TODO.md "SpotlightAndAmbient lighting".
struct PropSpotlightAndAmbient {
    float innerAngleDeg;
    float outerAngleDeg;
    float ambientBrightness;
};

// P$Heartbeat — dtype: Heartbeat, 4 bytes.
// Heartbeat sound timing — int32 milliseconds between beats. Set on
// objects (typically the player or AIs) to drive a periodic "heartbeat"
// schema (e.g. health-low warning).
struct PropHeartbeat {
    int32_t time;
};

// P$MaxSpeechPause — dtype: MaxSpchPa, 4 bytes.
// Per-AI maximum pause between speech utterances (ms).
struct PropMaxSpchPa {
    int32_t time;
};

// P$MinSpeechPause — dtype: MinSpchPa, 4 bytes.
// Per-AI minimum pause between speech utterances (ms).
struct PropMinSpchPa {
    int32_t time;
};

// P$AIHearing — dtype: aiaptitude (aliased), 4 bytes.
// Per-AI hearing sensitivity rating (default 3 = "Normal"). Values
// 1–5 = VeryLow → VeryHigh. Drives AIHearStat dist_muls / db_adds.
struct PropAI_Hearin {
    uint32_t rating;
};

// P$AISoundType — dtype: AI_SndTyp, 40 bytes.
// Maps a schema (or sound class) to an AI-side sound type for awareness
// propagation. `type` is an `aisoundtype` enum, `signal` is the script
// signal name fired when this sound is heard.
struct PropAI_SndTyp {
    uint32_t type;
    char     signal[32];
    int32_t  zero;
};

// P$VoiceIndex — dtype: VoiceIdx, 4 bytes.
// Per-object voice-list index. Drives speech selection: each AI archetype
// points at a slot in the global voice list (Speech_DB) so the same
// concept ("Greet", "Alert") plays a different sample for different
// guards.
struct PropVoiceIdx {
    int32_t voice;
};

// P$SpeechVoice — dtype: SpchVoice, 16 bytes.
// String name of the voice this AI uses. Resolved to a Speech_DB voice
// slot at runtime. Null-terminated char[16].
struct PropSpchVoice {
    char label[16];
};

// ============================================================================
// Gameplay properties
// ============================================================================

// P$HitPoints — dtype: HitPoints, inheritor: always, 4 bytes
struct PropHitPoints {
    int32_t hp;
};

// P$MAX_HP — dtype: MAX_HP, inheritor: always, 4 bytes
struct PropMaxHP {
    int32_t hp;
};

// P$StackCount — dtype: StackCoun, inheritor: always, 4 bytes
struct PropStackCount {
    int32_t count;
};

// P$CombineType — dtype: CombineTy, inheritor: always, 16 bytes
struct PropCombineType {
    char label[16];
};

// P$MovingTerrain — dtype: MovingTer, inheritor: always, 8 bytes
struct PropMovingTerrain {
    uint32_t active;         // bool32
    int32_t unknown;
};

// P$Locked — dtype: Locked, inheritor: always, 4 bytes
// Simple boolean lock state. When true, doors reject frob attempts while closed.
struct PropLocked {
    uint32_t isLocked;       // bool32: 0 = unlocked, nonzero = locked
};

// ============================================================================
// Frob interaction properties (Phase 4G)
// ============================================================================

// P$FrobInfo — dtype: FrobInfo, inheritor: always, 16 bytes
// 3 action fields + padding. Each field is a bitfield of frob actions.
struct PropFrobInfo {
    uint32_t worldAction;     // actions when frobbed in world
    uint32_t invAction;       // actions when frobbed in inventory
    uint32_t toolAction;      // actions when used as tool
    uint32_t pad;             // zero padding
};

// ============================================================================
// Pressure plate property (Phase 4F)
// ============================================================================
//
// P$PhysPPlat — dtype: PhysPPlat, 28 bytes.
// Defines a pressure plate that depresses under weight and sends script
// messages. The last two int32 fields are runtime state (state + cur_pause).

#pragma pack(push, 1)
struct PropPhysPPlate {
    float    weight;        // Activation weight threshold
    float    travel;        // Distance plate moves down (world units)
    float    speed;         // Movement speed (units/sec)
    float    pause;         // Hold time: -1=immediate, -2=latched, >0=seconds
    uint32_t blockVision;   // bool32: blocks AI vision when active
    int32_t  state;         // Runtime: ePPState (0=inactive, 1=active, 2=deactivating, 3=activating)
    float    curPause;      // Runtime: current pause countdown
};
#pragma pack(pop)
static_assert(sizeof(PropPhysPPlate) == 28, "PropPhysPPlate must match PhysPPlat dtype (28 bytes)");

// ============================================================================
// TPath link data (Moving Terrain / Elevators)
// ============================================================================
//
// TPath links connect waypoints in a chain. Each link carries movement
// parameters for the segment from source waypoint to destination waypoint.
// The dtype definition is in t2-types.dtype as struct TPath (16 bytes).

#pragma pack(push, 1)
struct PropTPath {
    float    speed;       // Movement speed along this segment (world units/sec)
    int32_t  pausetime;   // Pause duration at destination waypoint (milliseconds)
    uint32_t pathlimit;   // bool32: enforce hard position limit at waypoint
    int32_t  zero;        // Padding/reserved
};
#pragma pack(pop)
static_assert(sizeof(PropTPath) == 16, "PropTPath must match TPath dtype (16 bytes)");

// ============================================================================
// Tweq properties (Phase 5 — procedural object animation)
// ============================================================================
//
// Tweqs animate objects at runtime: rotating fans, flickering lights, scaling
// objects, cycling models. Config (Cfg) properties hold static parameters set
// by the level designer; state (St) properties hold per-instance runtime data.
//
// Flag values match the Dark Engine's tweqflgs.h definitions exactly.
// Struct layouts verified against t2-types.dtype and p_ver size fields.

// ── Tweq flag types ──

// Animation config flags (uint8 bitfield — CfgTweq*.anim)
enum TweqAnimFlags : uint8_t {
    kTweqAnimNoLimit        = 0x01,  // Ignore axis bounds
    kTweqAnimSim            = 0x02,  // Auto-start with simulation
    kTweqAnimWrap           = 0x04,  // Wrap at bounds (vs bounce)
    kTweqAnimOneBounce      = 0x08,  // Stop after one forward+reverse cycle
    kTweqAnimSimSmallRadius = 0x10,  // Only update within 20 units of camera
    kTweqAnimSimLargeRadius = 0x20,  // Only update within 80 units of camera
    kTweqAnimOffscreen      = 0x40,  // Only update when off-screen
};

// Curve/interpolation flags (uint8 bitfield — CfgTweq*.curve)
enum TweqCurveFlags : uint8_t {
    kTweqCurveJitterLow  = 0x01,  // Low jitter (level 1)
    kTweqCurveJitterHigh = 0x02,  // High jitter (level 2); low+high = level 3
    kTweqCurveMul        = 0x04,  // Multiply rate instead of add
};
constexpr uint8_t kTweqCurveJitterMask = 0x03;  // Bits 0-1: jitter level 0-3

// Halt action (uint8 enum — CfgTweq*.halt)
enum TweqHaltAction : uint8_t {
    kTweqHaltDestroy    = 0,  // Destroy the object
    kTweqHaltRemoveProp = 1,  // Remove the tweq property, keep object
    kTweqHaltStop       = 2,  // Stop the tweq, leave property attached
    kTweqHaltContinue   = 3,  // Continue forever (wrapping/bouncing)
    kTweqHaltSlay       = 4,  // Slay (damage-kill) the object
};

// Misc config flags (uint16 bitfield — CfgTweq*.misc)
enum TweqMiscFlags : uint16_t {
    kTweqMiscAnchor     = 0x001,  // Anchor at bottom of model
    kTweqMiscScripts    = 0x002,  // Send TweqComplete messages to scripts
    kTweqMiscRandom     = 0x004,  // Random variation
    kTweqMiscGravity    = 0x008,  // Apply gravity (emitters)
    kTweqMiscZeroVel    = 0x010,  // Zero velocity on emit
    kTweqMiscTellAI     = 0x020,  // Notify AI system
    kTweqMiscPushOut    = 0x040,  // Push emitted objects out of source
    kTweqMiscNegLogic   = 0x080,  // Invert on/off logic
    kTweqMiscRelVel     = 0x100,  // Emit velocity relative to object rotation
    kTweqMiscNoPhysics  = 0x200,  // Skip physics on emitted objects
    kTweqMiscVHot       = 0x400,  // Anchor at VHOT point
    kTweqMiscHostOnly   = 0x800,  // Run only on network host
};

// Animation state flags (uint16 base / uint32 per-axis — StTweq*.anim)
enum TweqStateFlags : uint16_t {
    kTweqStateOn      = 0x01,  // Tweq is currently active
    kTweqStateReverse = 0x02,  // Playing in reverse direction
    kTweqStateReSynch = 0x04,  // Resynchronize on next update
    kTweqStateGoEdge  = 0x08,  // Jump to edge when resynching
    kTweqStateLapOne  = 0x10,  // First lap completed (internal)
};

// Tweq type enum (matches Dark Engine eTweqType)
enum eTweqType : int32_t {
    kTweqTypeScale   = 0,
    kTweqTypeRotate  = 1,
    kTweqTypeJoints  = 2,
    kTweqTypeModels  = 3,
    kTweqTypeDelete  = 4,
    kTweqTypeEmitter = 5,
    kTweqTypeFlicker = 6,
    kTweqTypeLock    = 7,
    kTweqTypeAll     = 8,  // Sentinel: operate on all tweqs
};

// ── Per-axis configuration (shared by Rotate and Scale) ──

struct PropTweqAxisConfig {
    float rate;   // degrees/sec (rotate) or scale-units/sec (scale)
    float low;    // lower bound
    float high;   // upper bound
};
static_assert(sizeof(PropTweqAxisConfig) == 12);

// ── CfgTweqRo / CfgTweqSc — vector tweq config (Rotate, Scale) ──
// p_ver 2.48 → 48 bytes

struct PropCfgTweqVector {
    uint8_t  unknown;   // padding/version (always 0)
    uint8_t  curve;     // TweqCurveFlags
    uint8_t  anim;      // TweqAnimFlags
    uint8_t  halt;      // TweqHaltAction
    uint16_t misc;      // TweqMiscFlags
    uint16_t zero;      // unused for axis tweqs (per-axis rates instead)
    PropTweqAxisConfig x;
    PropTweqAxisConfig y;
    PropTweqAxisConfig z;
    int32_t  primary;   // physaxistype: 0=X, 1=Y, 2=Z (controls completion)
};
static_assert(sizeof(PropCfgTweqVector) == 48);

// ── CfgTweqMo — models tweq config ──
// p_ver 2.104 → 104 bytes

struct PropCfgTweqModels {
    uint8_t  unknown;
    uint8_t  curve;
    uint8_t  anim;
    uint8_t  halt;
    uint16_t misc;
    uint16_t rate;      // milliseconds per model swap
    char     modelName[6][16];  // up to 6 model names (null-terminated)
};
static_assert(sizeof(PropCfgTweqModels) == 104);

// ── CfgTweqBl / CfgTweqDe — simple tweq config (Flicker, Delete) ──
// p_ver 2.8 → 8 bytes

struct PropCfgTweqSimple {
    uint8_t  unknown;
    uint8_t  curve;
    uint8_t  anim;
    uint8_t  halt;
    uint16_t misc;
    uint16_t rate;      // milliseconds per step
};
static_assert(sizeof(PropCfgTweqSimple) == 8);

// ── StTweqRot / StTweqSca — vector tweq state ──
// p_ver 2.16 → 16 bytes

struct PropStTweqVector {
    uint16_t anim;      // TweqStateFlags (base: global on/off/reverse)
    uint16_t misc;      // unused (tweqnullflags)
    uint32_t x;         // TweqStateFlags per-axis (x)
    uint32_t y;         // TweqStateFlags per-axis (y)
    uint32_t z;         // TweqStateFlags per-axis (z)
};
static_assert(sizeof(PropStTweqVector) == 16);

// ── StTweqBli / StTweqDel / StTweqMod — simple tweq state ──
// p_ver 2.8 → 8 bytes

struct PropStTweqSimple {
    uint16_t anim;      // TweqStateFlags
    uint16_t misc;      // unused
    uint16_t time;      // elapsed time in ms
    uint16_t frame;     // current frame index
};
static_assert(sizeof(PropStTweqSimple) == 8);

#pragma pack(pop)

} // namespace Darkness

#endif // __DARKPROPERTYDEFS_H
