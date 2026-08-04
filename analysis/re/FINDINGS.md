# Dark Engine — binary-derived findings

Curated knowledge recovered by analyzing executables we own. **This file is tracked**;
everything in it must carry its evidence and a confidence grade. Raw dumps, Ghidra
projects and working files stay out of the repo.

Provenance policy, tier definitions and the `[BIN:]` tag rule:
`.claude/PLAN.BINARY_RE_PIPELINE.md` and `.claude/CLAUDE.md` → Reference Implementations.

Confidence grades: `CONFIRMED` (observed and independently cross-checked) / `HIGH` /
`MEDIUM` / `LOW` / `UNVERIFIED`.

**Rules for this file.** One address, one entry — reconcile on write, never append a
contradictory second entry (the failure mode of the third-party knowledge base this
pipeline is modelled on). Every section ends with a `Suggested verification` block. When a
later pass overturns an earlier claim, correct it *in place* and record the correction
below it — do not silently edit.

## Binaries analyzed

| Tag | Binary | Size | Image base | Notes |
|---|---|---|---|---|
| `ND128` | `../newdark_v128/new_dark/Thief2.exe` | 5,009,920 | `0x400000` | NewDark 1.28. Primary target. |
| `ND128-ED` | `../newdark_v128/editor/DromEd.exe` | 7,435,776 | — | Editor. Not yet analyzed. |
| `T2-107` | `THIEF2.EXE` (retail CD) | 2,658,304 | `0x400000` | v1.07, 2000-08-24. Original-behaviour reference. |

Both are `PE32 / IMAGE_FILE_MACHINE_I386`. Section layout is identical in both and includes
the engine's two custom allocator sections: `.text, lgalloc, .rdata, .data, lgalloc_,
.rsrc` (`ND128` adds `.reloc`).

---

## 1. Import tables — the audio backend is NOT statically linked

**Grade: CONFIRMED.** Extracted with `analysis/re/bootstrap_imports.py` (pefile) from both
binaries.

| | `T2-107` retail | `ND128` |
|---|---|---|
| imported modules | 6 | 9 |
| imported symbols | 224 | 369 |
| modules | ADVAPI32, GDI32, KERNEL32, ole32, USER32, WINMM | + MSVCP90, MSVCR90, SHELL32 |

### 1.1 Negative result: no audio API is imported by either binary

**Neither binary imports `dsound.dll`, `openal32.dll`, `soft_oal.dll`, `eax.dll`,
`dinput*.dll`, or `dxguid.dll`.** The only audio-adjacent import in either is `WINMM.dll`.

This settles an open question and is worth recording precisely so nobody re-runs it: **the
audio output backend is resolved dynamically at runtime (`LoadLibrary`), not bound at link
time.** Consequently the import table cannot tell us which audio stack the engine uses, and
identifying it requires string + xref analysis instead.

This is consistent with NewDark's documented behaviour — `new_config_vars.txt` exposes
OpenAL device selection and a `disable_oal_eax_reverb` switch that chooses EAX reverb vs
AL EFX reverb *at runtime*, which is only possible with a dynamically-loaded backend.

### 1.2 `WINMM` usage differs sharply between the two versions

`T2-107` imports **14** WINMM symbols; `ND128` imports **7**. The delta is entirely the
`mmio*` family:

| Symbol | `T2-107` | `ND128` | Purpose |
|---|---|---|---|
| `mmioOpenA`, `mmioRead`, `mmioSeek`, `mmioClose`, `mmioDescend`, `mmioAscend`, `mmioGetInfo` | yes | **no** | Windows multimedia file I/O — the standard RIFF/WAV chunk reader |
| `mciSendCommandA` | yes | yes | MCI (redbook/CD audio) |
| `timeSetEvent`, `timeKillEvent`, `timeBeginPeriod`, `timeEndPeriod`, `timeGetDevCaps`, `timeGetTime` | yes | yes | multimedia timer + high-resolution clock |

**Interpretation (grade: HIGH).** Retail reads WAV assets through the Win32 `mmio` RIFF
parser. NewDark dropped it entirely and supplies its own sample loading — expected, since
NewDark adds codec support beyond PCM WAV and ships `ffmpeg.dll` / `lgvid.dll` in
`contrib/`. For us this means **retail's on-disk sample handling is plain RIFF/WAV via the
OS parser** — no engine-specific container.

**Interpretation (grade: MEDIUM).** The retained `timeSetEvent`/`timeKillEvent` pair is a
periodic multimedia-timer callback. In an engine of this vintage that is the usual home for
a sound-scheduler or sim tick. Not yet localized to a function.

### 1.3 CRT linkage

`T2-107` statically links the CRT (no MSVC runtime imports). `ND128` imports `MSVCP90.dll`
+ `MSVCR90.dll` (165 + 11 symbols) and ships `contrib/Microsoft.VC90.CRT/`. Practical
consequence for RE: in `ND128` the CRT is *not* inlined into the image, so CRT-signature
noise is absent and library-function identification is cheaper — a point in favour of
`ND128` as the primary target.

### Suggested verification

1. Confirm the dynamic-load hypothesis directly: search both binaries' string tables for
   `dsound.dll`, `OpenAL32.dll`, `soft_oal.dll`, `eax.dll`, and xref each hit to the
   `LoadLibrary`/`GetProcAddress` site that consumes it. Expect a resolver table.
2. Xref `timeSetEvent` in both binaries to find the callback registration, then follow the
   callback to identify what the multimedia timer actually drives.
3. Repeat the import extraction on `ND128-ED` (`DromEd.exe`) — the editor may link things
   the game loads dynamically.
4. Repeat on the Thief 1 / Gold retail binaries for a T1-vs-T2 differential.

---

## 2. Audio backend and the acoustics model (`ND128`)

**Grade: CONFIRMED** for the string/table content below; **HIGH** for the interpretation.
All from the SQLite index (`analysis/re/export_index.py` → `query.py`): 16,996 functions,
134,302 symbols, 11,618 strings, 398,722 xrefs.

### 2.1 The dynamic-load hypothesis from §1.1 is confirmed

`dsound.dll` (`0x78BF90`) and `openal32.dll` (`0x78C33C`) exist as **strings, not imports** —
so both backends are resolved at runtime. Supporting evidence, all as strings:

- EAX capability probes: `EAX2.0`, `EAX3.0`, `EAX4.0`, `EAX5.0`, `EAX-RAM` (`0x78CB08`+),
  reported via `EAX2 : %d` … `EAX5 : %d` and `occlusion : %d`.
- OpenAL EAX *extension* entry points resolved by name: `EAXSetBufferMode`,
  `EAXGetBufferMode`, `AL_EAX_RAM_SIZE`, `AL_EAX_RAM_FREE` (`0x78C02C`+).
- Runtime capability banners: `EAX reverb support enabled` / `EFX reverb support enabled` /
  `occlusion support enabled` (`0x78CED8`+).
- Config-var names present as strings, matching `new_config_vars.txt` exactly:
  `sfx_source_reverb_mix`, `sfx_eax`, `disable_oal_eax_reverb`, `submerged_sound_occlusion`,
  `force_underwater_reverb`, `snd_oal_full_enum`, `snd_oal_2d_channels`,
  `skip_dsound_check`, `fail_dsound_check`.
- `al_reverb.ini` (`0x78BF9C`) plus its two failure messages — `WARNING: failed to load
  al_reverb.ini` and `WARNING: parse error reading al_reverb.ini (for reverb preset #%d)`.
  This ties the shipped doc file directly to a runtime load path, and confirms the file is
  parsed **per preset index**.

### 2.2 Two PARALLEL name tables over one 0–25 index space

**Grade: CONFIRMED** — read directly out of `.data` as two pointer arrays, not inferred
from string adjacency.

> **Correction (supersedes the first pass).** An earlier version of this section derived
> both tables by scanning the *string pool* around `0x76E730`–`0x76E8E4` and got the room
> types' indices wrong (it started them at `Small Dead` = 0) while missing three named
> entries entirely (`Large Dead`, `Small Live`, `Sewers`), which live at lower addresses
> outside the sampled range. String-pool order is not array order. The arrays below are
> the authority.

Two `const char*` arrays sit **adjacent and contiguous** in `.data`, each exactly **26
entries**, covering the same index space 0–25:

| array | span | idx 0 | idx 25 |
|---|---|---|---|
| EAX preset names | `0x7FAA48`..`0x7FAAAC` | `Generic` | `Psychotic` |
| Thief room-type names | `0x7FAAB0`..`0x7FAB14` | `Generic` | `Type 25` |

[BIN: pointer arrays dumped at 0x7FAA48 / 0x7FAAB0, Thief2.exe NewDark 1.28]

| idx | EAX preset | Thief room type | idx | EAX preset | Thief room type |
|---|---|---|---|---|---|
| 0 | Generic | Generic | 13 | StoneCorridor | **Live Hallway** |
| 1 | PaddedCell | **Small Dead** | 14 | Alley | **Tunnels** |
| 2 | Room | **Small Normal** | 15 | Forest | **Outside** |
| 3 | Bathroom | Type 3 | 16 | City | Type 16 |
| 4 | LivingRoom | Type 4 | 17 | Mountains | Type 17 |
| 5 | StoneRoom | **Large Normal** | 18 | Quarry | Type 18 |
| 6 | Auditorium | Type 6 | 19 | Plain | **Large Dead** |
| 7 | ConcertHall | Type 7 | 20 | ParkingLot | **Small Live** |
| 8 | Cave | **Large Live** | 21 | SewerPipe | **Sewers** |
| 9 | Arena | **Caverns** | **22** | **UnderWater** | Type 22 |
| 10 | Hangar | Type 10 | 23 | Drugged | Type 23 |
| 11 | CarpetedHallway | **Dead Hallway** | 24 | Dizzy | Type 24 |
| 12 | Hallway | **Normal Hallway** | 25 | Psychotic | Type 25 |

**The two arrays are the same list, labelled twice.** `Type N` is simply "this EAX slot has
no Thief-facing name". The semantic correspondence at every named index is too exact to be
coincidence:

- `Small Dead` → **PaddedCell** (the deadest small space in the EAX set)
- `Small Normal` → **Room**; `Large Normal` → **StoneRoom**; `Large Live` → **Cave**
- `Dead / Normal / Live Hallway` → **CarpetedHallway / Hallway / StoneCorridor** — the
  damping order matches exactly, in order
- `Sewers` → **SewerPipe**; `Outside` → **Forest**; `Caverns` → **Arena**

So **a Thief room's acoustic type IS an EAX preset index**, presented to the level designer
under a friendlier name. That is directly usable: `room type → index → the parameter row at
that position in `../newdark_v128/new_dark/doc/al_reverb.ini`` gives the full reverb
definition (decay, reflections, HF damping, room size…) for every acoustic environment the
original levels are authored in.

The named set is a clean **size × damping** matrix plus a hallway triad plus special
spaces — `{Small, Large} × {Dead, Normal, Live}`, `{Dead, Normal, Live} Hallway`, and
`Caverns / Tunnels / Sewers / Outside / Generic`.

**Cross-validation:** `new_config_vars.txt` documents `force_underwater_reverb` as
defaulting to preset **22**, and the array independently places `UnderWater` at index 22.
Documentation and binary agree.

### 2.3 Acoustics is a two-level file-var property — descriptor recovered

**Grade: CONFIRMED.** The descriptor records were located by scanning `.data` for pointers
to the label strings (`analysis/re/find_refs.py`), since Ghidra had not typed them as
pointers and `query.py --xref` therefore reported nothing.

Two records, **stride 0x2C (44 bytes)**, each laid out as an inline variable name followed
by a label pointer and a type-name pointer:

| field | `0x7FA9F0` | `0x7FAA1C` |
|---|---|---|
| inline var name | **`MissionEAX`** | **`GameSysEAX`** |
| label (`+0x0C`) | `"Mission Default EAX Value"` | `"GameSys Default EAX Value"` |
| type name (`+0x10`) | `"sAcousticsProperty"` | `"sAcousticsProperty"` |
| trailing ints | `1, 0, 1, 0, 0, 1` | `1, 0, 1, 0, 0` |

[BIN: descriptor records dumped at 0x7FA9F0 / 0x7FAA1C, Thief2.exe NewDark 1.28]

RTTI confirms two `cFileVar` instances over the same payload struct —
`cFileVar<sAcousticsProperty, &gGameSysAcoustics>` and
`cFileVar<sAcousticsProperty, &gMissionAcoustics>` — with `cGameAcoustics` /
`cMissAcoustics` as the consumers.
[BIN: RTTI strings @0x8533A0 / @0x853300, Thief2.exe NewDark 1.28]

A neighbouring record at `0x7FA9BC` has inline name `Acoustics`, type tag `4`, a storage
pointer `0x00A2AA34`, and label pointers `"Room"` / `"Acoustics"` — i.e. the per-room
acoustics property, distinct from the two global defaults. `0x00A2AA34` is a candidate
acoustics storage global (grade: MEDIUM, not yet verified).

**Interpretation (HIGH):** the reverb environment resolves through **three** levels —
gamesys default (`GameSysEAX`) → mission default (`MissionEAX`) → per-room value. A
faithful reimplementation needs all three, not just the per-room one.

### 2.4 Reverb mix and the submerged-occlusion model — arithmetic recovered

**Grade: CONFIRMED** (globals, defaults, formulas) / **MEDIUM** (the submerged gate condition).

Config vars are registered by `FUN_0067F9B0(type, &global, &default)` with the **name string
in ECX**; type `1` = int, `2` = float. That call shape makes every config var's backing
global directly recoverable.
[BIN: registration sites disassembled at 0x56FF60ff / 0x5703A0ff, Thief2.exe NewDark 1.28]

| config var | global | default | meaning |
|---|---|---|---|
| `sfx_source_reverb_mix` | `0x7F91C8` | **0.6** | per-source reverb send |
| `sfx_gain_scale` | `0x7F91CC` | — | |
| `submerged_sound_occlusion` arg 1 | `0x7F91DC` | **−1e6** | min blocking factor |
| … arg 2 | `0x7F91E0` | **1.0** | distance penalty factor |
| … arg 3 | `0x7F91E4` | **−1.0** | submerged reverb-mix override |

The 0.6 default **matches `new_config_vars.txt` exactly** — documentation and binary agree.
Args 1 and 3 default to out-of-range sentinels (−1e6, −1) meaning *inert*; arg 2 defaults to
a live 1.0. The var accepts up to 3 args (`MOV [ESP+0x20], 3` before the call), matching the
documented `<min blocking> [<dist penalty> [<reverb mix>]]` signature.

**Reverb-mix selection** (`0x570D99`–`0x570DE1`):

```
mix = *0x7F91C8                       // sfx_source_reverb_mix (0.6)
if (*0xA2ABFC != 0)                   // submerged state active
    if (*0xA2AC04 == 8) { mix = *0x7F91E4; mode = 3; }   // submerged override
    else                 {                  mode = 2; }
source->vtbl[0x10C](mix)              // per-source reverb send
```

**Distance penalty and blocking floor** (`0x577FB7`–`0x577FFC`):

```
dmax  = (src[0x10] > 0) ? src[0x10] : src[0x0C]          // max, else base radius
dist  = dist + (dmax - dist) * 0.5 * (*0x7F91E0)         // 0.5 is a hard double const
blocking = max(blocking, *0x7F91DC)                       // min-blocking is a FLOOR
```

[BIN: 0.5 read as `double` from 0x78F648, Thief2.exe NewDark 1.28]

Two things worth noting. The distance penalty is a **lerp toward the source's own max
radius**, not a multiplier on distance — at the default factor 1.0 the effective distance
becomes the **midpoint** between true distance and max radius, i.e. submerged sources read
as roughly twice as far as they are. And the min-blocking factor is a **floor** (`max`), so
raising it guarantees a minimum occlusion/low-pass regardless of geometry — which is exactly
what the documentation describes as making underwater "less direct".

**The gate — RESOLVED (grade: CONFIRMED).** Decompiling the enclosing function
`FUN_00577DB0` shows the distance-penalty and blocking-floor block is guarded by:

```c
if (DAT_00a2a984 != 0                       // a global object id
    && *listener == DAT_00a2a984            // THIS object is that object
    && (obj = FUN_00506f60(*listener)) != 0 // resolve it
    && *(int *)(obj + 0x8C) == 8)           // its medium/state field == 8
{ ...distance penalty + min-blocking floor... }
```

[BIN: FUN_00577DB0 decompiled, guard at 0x577FB5, Thief2.exe NewDark 1.28]

Two consequences:

1. **The submerged occlusion applies to ONE object only** — the one whose id is in the
   global `0xA2A984`, which in a single-player game is the player. It is not a general
   per-listener effect. Worth flagging against our "simulation over player-centric hacks"
   rule: here the *original* is player-specific, so matching it means accepting a
   player-only path.
2. `+0x8C == 8` is the same constant as `*0xA2AC04 == 8` in the reverb-mix path, so
   `0xA2AC04` is a cached copy of that object field. **8 is the submerged/medium code.**
   Note it is *not* the `WRCell+0x05` medium enum (1=air, 2=water) — a different encoding,
   so do not conflate them.

### Suggested verification

1. ~~Find the file-var descriptor behind the acoustics globals~~ — **done, §2.3.**
2. ~~Confirm the room-type → EAX-preset mapping~~ — **done, §2.2. It is an identity
   mapping: the two tables are parallel arrays over one index space.**
3. Xref `al_reverb.ini` (`0x78BF9C`) to its loader and recover the field parse order, so
   `al_reverb.ini`'s 23 columns bind to names rather than positions. The header comment in
   the shipped file already names them; this would confirm the binary agrees.
4. ~~Xref `submerged_sound_occlusion` and `sfx_source_reverb_mix` to their consumers~~ —
   **done, §2.4.** Remaining sub-question: identify the `0x577FB5` gate and the meaning of
   `0xA2ABFC` / `0xA2AC04 == 8`.
5. Verify `0x00A2AA34` is the per-room acoustics storage (§2.3), and find how a room's
   stored value is read at runtime.
6. Repeat §2.2 on retail `T2-107`. If the tables differ, that is a NewDark divergence and
   falls under the gated-decision rule — surface it rather than adopting silently.
7. Apply the same `find_refs.py` technique to `gHearStatDesc` (AI hearing) — the same
   file-var pattern should expose the **hearing model's field names** as literal strings.

---

## 3. The file-var inventory — every externally-tunable data block

**Grade: CONFIRMED.** The MSVC RTTI for `cFileVar<Payload, &gDescriptor>` encodes **both**
the payload struct name and its descriptor global, so one query over the strings table
enumerates every file-var in the engine. 26 of them.
[BIN: RTTI strings matching `cFileVar`, Thief2.exe NewDark 1.28]

| Area | payload → descriptor |
|---|---|
| **Audio / AI senses** | `sHearingStats` → `gHearStatDesc` · `sAISoundTweaks` (cFileVar2) · `sAIAcuitySets` (cFileVar2) · `sAcousticsProperty` → `gMissionAcoustics` / `gGameSysAcoustics` |
| **AI** | `sAIPathOptions` → `g_AIPathFileVarDesc` · `sAIGamesysPathOptions` → `g_AIGamesysPathFileVarDesc` · `sAICreatureSizes` (cFileVar2) |
| **Combat / impact** | `sCombatVars` → `gCombatVarDesc` · `sBashVars` → `gBashVarDesc` |
| **Render / sky / weather** | `sMissionRenderParams` → `gRenderParamsDesc` · `sEnvMapTable` · `sSkyMode` · `sMissionSkyObj` · `sMissionCloudObj` · `sMissionStarObj` · `sMissionDistantObj` · `sMissionWeather` · `sMissionFogZones` · `sWaterBanks` → `gWaterBankDesc` |
| **Music** | `sMissionSongParams` → `gSongParamsDesc` · `sThemeSaveLoad` → `gThemeVarDesc` |
| **Game / mission** | `sDarkSettingsVar` · `sMissionData` · `sMapSourceData` (automap) · `sVisited` · `sMissLoopState` · `sScriptManTimerID` |

**Why this matters:** for every one of these 26 structs the **field names, sizes, byte
offsets and (often) default values are recoverable from the binary** with no decompilation.
`analysis/re/dump_filevar.py` automates the whole recipe.

#### The file-var registration block format (fully mapped)

```
header (variable length, >= 0x28)
    +0x00  char  var_name[12]      inline, e.g. "AIHearStat", "RENDPARAMS"
    +0x0C  char* label             e.g. "AIHearingStats"
    +0x10  char* payload_type      e.g. "sHearingStats"    <-- the anchor
    +0x14  int   flags[5]
    (optional) inline array of char* enum value-names — RENDPARAMS has four
               shadow-mode names here, which makes its header 0x44 not 0x28

field records, stride 0x40, after the header (NOT always immediately after)
    +0x00  char  label[0x20]       inline
    +0x20  int   ui_range          editor slider bound; 0 = unset
    +0x24  int   size              FIELD SIZE IN BYTES — 4, 12 (vec3), 16 (char[16])
    +0x28  int   struct_offset     byte offset into the payload struct
    +0x2C  int   (zero in every record seen)

trailer — relative to the INLINE .data copy of the payload type name
    +0x20  int   struct_size
    +0x28  int   field_count
    +0x2C  char* pointer back to the field array   <-- cross-check
    +0x30  defaults template (when present)
```

**`+0x24` is a SIZE, not a type code.** Offsets chain exactly against it, and the C type is
*not* encoded — `sHearingStats` stores floats and int32s in fields that both report size 4.
Read values as both and let the magnitudes decide.

**Two failure modes, both now detected by the tool rather than producing plausible garbage:**

1. **No inline defaults template.** `+0x30` then lands on the *next* file-var header, whose
   first bytes are an inline variable name. `sWaterBanks` (→ `FlowColor`) and
   `sMissionRenderParams` (→ `Global`) are both like this; their defaults are
   zero-initialised or set in code.
2. **Field-array mismatch.** When the trailer's back-pointer disagrees with the records
   walked, the record set is wrong for that file-var — `sBashVars` mismatches, and its
   "defaults" decode to ASCII. The tool now refuses to report rather than guess.

**Anchor on the TRAILER, not the header.** The trailer carries `field_count` *and* a pointer
straight to the field array, so the header is never needed to find the fields. This matters
because **`cFileVar2` registers no `.data` pointer to its type-name string** — a
header-first search simply fails on `sAISoundTweaks`, `sAIAcuitySets` and
`sAICreatureSizes`, in both the game and the editor binary. Trailer-first resolves all of
them, and gives an exact record count so no terminator heuristic is needed. The header is
then optional metadata (variable name + label), found by looking for a pointer to *any*
copy of the type-name string whose `-0x10` holds a plausible inline name.

**Three guards, because each failure mode produced confident garbage before it was added:**

1. **No inline defaults template** — `+0x30` lands on the next file-var header; detected by
   its printable inline variable name (`sWaterBanks` → `FlowColor`, `sMissionRenderParams`
   → `Global`).
2. **Implausible template** — `+0x30` lands on non-ASCII unrelated data. Signature: nonzero
   fields exist but **none** decode to a normal-range float, i.e. a counter/index table.
   `sAIAcuitySets` hits this.
3. **Field-array disagreement** — obsolete once anchoring moved to the trailer, since the
   array pointer now comes from the trailer by construction. It was what originally exposed
   `sBashVars`.

Several land directly on our roadmap: `gRenderParamsDesc` can validate `RenderParamsParser.h`;
`gWaterBankDesc` bears on the water-flow work; `g_AIPathFileVarDesc` on AI pathing;
`gSongParamsDesc` / `gThemeVarDesc` cover the music system we have not implemented.

### 3.1 `sHearingStats` — the AI hearing model, fully recovered

Descriptor `AIHearStat` at `0x814828` (label `"AIHearingStats"`, type `"sHearingStats"`),
records from `0x814850`, stride `0x40`, `type=4` (float) throughout.
[BIN: descriptor records dumped at 0x814828/0x814850, Thief2.exe NewDark 1.28]

| label | struct offset | | label | struct offset |
|---|---|---|---|---|
| `VeryLow: DistMul` | 4 | | `VeryLow: DB Add` | 28 |
| `Low: DistMul` | 8 | | `Low: DB Add` | 32 |
| `Normal: DistMul` | 12 | | `Normal: DB Add` | 36 |
| `High: DistMul` | 16 | | `High: DB Add` | 40 |
| `VeryHigh: DistMul` | 20 | | `VeryHigh: DB Add` | 44 |

Reconstructed layout — two parallel `float[6]` arrays, index 0 unlabelled (very likely an
unused/zero tier), indices 1–5 = VeryLow…VeryHigh:

```c
struct sHearingStats {   // 48 bytes
    float distMul[6];    // +0x00, labelled entries at +4..+20
    float dbAdd[6];      // +0x18, labelled entries at +28..+44
};
```

**The model: AI hearing acuity is five tiers, each contributing a distance multiplier and a
decibel offset.** Two points for our AIHearingService: the tiering is *five* levels (not the
four alertness levels), and **the engine reasons about loudness in decibels** — an additive
dB offset per tier, not a linear gain. That matches our centibel-based schema volumes.

#### Defaults — CONFIRMED, with the array reading now settled

The header trailer carries `{struct_size, ?, field_count, field_array_ptr}`; for this
file-var that is `{0x30 (48), 0, 10, 0x814850}` at `0x814AF0`. The **defaults template**
follows inline at **`0x814B00`**, and `FUN_004909A0` copies exactly **12 dwords (48 bytes)**
from it into an instance — independently confirming the struct size.
[BIN: template @0x814B00, copy loop in FUN_004909A0, Thief2.exe NewDark 1.28]

| tier | `distMul` (float) | `dbAdd` (**int32**) |
|---|---|---|
| VeryLow | **0.25** | **+1000** |
| Low | **0.65** | **+200** |
| Normal | **1.00** | **0** |
| High | **1.50** | **−200** |
| VeryHigh | **3.00** | **−1000** |

Unlabelled slots: offset 0 = `0`, offset 24 = `1000000`. So the `[6]` reading is now
**CONFIRMED** — two parallel 6-element arrays with index 0 reserved (the `1000000` reads as
a "never / infinite" sentinel for a zero-acuity tier).

```c
struct sHearingStats {   // 48 bytes
    float distMul[6];    // [0]=0 unused, [1..5] = VeryLow..VeryHigh
    int32 dbAdd[6];      // [0]=1000000 sentinel, [1..5] = VeryLow..VeryHigh
};
```

**Note the mixed types:** `distMul` are floats, `dbAdd` are **integers** — the descriptor's
size column is 4 for both and does not encode the C type, so this is only visible from the
values. Reading `dbAdd` as float yields garbage (two of the five are NaN).

#### RESOLVED — six tiers, and the sign convention is settled

> **Correction.** Earlier passes called index 0 "reserved/unused padding". It is not: it is
> **`Deaf`**, a real sixth acuity tier. The six ratings are
> **Deaf, VeryLow, Low, Normal, High, VeryHigh.**

**Units: centibels (0.1 dB), stored as int32.** **Sign: positive `dbAdd` = MORE attenuation
= harder to hear.**

The `Deaf` slot is the proof, and it is decisive. `dbAdd[Deaf] = 1,000,000 cb = +10,000 dB`.
No reading except "attenuation added" makes a *deaf* AI deaf — if the value were added to
perceived loudness, Deaf would hear everything in the level. So the ladder reads cleanly as
attenuation: VeryLow **+10 dB harder**, Low **+2 dB**, Normal **0**, High **−2 dB**,
VeryHigh **−10 dB easier**. That closes the open question from earlier passes.

#### Our implementation was already correct — this is a validation, not a fix

`src/services/audio/AIHearingData.h` already carries the exact shipped values
(`dist_muls {0, 0.25, 0.65, 1.0, 1.5, 3.0}`, `db_adds {1000000, 1000, 200, 0, -200, -1000}`),
a `static_assert(sizeof == 48)` matching the chunk, the six-tier `Deaf..VeryHigh` naming, the
centibel unit, and an explicit note that **`db_adds` are int32 and decoding them as floats
produces denormals/NaNs** — the exact trap this investigation rediscovered from the binary.

The same file's `AISNDTWK` decoder matches §3.2 exactly: six sound types
(`Untyped, Inform, MinorAnomaly, MajorAnomaly, NonCombatHigh, Combat`) as int32 world units.

**Independent agreement between our parser and the binary/chunk analysis on both structs.**
Worth noting the direction of value here: this round the existing code corrected *my*
analysis, not the other way round.

### 3.2 `sAISoundTweaks` — AI sound categories

Labels at `0x814B30`+ (same stride/format), payload type string at `0x814CB0`:
`Default Untyped Range`, `Default Inform Range`, `Default Minor Anomoly Range`,
`Default Major Anomoly Range`, `Default Non-combat High Range`, `Default Combat Range`.
(“Anomoly” is the engine's own spelling.)

**Six AI-sound categories, each with a propagation range.** Shipped values read from the
gamesys chunk `AISNDTWK` (24 bytes, §5) — all **int32**:

| off | field | shipped value |
|---|---|---|
| 0 | Default Untyped Range | **18** |
| 4 | Default Inform Range | **18** |
| 8 | Default Minor Anomoly Range | **18** |
| 12 | Default Major Anomoly Range | **32** |
| 16 | Default Non-combat High Range | **48** |
| 20 | Default Combat Range | **48** |

A clean three-tier ladder in world units: routine sounds **18**, a major anomaly **32**,
high-alert and combat **48**. Reproducing AI sound propagation means reproducing these
categories and their ranges.

> **CORRECTION — this supersedes an earlier, wrong entry.** A previous pass reported these
> as *floats* `1.0, 1.0, 1.0, 0.3, 3.0, 1.0` and reasoned about them as "multipliers on a
> base range". That was wrong on both counts. Those six floats are the **first six values of
> the `sAIAcuitySets` template** (§3.3), which happens to sit at `+0x30` from
> `sAISoundTweaks`' inline type name — the heuristic pointed at a neighbouring struct's
> template. Reading the gamesys chunk settles it: the values are integers and they are
> absolute ranges. **A template found by offset heuristic must be cross-checked against the
> chunk before it is believed.**

### 3.3 `sAIAcuitySets` — AI *vision* acuity: **SIX** modes, not four

**Grade: CONFIRMED** (layout) / **UNRESOLVED** (defaults).
Struct **72 bytes, 18 fields**, array at `0x814D28`.
[BIN: trailer @0x8151C8, field array @0x814D28, Thief2.exe NewDark 1.28]

> **Correction.** An earlier pass read this by locality off the string pool and reported
> *four* vision modes. The trailer's exact field count is **18**, i.e. **six** modes × three
> factors. The two missed modes are `Movement Only` and `Low Light`.

| off | mode × factor | | off | mode × factor |
|---|---|---|---|---|
| 0/4/8 | **Normal**: Lighting / Movement / Exposure | | 36/40/44 | **Light Only**: … |
| 12/16/20 | **Peripheral**: … | | 48/52/56 | **Movement Only**: … |
| 24/28/32 | **Omni**: … | | 60/64/68 | **Low Light**: … |

Every AI vision mode is parameterised on the same three inputs — **Lighting, Movement,
Exposure** — which is the whole visibility model in one table. `Movement Only` and
`Low Light` are clearly special sense modes (creatures that track motion, or see in
darkness).

#### Shipped values — CONFIRMED from the gamesys chunk `AIACS` (72 bytes, §5)

| mode | Lighting | Movement | Exposure |
|---|---|---|---|
| Normal | 1.0 | 1.0 | 1.0 |
| Peripheral | **0.3** | **3.0** | 1.0 |
| Omni | 0.8 | 1.4 | 1.2 |
| **Light Only** | 1.0 | **0** | **0** |
| **Movement Only** | **0** | **5.0** | **0** |
| **Low Light** | **6.0** | 1.0 | 1.0 |

**The mode names predict the values exactly, which is what makes this self-validating:**
`Light Only` responds *only* to lighting (movement and exposure both zero); `Movement Only`
responds *only* to movement, and strongly (5.0); `Low Light` is hypersensitive to lighting
(6.0); `Peripheral` is poor at lighting (0.3) but excellent at catching motion (3.0) —
exactly how peripheral vision behaves. `Normal` is the 1.0 baseline.

These are **per-mode multipliers on three sense inputs**, and they are the whole AI
visibility model. Directly relevant if we ever reimplement AI vision.

The in-binary template for this struct is at `0x814CE0` — i.e. at `+0x30` from
*`sAISoundTweaks`'* inline type name, not its own. That mis-seating is what produced the
§3.2 error; the chunk is the authority.

### 3.4 Other file-vars extracted

**`sAIPathOptions` / `AIPATHVAR`** — struct 4 bytes, 1 field.
[BIN: header @0x80EFF0, trailer @0x80F078, Thief2.exe NewDark 1.28]

| off | field | ui | default |
|---|---|---|---|
| 0 | `Pathable Water` | 1 (bool) | **1 (true)** |

A single mission-wide switch for whether AI pathfinding may cross water. Worth knowing for
our AI pathing work — it is a per-mission flag, not a per-region property.

**`sCombatVars`** — struct 8 bytes, 2 fields.
[BIN: header/trailer @0x81C8D8, defaults @0x81C8E8, Thief2.exe NewDark 1.28]

| off | field | default | type |
|---|---|---|---|
| 0 | `backstab bonus` | **10000** | int32 |
| 4 | `'in combat' min distance` | **25.0** | float |

Note the mixed types again — `backstab bonus` is an integer 10000 (a damage-scale
multiplier in some fixed-point unit), `in combat min distance` a float in world units.

**`sWaterBanks` / `WATERBANKS`** — struct 32 bytes, 8 fields, label `"Water Colors"`.
[BIN: header @0x7EF3AC, trailer @0x7EF5F8, Thief2.exe NewDark 1.28]

Four banks × `{Color (size 4, ui=14), Alpha (size 4, ui=10)}` at offsets 0/4, 8/12, 16/20,
24/28. `ui=14` on the colour fields is a distinct editor widget code (a colour picker), and
size 4 means the colour is a **packed 32-bit value, not a vec3** — worth noting against our
water work, which currently reasons in `Vector3`. No inline defaults template.

Immediately after WATERBANKS the same `.data` run continues with further file-var headers —
`FlowColor`, `Z-Bias`, `Bump Map` — i.e. the water/render registrations are packed
contiguously. Useful for locality-based discovery.

**`sMissionRenderParams`** — trailer confirms **struct_size = 248, field_count = 27**,
exactly matching the reconstruction in §4 (which had derived "≥248" and 27 records from the
field walk alone). Independent confirmation of that layout.

**`sBashVars` / `BASH`, label `"Bash Vars"`** — struct 8 bytes, 2 fields, array at
`0x7F7458`. Resolved once the tool switched to trailer-first anchoring (the earlier header
scan had found the wrong array). Field names are fuller than the first pass suggested:
`bash velocity threshold` (off 0) and `bash velocity coeff` (off 4).
[BIN: header @0x7F750C, trailer @0x7F74F8, Thief2.exe NewDark 1.28]

Defaults pass the plausibility guard but read oddly — threshold `1` (int) and coeff
`216325.0` — so grade them **MEDIUM**; the coefficient magnitude is not obviously sane and
the template location may still be wrong for this one.

### 3.5 `DromEd.exe` — surveyed, and it adds nothing here

**Grade: CONFIRMED.** The editor was expected to be the richer source. For file-vars it is
not:

- **Identical file-var inventory** — 27 payload types in both binaries, with **zero**
  editor-only and **zero** game-only entries. The editor and the game share exactly the same
  tunable-data surface.
- **`cFileVar2` behaves the same** — `sAISoundTweaks`, `sAIAcuitySets` and
  `sAICreatureSizes` register no `.data` pointer to their type-name string in the editor
  either, so this is inherent to `cFileVar2`, not a property of the game build.
- **Imports**: 11 modules / 402 symbols (game: 9 / 369). Same 7 `WINMM` symbols, still no
  `mmio*` family, still no audio API — so the editor also loads its audio backend
  dynamically.

Ghidra analysis of `DromEd.exe` completed successfully and the project exists
(`DromEd_ND128`), so it remains available for questions the game binary cannot answer —
but the file-var channel is fully covered by the game binary alone.

### Suggested verification

1. Extract struct offsets for §3.2 and §3.3 the same way as §3.1.
2. Recover the *default values*. The descriptors give offsets; the defaults live in the
   target globals, and the `.data` neighbourhood already shows embedded floats
   (e.g. near `0x814B07`).
3. Resolve the offset-0 / offset-24 question in `sHearingStats` — is index 0 a real tier?
4. Run the same recipe over `gRenderParamsDesc` and compare against `RenderParamsParser.h`;
   a mismatch is a live bug in our parser.

---

## 4. `sMissionRenderParams` — NewDark extended it; our parser matches retail

**Grade: CONFIRMED** on both binaries via `analysis/re/dump_filevar.py`.

**Correction to §3's descriptor documentation:** the field record's `+0x24` word is **the
field's size in bytes**, not a type enum. Every offset chains exactly
(`0`+16→16, `16`+12→28, `28`+4→32, `36`+12→48 …), and `sHearingStats`' uniform `4` is simply
"all floats". Sizes seen: `4` (int/float/BOOL32), `12` (vec3), `16` (char[16]).

### 4.1 Retail `T2-107` — matches `RenderParamsParser.h` exactly

| off | size | label |
|---|---|---|
| 0 | 16 | Palette Res |
| 16 | 12 | Ambient Light |
| 28 | 4 | Use Sunlight |
| 32 | 4 | **Quad Sunlight** (`ui=1` → boolean) |
| 36 | 12 | Sunlight Direction |
| 48 | 4 | Sunlight Hue (0-1) |
| 52 | 4 | Sunlight Saturation (0-1) |
| 56 | 4 | Sunlight Brightness (0-1023) |

Ends there. Our documented 84-byte layout adds the two cached vec3s (`sun_scaled_rgb`,
`sun_rgb`) at 60 and 72, which the descriptor does not expose because they are derived, not
editable. **Our parser is correct for original behaviour.**

### 4.2 NewDark `ND128` — two changes

**(a) Offset 32 changed meaning: `Quad Sunlight` (bool) → `Sunlight Mode` (4-value enum).**
The `ND128` header carries an inline array of four value names at `+0x30`:

| value | name |
|---|---|
| 0 | `Single Unshadowed` |
| 1 | `Quad + Objcast Shadows` |
| 2 | `Quad Unshadowed` |
| 3 | `Single + Objcast Shadows` |

[BIN: enum name pointers at 0x7E817C..0x7E8188, Thief2.exe NewDark 1.28]

This is the retail boolean widened to two independent bits — *single vs quad* **and**
*unshadowed vs object-cast shadows*. Note the ordering is **not** a clean bitfield: "quad"
is true for values 1 and 2 only, so a boolean read (`!= 0`) misreads value **3**
(`Single + Objcast Shadows`) as quad. Retail data only ever stores 0 or 1, so this can only
bite on NewDark-era missions.

**(b) The struct grew from 60 to ≥248 bytes**, adding:

| off | size | label |
|---|---|---|
| 84 | 4 | Required View Dist |
| 100–195 | 12 each | **Ambient Light Zone 1..8** |
| 196 | 4 | Global AI Vis Bias |
| 200–231 | 4 each | Amb Zone AI Vis Bias 1..8 |
| 232 | 16 | Motion DB Res |

(60–83 are the two derived sun vec3s; 88–99 unlabelled.)

Eight per-zone ambient colours bear on object lighting; the AI vis biases are a stealth/AI
visibility knob with a global value plus a per-zone override.

**Status: BOTH ADOPTED, 2026-08-03.** Under the standing policy (follow the NewDark
construction where it stays compatible with original content — `CLAUDE.md` → Reference
Implementations), (a) is a backward-compatible reinterpretation and (b) is additive, so
neither needed gating.

Implemented in `src/main/RenderParamsParser.h`:
- `enum class SunlightMode` with `sunlightModeIsQuad()` / `sunlightModeCastsObjectShadows()`
  helpers. `sunlightQuad` is now *derived* from the enum, so retail values 0/1 decode
  exactly as before and value 3 stops being misread.
- The extended fields are parsed into `requiredViewDist`, `ambientZone[8]`,
  `globalAiVisBias`, `ambZoneAiVisBias[8]`, with `hasNewDarkFields` recording whether the
  chunk carried them.

**Original content is provably unaffected:** every new read is gated on `chunkSize >= 88`
or more, and shipped missions carry an 84-byte chunk (§4.3), so no new branch can fire on
original data. Build clean; suite 463 passed / 1 skipped / 0 failed.

**Not consumed yet:** `ambientZone[]` is stored but unused — how a zone is *selected* for a
given object is not established, so `ObjectIllumination` still uses the single global
ambient. Resolving that lookup rule is the follow-up.

### 4.3 Class A — the extended fields are inert on original content

**Grade: CONFIRMED.** The question "does NewDark use this to improve the *original* levels,
or only fan content?" is answered by what the shipped missions store:

| mission | RENDPARAMS chunk |
|---|---|
| miss6 / Miss8 / miss14 / Miss16 | **v1.0, 84 bytes** (all four) |

84 bytes = the retail layout exactly (60 bytes of descriptor-exposed fields + the two
derived sun vec3s at 60 and 72). The NewDark struct implies **≥248 bytes**. So the shipped
levels carry **none** of the ambient-zone, AI-vis-bias, Required-View-Dist or Motion-DB-Res
values, and the retail exe's descriptor does not define them at all.

**Conclusion: NewDark's extended RENDPARAMS cannot improve the original levels.** The new
fields are reachable only by content authored or re-saved in a NewDark DromEd — i.e.
**Class A, fan-content-only** per `PLAN.BINARY_RE_PIPELINE.md` §3.0.1.

Contrast with §2: the EAX/EFX reverb work is **Class B** — original levels already set
per-room acoustic types, and those types *are* EAX preset indices, so NewDark's better
reverb path improves shipped content with no new authoring. Same era, same subsystem area,
opposite answers. Always run the classification recipe.

### 4.4 What "Quad" means, and why the stakes are low

**"Quad" = quadratic falloff. CONFIRMED** by a sibling field: the per-light property
`sLightProp` has a field literally labelled **`quad lit`** (`0x7EA728`, alongside
`inner radius (0 for none)`), i.e. per-light quadratic falloff. The sunlight field is the
same concept applied to the sun. NewDark's four modes therefore combine two independent
bits: *falloff* (single | quad) × *object-cast shadows* (off | on).
[BIN: sLightProp field labels @0x7EA6E8..0x7EA768, Thief2.exe NewDark 1.28]

**Hypothesis (grade: MEDIUM, not yet confirmed): this is a lightmap-BAKE parameter, inert at
runtime.** Sunlight in this engine contributes to the baked lightmaps, and object-cast
static shadows are likewise baked; a shipped `.mis` already contains the result. On that
reading the field is *descriptive metadata about how the lightmaps were computed*, not a
runtime render toggle. Supporting but not conclusive: the game binary contains exactly one
code reference to the descriptor (the file-var registration itself). Confirming this needs
the storage global located and its reads traced — not yet done.

**Practical position: we parse `sunlightQuad` and never read it.** Grep over `src/` returns
three hits only — the layout comment, the struct member, and the assignment in the parser.
There is no consumer. So the value-3 misread has **no observable effect today**, and both
gated decisions above are about future-proofing rather than fixing a live defect.

---

## 5. LGMD sub-object `+0x09` is a joint index

Recorded in full in `.claude/NOTES.PROJECT.md` (data-derived from `obj.crf`, not from a
binary — 1782 models / 2207 sub-objects, two independent exact predictions, both 100%).
Summarized here because it is the first result the pipeline's discipline was applied to.

**Grade: CONFIRMED.** `joint_idx == -1` iff `movement == 0` (2207/2207); the `@sNN`
authoring prefix in the sub-object name equals the field value (425/425). Three sub-objects
carry a value `>= num_objs`, directly refuting the previous `parent` reading.

**Correction to a third-party claim:** the RTX-Remix knowledge base states the engine caps
at 6 joints. Stock data uses joint indices up to **13** (`MECLOCK2.BIN`, `CLOCK.BIN`,
`MECLOCK.BIN`, `CAMERACO.BIN`, `tower2.bin`), so no 6-value ceiling should be coded against
until that cap is re-derived.

---

## 6. What should inform OUR audio pipeline

Synthesis across §1–§5, ranked by how directly it should change what we build. Everything
here is **Class B** (affects original levels) unless marked otherwise.

### 6.1 Adopt — high confidence, directly actionable

**1. Room acoustic type → EAX preset → reverb parameters — as a DESIGN TARGET, not a
runtime path.** A room's acoustic type *is* an EAX preset index (§2.2, identity mapping over
26 entries), and every stock mission already authors it. `al_reverb.ini` then gives the full
parameter set for that index — decay time, reflections + delay, HF/LF damping, room size,
air absorption, echo/modulation. **This is the level designer's intended reverb, per room,
for every shipping mission.**

> **Decision (2026-08-03):** Steam Audio produces all reverb **directly** from geometry and
> acoustic materials. The EAX preset table is a **sanity reference** — *not* a fallback
> path, *not* a runtime lookup, and **not an authoritative target**. A preset-driven runtime
> path would be an authored-value shortcut that masks simulation errors instead of exposing
> them, which the no-hacks rule exists to prevent.

**The presets are not more correct than us — they are almost certainly less.** EAX offers a
fixed palette of **26 generic environments**; a level designer picked the nearest match from
a menu for each room. We simulate that specific room's actual geometry and materials. Once
tuned, physical modelling should be *more* realistic than a 1998 preset chosen by eye. So:

- **Divergence is the expected outcome, not a defect.** Only *gross* or *categorical*
  divergence is informative.
- **Never tune parameters to minimise divergence.** Fitting our physical model to a coarse
  26-entry palette would actively degrade it. The reference is a smoke detector, not a
  fitness function.

**What the reference is genuinely good for:**

1. **Gross error detection.** If a stone corridor simulates to RT60 0.2 s where the preset
   says ~2.7 s, something is broken — missing geometry, wrong acoustic material, misplaced
   probe. Order-of-magnitude, not parameter matching.
2. **Character ordering.** Do rooms the designer marked *dead* land on our damped end, and
   *live* rooms on our reverberant end? **Rank correlation across rooms** is the robust
   test, and it is insensitive to the palette being coarse.
3. **Designer intent as context.** Where someone picked `Cave` for an indoor space, that
   records intended *mood* which physics will not reproduce — useful to know before
   "correcting" it.

The EAX parameters map onto quantities extractable from a Steam Audio impulse response,
which is what makes even a coarse comparison possible:

| EAX parameter | measurable counterpart |
|---|---|
| `DecayTime` | RT60 (broadband) |
| `DecayHF` / `DecayLF` | RT60 ratio per band |
| `Reflections`, `ReflDelay` | early-reflection level and onset |
| `Reverb`, `RevDelay` | late-field level and onset |
| `Room`, `RoomHF` | overall level and HF rolloff |
| `Size` | room size / mean free path |

**Proposed use:** a per-room comparison harness over the shipping missions — extract RT60
per band from our simulated IR, look up the preset via the room's acoustic type, and report
**rank correlation plus outliers**, not absolute error. Flag only rooms that disagree
grossly or land in the wrong character class. No preset data reaches the runtime signal
path, and no tuning loop optimises against it.

**2. `sfx_source_reverb_mix = 0.6`** (§2.4). The engine's per-source wet/dry send. Our
default should be this number, not an invented one.

**3. AI sound categories with absolute ranges** (§3.2, from `AISNDTWK`): Untyped 18,
Inform 18, Minor Anomaly 18, Major Anomaly 32, Non-combat High 48, Combat 48 world units.
AI sound propagation should classify by these six categories at these ranges — not a single
radius. Note the ladder is 18 / 32 / 48, not a smooth curve.

**4. AI hearing acuity** (§3.1, from `AIHearStat`): five tiers, each a distance multiplier
**and** an additive offset — `distMul` 0.25 / 0.65 / 1.0 / 1.5 / 3.0, `dbAdd` +1000 / +200 /
0 / −200 / −1000. The engine reasons about loudness **additively in decibel-like units**,
consistent with our centibel schema volumes. Caveat: the **sign convention is still MEDIUM**
— +1000 for the *worst* hearing only makes sense as a threshold offset, and that has not
been observed directly. Confirm before implementing the direction.

### 6.2 Adopt if/when we implement the feature

**5. Submerged occlusion** (§2.4) — the exact model, and its shape is non-obvious:
`dist += (maxRadius − dist) × 0.5 × penalty` (a **lerp toward the source's max radius**, so
at the default penalty a submerged source reads roughly twice as far), plus
`blocking = max(blocking, minBlocking)` (a **floor**, not a scale), plus a reverb-mix
override. Applies to **one object only** (the player) and only when its medium field == 8.
Flag against our "simulation over player-centric hacks" rule: here the original *is*
player-specific.

**6. Three-level acoustic resolution** (§2.3): gamesys default → mission default → per-room.
In stock content both defaults ship as **0 (Generic)**, so rooms carry everything — but FM
support needs all three.

### 6.2b The original engine had NO per-surface acoustic materials

**Grade: CONFIRMED (negative result).** NewDark's `.mtl` material system
(`doc/material-format.txt`, 627 lines) is **entirely visual**. Every keyword is rendering:
`env_map`, `illum_map`, `render_pass`, `render_material_only`, `chroma_key_to_alpha`,
`force_alpha_key`, `no_mipmap`, `tile_factor`, `uv_clamp`, `terrain_scale`, `ani_frames`,
`ani_mode`, `ani_rate`, `edge_padding`, `priority`. **Nothing acoustic.**

So in the original engine, **reverb character came entirely from the room's EAX preset** —
there was no surface-material contribution at all. Our `AcousticMaterials.h` per-surface
keyword system has **no engine counterpart**; it is an enhancement, not a replication.

Two consequences worth holding onto:

1. It is explicitly *not* a divergence to justify under the replicate-exactly rule — the
   original simply had no such concept, and the project charter welcomes enhancements.
2. **It independently explains why divergence from the EAX presets is expected** (§6.1).
   They had one hand-picked preset per room; we have per-surface materials driving a
   simulation. These are different models, not two attempts at the same number.

### 6.3 Corrections to our existing assumptions

**7. `cZonePairTable` is NOT an acoustics structure** (§4.5) — it is AI pathfinding. Remove
it from the audio lead list; it was a false trail.

**8. Rooms matter for reverb SELECTION even though we propagate physically.** Our standing
decision is to propagate sound by physical connectivity rather than the room database, and
that stands — but this work shows rooms remain the **authored carrier of reverb character**.
The refined position: *physical connectivity for occlusion and transmission; room acoustic
type for reverb selection*. Those are different questions and the room DB is the right
source for the second one.

**9. Retail samples are plain RIFF/WAV** read through the Win32 `mmio` API (§1.2); NewDark
replaced that with its own loader. No engine-specific container to reverse.

### 6.4 Leads worth following next

- **Ambient sound object pool.** Editor strings show a fixed-size pool —
  `"Out of ambient objects!"`, `"Invalid ambient index %d requested!"`, `ambient_volume`,
  plus a `P$AmbientHacked` property with a T1-compatibility fixup. Our `AmbientSoundManager`
  has the same job; worth recovering the pool size and eviction rule.
- **`ENV_SOUND` chunk** in DARK.GAM is **19,381 bytes** — the environmental-sound database,
  by far the largest audio-related chunk. `read_chunk.py` can dump it; decoding it would
  give the stock env-sound tag set directly.
- **The `dbAdd` sign convention** (§3.1) — the one open question blocking a faithful hearing
  implementation.

## 7. The console command table — a subsystem map with authored help text

**Grade: CONFIRMED.** The engine registers console commands in a flat `.data` table of
**24-byte** records; `analysis/re/dump_commands.py` extracts them.

```
+0x00  char* name        "ai_spew_zones"
+0x04  int   (0)
+0x08  void* handler     .text address
+0x0C  char* help        "spew all AI path zones"
+0x10  int   flags       (6 on AI commands)
+0x14  int   (0)
```

**227 commands in `DromEd.exe`.** Every one carries a human-written help string *and* a
handler address, so the table doubles as a subsystem map and a set of entry points for
further analysis. This is the single richest thing the editor binary offers.

### 7.1 AI — pathfinding is a THREE-tier system

The 16 AI commands settle the structure, which was not obvious from the RTTI alone:

| tier | commands | what it is |
|---|---|---|
| **AI path cells** | `ai_build_path_database` ("Update the AI path database"), `ai_draw_cells` ("show AI path cells in wireframe"), `ai_draw_cellids` | A dedicated navmesh-like cell decomposition — **separate from WR render cells** |
| **AI path zones** | `ai_spew_zone`, `ai_spew_zones`, `ai_use_zones`, `ai_draw_zone_type` ("0 N, 1 N+L, 2 H, 3 H+L") | Four zone flavours — `Normal`, `NormalLVL`, `HighStrike`, `HighStrikeLVL` — plus **zone pairs** (`cZonePairTable`), built by "Path: constructing zones" / "Linking pathfind zones..." |
| **AI Room path DB** | `build_ai_room_database`, `ai_room_db_spew` ("Spew AI Room Path Database") | The coarsest tier, built over room brushes |

Supporting facts from the same table:

- **`ai_rehint_objects`** — *"rehint all objects (update cache for which ai cell is closest
  to each object)"*. Objects cache their nearest AI cell; it is a maintained hint, not
  computed per query.
- **`ai_sleep_all` / `ai_wake_all`** — AIs have an explicit sleep/wake state, i.e. the
  original already did AI LOD by suspension.
- **`ai_val_fpts`** ("validate flee points") — flee points are a first-class validated
  entity, not an ad-hoc search.
- **`ai_report_large_door_size`** — door size is a known pathing concern.
- **`spew_ai_collides`** — "Spew all objects AI collide with".

### 7.2 Rooms — and a named failure mode

`rooms_build` ("Convert rooms to internal rep"), `rooms_compile` ("...and update cells"),
`reconstruct_rooms`, `rooms_spew`, `next_room`, `fix_rooms` ("Fix dangling room pointers"),
and notably **`spew_bad_room_obj_assigns`** — *"List objects that have incorrect room
indexing or assignments"*.

That last one is worth noting: **object→room assignment being wrong was a known, tooled-for
failure mode in the original engine.** Anything of ours that trusts room assignment should
expect the same class of bug.

### 7.3 Sound — the original had sound-path visualisation

Only three sound commands survive in the editor, but two are significant:

- **`spew_sounds`** — *"Spews all propagating sounds"*. Propagating sounds were a live,
  enumerable list.
- **`clear_sound_path`** — *"Clears any drawn sound paths"*. **The original tooling drew
  sound propagation paths.** That is exactly what `PLAN.PROBE_DEBUG_TOOLING.md` builds, so
  the idea is validated by precedent rather than invented.
- `destroy_sound` — "Destroy all schemas and speech"; plus `reload_schemas` /
  `zggtvrk_load_schemas` for schema hot-reload.

## 8. LGMD sub-object joints — root cause of BOTH the missing prop animation and the door bugs

**Grade: CONFIRMED** for the diagnosis; **OPEN** for the rotation-axis convention.

### 8.1 Root cause: we bake the sub-object transform at parse time

`src/main/BinMeshParser.h:446-470` applies each sub-object's `rot` + `axle_point` to its
vertices **once, at load**, producing a single static mesh:

```cpp
} else {   // non-root sub-object
    const float *r = sob.trans.rot;
    const Vertex &axle = sob.trans.axle_point;
    bv.x = r[0]*v.x + r[3]*v.y + r[6]*v.z + axle.x;   // baked, frozen at rest pose
```

Every model therefore renders permanently in its rest pose. **No joint animation is
possible by construction** — the sub-object structure does not survive loading. We also
have **zero `P$JointPos` support** anywhere in `src/`.

### 8.2 Door LEAVES are the static root — the joints are handles

An earlier revision of this section claimed door leaves were jointed sub-objects and that
`DOOR20`-family models were double doors. **Both claims were wrong.** They were inferred from
sub-object *names and counts* without checking each part's vertex extents. Measuring the
per-sub-object point ranges (`point_start` / `sub_num_points` at `+0x4D`, indexing the vertex
table at `offset_verts`) settles it:

| model | root sub-object (`movement=0`) | jointed sub-objects |
|---|---|---|
| `addoor01..05.bin` | slab `4.01 x 0.26 x 8.00` | one `@s00bb`, `0.71 x 1.19 x 0.34`, centred at x≈1.6 |
| `DOOR20/21`, `DOORKEEP`, `DOORKEP2`, `OPDOOR`, `DOORTRAN` | slab `3.50 x 0.23 x 7.00` | `@s00bb` **and** `@s00cc`, `0.62 x 0.40 x 0.29`, at x≈1.3, one at −y and one at +y |
| `DOOR17SW.BIN` | flat panel `8.00 x 0.63 x 8.00` | `@s00bb` joint 0 range `[0, 6.283]`, `@s01hh` joint 1 range `[−2.094, 0]` |
| `CHESTLOC.BIN` | body `2.95 x 1.60 x 1.36` | `@s01bb` joint 1 = **lid** `2.95 x 1.47 x 0.78`; `@s00ff` joint 0 = lock plate `0.63 x 0.03 x 0.18` |

The door **slab is the static root sub-object in every stock model.** The jointed parts are
the *door handles* — one per face, hence the two on `DOOR20` (it is a single door with a knob
on each side, not a double door). `DOOR17SW` is a wall switch panel, not a door: joint 0
turns a full `2π` (a valve wheel) and joint 1 sweeps `−120°` (a lever).

Consequences:

- **Our whole-object door rotation is the correct construction.** `DoorSystem` rotates the
  root — which is the leaf — about a hinge-edge pivot. Nothing about door *swing* depends on
  joints.
- The only door motion we are missing is the **handle turning**, a cosmetic detail.
- No stock door model has a jointed leaf, so the joint work carries **no** consequence for
  door collision, door audio occlusion, or the acoustic route graph (§8.7).

### 8.3 The ranges are RADIANS

`min_range` / `max_range` (`+0x0D`, `+0x11`) read as floats:

```
addoor01 @s00bb : [ 0.000, 1.571]   = [0, pi/2]  handle quarter-turn
CHESTLOC @s01bb : [ 0.000, 1.571]   = [0, pi/2]  chest lid
DOOR17SW @s00bb : [ 0.000, 6.283]   = [0, 2*pi]  valve wheel, full turn
DOOR17SW @s01hh : [-2.094, 0.000]   = [-120 deg, 0]  lever, negative travel
```

`1.571 = π/2`, `6.283 = 2π`, `2.094 = 2π/3`. The limits are **radians**, the interval is
signed, and it need not start at zero. This confirms `movement == 1` sub-objects are hinged
rotations driven by the joint value, and gives us the authored travel per part for free.

### 8.4 Architecture: sub-objects are a BONE HIERARCHY

OPDE (Tier 1, citable) builds exactly this in `ManualBinFileLoader.cpp:875-925` — each
sub-object becomes a skeleton bone, `position = axle_point`, `orientation = rot`, parented
via the `child_sub_obj`/`next_sub_obj` tree, and flagged manually-controlled. OPDE never
*drives* the bones (no joint animation), but the structure is right and matches the
engine's dispatch.

The engine's dispatcher (`FUN_005F6D90`-adjacent) confirms the shape:
`movement = sub[+0x08]`; `0` → no transform, `1` → rotate path, `2` → slide path; both push
the current matrix onto a stack (depth 8) and compose. The joint value is fetched from a
per-object array indexed by `joint_idx` with **stride 4**.
[BIN: movement dispatch + joint-array indexing, Thief2.exe NewDark 1.28]

### 8.5 OPEN: the rotation-axis convention

Each sub-object's `rot` is a frame the artist orients so a **single fixed local axis** lands
on the intended hinge line. Which local axis is the convention is not yet established.

An earlier revision argued the matrices *contradict* any single convention, citing identical
`rot` on `DOOR20 @s00bb` and `CHESTLOC @s00ff`. That argument fell with §8.2: `@s00ff` is the
chest's **lock plate**, not its lid, and a lock plate and a door handle plausibly do turn about
the same local axis. The chest **lid** is `@s01bb`, whose `rot` is a *different* matrix
(`diag(-1, 1, -1)`). So there is no counter-example on the table and a single convention
remains the most likely answer.

**Cheapest resolution is still empirical:** implement with the axis as a constant, drive a
handle joint to `max_range`, and see which of local X/Y/Z turns it about its own shank.
Cross-check on `CHESTLOC @s01bb` (lid must tilt up on its rear edge) and `DOOR17SW @s00bb`
(wheel must spin in its own plane) — three parts with three different intents, one test.

### 8.6 Implementation plan

1. **Stop baking.** Keep sub-objects as separate submesh ranges; tag vertices with their
   sub-object index. Preserve the `child_sub_obj`/`next_sub_obj` tree.
2. **Per-sub-object local transform** = `translate(axle_point) · rot3x3`, exactly what is
   baked today — so a model with all joints at rest renders identically to now. That is the
   regression guard.
3. **Joint pose** — for `movement==1`, compose `rotateAboutAxis(axis, jointValue)` into the
   local frame; for `movement==2`, translate along the axis by the joint value. Clamp to
   `[min_range, max_range]`.
4. **`P$JointPos`** — new property (engine caps at 6 joints per object) feeding the joint
   array; `TweqSystem`'s existing `Joints` type drives it, and `StdLever.h:181` already
   reads `StTweqJoints`/`AnimS`.
5. **Doors** — unchanged. Per §8.2 the leaf is the static root, so `DoorSystem` keeps driving
   the object transform. Doors gain only a turning handle, from the same joint plumbing every
   other prop uses.

### 8.7 Blast radius on the systems already working

Measured against the current tree, not assumed. All three worries turned out smaller than
they looked.

**Door collision — no impact.** `ObjectCollisionBody` is one OBB. Its `edgeLengths` come from
`P$PhysDims.size`, falling back to the `.bin` **header** `bbox_max`/`bbox_min`
(`ObjectCollisionGeometry.h:760-815`), and `bboxCenter` likewise. Those are header fields
(`BinMeshParser.h:165-166`), never recomputed from the vertex stream, so un-baking sub-object
transforms cannot move them. The pose comes from `updateBodyTransform(objID, fullGlm)` off the
door's *object* matrix, which §8.2 leaves alone. Checked all shipping doors carry explicit
PhysDims (34/34 in MISS6, 113/113 in MISS14), so the header-bbox fallback — which *would*
include the handles, `y = 1.14` vs the slab's `0.32` — is never hit for doors in stock content.

**Audio — no impact.** `AudioService` never sees a `ParsedBinMesh`; grep for it across
`src/services/audio/` returns nothing. Object models contribute no geometry to the Steam Audio
scene. Door occluders are boxes synthesised by `DoorSystem::buildBoxMesh` from
`door.edgeLengths` (same PhysDims source), posed by `computeAudioWorldMatrix`, with the route
graph keyed off the pose-independent `closedWorldTransform`. Every one of those inputs is
untouched.

**Vertex layout — not a real risk.** `BinVert` is a CPU-side intermediate; the GPU struct is
`PosColorUVNormalVertex`, converted at upload (`DarknessRenderInit.h:1183-1227`). Adding a
field to `BinVert` does not touch a bgfx vertex layout. And it is not needed: `loadPolygons`
emits fresh vertices per polygon and never shares them, so every vertex already belongs to
exactly one sub-object — keying `mMatTriangles` on `(matIndex, subObj)` instead of `matIndex`
splits the ranges with no per-vertex attribute at all.

**The one real coupling** is `DoorSystem::populateOBBAndPivot`'s acoustic-material pick
(`DoorSystem.h:665-692`), which scans `mesh.subMeshes` for the largest-*area* submesh. Rigid
sub-object transforms preserve area, so un-baking alone is inert; but splitting submeshes by
sub-object repartitions the areas and could flip the winner. Restricting the pick to the root
sub-object fixes it and is more correct anyway — it is the exact bug the existing comment
describes (the handle material `DHANDLE` winning on 101/200 MISS7 doors) solved properly
rather than by area heuristic.

**Draw-call cost.** 241 of 1782 models have >1 sub-object; 1541 are single-part and completely
unaffected. Worst cases by `num_objs × num_mats` upper bound: `MECLOCK2` (15 × 14),
`CLOCK` (13 × 11), `MECLOCK` (10 × 11). The true count is far lower — a sub-object only
produces a range per material it actually uses.

## The toolkit

All in `analysis/re/`, all tracked. Typical flow: **import → index → locate → decode**.

| tool | what it does |
|---|---|
| `import_binary.sh <proj> <exe>` | Ghidra headless import + auto-analysis. Resolves the keg-only JDK itself. |
| `export_index.py <proj> <exe>` | Analyzed program → SQLite (`functions`, `symbols`, `strings`, `xrefs`, `blocks`). Aborts loudly if the program has 0 functions. |
| `query.py "<SQL>"` | SQL front-end. Also `--schema`, `--xref VA`, `--str PATTERN`, `--fn PATTERN`. |
| `bootstrap_imports.py <exe>...` | Import-table catalog with an audio-DLL callout. Pure `pefile`, no Ghidra needed. |
| `find_refs.py <exe> VA...` | Pointers to a VA by **raw image scan**. Also `--dump VA N` (dwords with strings resolved) and `--strings LO HI` (printable runs). |
| `dump_filevar.py <exe> <type>` | Full file-var decode: header, field labels/sizes/offsets, trailer, defaults. |
| `decompile.py <proj> <exe> VA...` | Ghidra decompiler. Also `--reads VA` (every function referencing it) and `--disasm VA N`. |
| `read_chunk.py <db> <CHUNK>` | Read a raw chunk out of a `.gam`/`.mis`. Also `--list`. **The authority for tunable values** (§5) — the binary only holds fallbacks. |

**Workflow that emerged, and the order matters:** use the binary to establish *layout*
(field names, sizes, offsets, struct size), then use `read_chunk.py` to get the *values*
from shipped data. Doing it the other way — reading values out of `.data` — is what produced
the two corrections in §3.2 and §3.4.

**Why `find_refs.py` exists, and when to reach for it.** Ghidra only creates a reference
where its analysis decided a location holds a pointer. Static descriptor tables are
routinely left untyped, so `query.py --xref` returns **nothing** for them even though the
table plainly exists — this happened on all three acoustics label strings. The raw scan does
not depend on analysis having happened. **If an xref query comes back empty, do not conclude
the address is unused — re-check with `find_refs.py`.**

**Why `--disasm` exists.** The decompiler folds some string references away entirely; the
config-var registration loop is the standard case. Both `sfx_source_reverb_mix` and
`submerged_sound_occlusion` were invisible in decompiled output and obvious in disassembly.

## 4.5 Ambient-zone selection rule — UNRESOLVED

**Grade: UNRESOLVED.** What the NewDark RENDPARAMS extension *stores* is settled (§4): eight
`Ambient Light Zone` vec3s at 100–195, each paired with an `Amb Zone AI Vis Bias` float at
200–231. **How an object or region selects a zone is not established.** Everything tried:

| approach | result |
|---|---|
| NewDark `doc/` + `release_notes.txt` | **no mention** of ambient zones or vis bias anywhere |
| `DromEd.exe` string sweep for `zone` | only **AI pathfinding** zones (see below) |
| `DromEd.exe` string sweep for `ambient` | only **ambient SOUND** (volume, object pool, `AmbientHacked`) |
| Scan `.data` for the file-var object shape `{vtable, data_ptr, 248}` | no hits — the instance is likely in `.bss`, which is not in the file image |
| Mission property chunks | no zone-index-shaped property found |

**Leading hypothesis (MEDIUM):** like `Sunlight Mode` (§4.4), this is **bake-time / editor
data** — DromEd uses the zones when computing lighting and AI visibility, and the result is
baked into the mission. That would explain the total absence of runtime documentation,
config vars and editor UI strings. It is consistent with RENDPARAMS being a bake-time struct
generally, but it is **not proven**.

**What would settle it:** locate the `sMissionRenderParams` instance (probably `.bss`, so it
needs Ghidra's memory map rather than the raw image) and check whether any `.text` code
reads offsets 100–231. No readers ⇒ editor-side only.

**Practical position:** we parse the fields (§4) and do not consume them. Since original
missions carry an 84-byte chunk with no zone data at all, nothing is lost on stock content,
and the question only becomes live for NewDark-era FMs.

### Correction: `cZonePairTable` is AI pathfinding, NOT acoustics

An earlier note listed `cZonePairTable` (RTTI `0x70E768`) as an acoustics/room-coupling lead.
The editor strings settle it: zones here are **AI path zones** in four flavours —
`Normal`, `NormalLVL`, `HighStrike`, `HighStrikeLVL` — with progress messages
`"Path: constructing zones"`, `"Linking pathfind zones..."`, `"%d Normal zones, %d zone
pairs"`, plus the debug commands `ai_use_zones`, `ai_draw_zone`, `ai_spew_zones`.
[BIN: DromEd.exe NewDark 1.28, strings @0x85F6C0..0x85F81D and @0x8590F8..0x859A84]

**Drop it as an audio lead.** Acoustic room coupling is carried by the per-room acoustic type
(§2.2), not by this table.

## 5. File-vars are CHUNK-BACKED — the shipped values are in the data, not the binary

**Grade: CONFIRMED.** This is the structural key to the whole file-var system, and it
supersedes reading defaults out of `.data` wherever the two disagree.

**A file-var's inline `var_name` IS a chunk name**, and the chunk payload is exactly the
payload struct. Verified by matching `struct_size` (from the binary trailer) against the
chunk payload length (from the shipped data) — every one agrees:

| var_name | where | chunk payload | binary `struct_size` |
|---|---|---|---|
| `AIHearStat` | DARK.GAM | 48 | **48** ✓ |
| `AIACS` | DARK.GAM | 72 | **72** ✓ |
| `AISNDTWK` | DARK.GAM | 24 | **24** ✓ |
| `BASH` | DARK.GAM | 8 | **8** ✓ |
| `DARKCOMBAT` | DARK.GAM | 8 | **8** ✓ |
| `GameSysEAX` | DARK.GAM | 12 | — (matches the `GameSysEAX` var name, §2.3) |
| `AIGPTHVAR` | DARK.GAM | 4 | **4** ✓ |
| `AIPATHVAR` | mission | 4 | **4** ✓ |
| `WATERBANKS` | mission | 32 | **32** ✓ |
| `RENDPARAMS` | mission | 84 | 248 NewDark / 60 retail (§4) |
| `MissionEAX` | mission | 12 | — |

The split is meaningful: **AI tuning is gamesys-wide** (hearing, acuity, sound ranges, bash,
combat) while **environment is per-mission** (render params, water, acoustics, path options).
That matches the two-level acoustics design in §2.3.

**The binary template is only a fallback.** Both defaults-copiers decompile to the same
shape — copy `struct_size` bytes from a pointer held in the file-var object, or
`memset(dst, 0, size)` when that pointer is null. So a struct with no template defaults to
**all zeros**, which is the honest answer for `sWaterBanks` and `sMissionRenderParams`.
[BIN: FUN_00493F80 / FUN_00494090 / FUN_004909A0, Thief2.exe NewDark 1.28]

### 5.1 Shipped values often DIFFER from the binary defaults — always read the chunk

| struct | binary default | shipped (chunk) |
|---|---|---|
| `sHearingStats` | 0.25/0.65/1/1.5/3 + 1000/200/0/−200/−1000 | **identical** ✓ |
| `sCombatVars` `backstab bonus` | 10000 | **400** |
| `sAIPathOptions` `Pathable Water` | 1 (true) | **0 (false)** in miss6 |
| `sBashVars` | (template mis-seated) | `bash velocity threshold` **500.0**, `bash velocity coeff` **0.1** |

Three of four differ. **Reading a tunable value out of `.data` and assuming it is what the
game uses is unsafe** — the chunk is the authority. `analysis/re/read_chunk.py` reads them
directly. This also finally resolves `sBashVars`: its shipped values are two sane floats,
confirming the earlier MEDIUM grading was right to withhold the `.data` reading.

### 5.2 Other shipped values read

- **`GameSysEAX` = {0, 0, 0}** and **`MissionEAX` (miss6) = {0, 0, 0}** — both default to
  preset **0 = Generic**, so in stock content the acoustic environment comes entirely from
  each room's own type, not from the gamesys/mission defaults (§2.2, §2.3).
- **`WATERBANKS` (miss6)** — all four banks identical: colour `0x000A3228` packed,
  alpha **0.35**. Confirms the packed-32-bit colour reading from §3.4 (it is not a vec3).

**Why this matters beyond the numbers:** it means every tunable struct in §3 can be read
from shipped data with `read_chunk.py` — no disassembly, no heuristics, and the values are
what the game actually runs. The binary work established the *layout*; the chunks supply
the *values*.

## Tooling notes (macOS arm64)

Two install traps cost time; both are recorded so they are hit once only.

- **Ghidra is a Homebrew FORMULA, not a cask.** `brew install --cask ghidra` fails with
  "No Cask with this name exists". Correct: `brew install ghidra` (pulls `openjdk@21`).
- **Homebrew's `openjdk@21` is keg-only**, so it is never symlinked into the system JVM
  path and `/usr/libexec/java_home` cannot see it. Ghidra's launcher then dies with
  "Unable to locate a Java Runtime" + "no TTY detected". `analysis/re/import_binary.sh`
  sets `JAVA_HOME` to the keg explicitly.
- **Ghidra rejects any path element beginning with `.`** — "Path element starting with '.'
  is not permitted". Projects therefore cannot live under `.claude/`; they are kept at
  `../re_projects/`, outside the repo (they run to hundreds of MB).
