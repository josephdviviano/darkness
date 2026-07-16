# darkness

`darkness`, an open [Dark Engine](https://en.wikipedia.org/wiki/Dark_Engine)(tm), is built on the headless core of the [openDarkEngine](https://github.com/volca02/openDarkEngine) project, and is optimized for a modular design permitting extensibility and full cross-platform compatibility on modern hardware. It aims to support all previously-generated official and fan content, and admit modular access to modernizing the graphics, audio, physics, and AI stacks.

![don't be so sure](doc/orly.png)
![but actually tho](doc/physics.png)

---

## Project status

**`darkness` is an engine under construction, not a playable game.** You cannot play Thief 1 or 2 with it yet. What you can do today: load a shipping Thief 2 mission, walk it with full player physics, open doors, ride elevators, throw levers, trip pressure plates, pick up and throw objects, and hear all of it through an modernized acoustic simulation. I still have to implement NPCs, inventory,  weapons, no objectives, no loot totals, dynamic lighting, no light gem, no HUD, save/load, game logic, AI.

### Subsystem status

| Area | Status |
|------|--------|
| Mission / resource loading (`.mis`, `.gam`, `.crf`) | **Working** |
| World rendering — geometry, textures, lightmaps, water, sky, fog, portal culling | **Working** |
| Object rendering (static models) | **Working** |
| Player physics — walk, crouch, jump, mantle, lean, stairs, slopes | **Working** |
| Object interaction — doors, elevators, levers, tweqs, pressure plates, triggers, pushable objects | **Working** |
| Frob / grab / throw | **Working** — see caveat below |
| Spatial audio — HRTF, occlusion, reflections, portal pathing, schemas, speech | **Under Construction** — active focus |
| Built-in object scripts | **Partial** — 31 scripts; see caveat below |
| Game logic — inventory, objectives, loot, keys, mission completion | **Absent** |
| NPCs / creatures — rendering, animation, behaviour | **Absent** |
| AI — senses, alertness, patrol, pathfinding, combat | **Absent** |
| Save / load | **Absent** |
| Modern rendering — shadows, SSAO, PBR, volumetrics | **Absent** |

Testing is currently being done with Thief 2. I will back port this to Thief 1. I am currently unsure if I will be able to easily adopt SS2.

## Prerequisites

- CMake 3.21+
- C++17 compiler (Clang, GCC, or MSVC)
- Git

Dependencies are managed automatically via [vcpkg](https://vcpkg.io/) manifest mode:
bgfx, GLM, SDL2, zziplib, ODE, yaml-cpp, Catch2, miniaudio, steam-audio.

## Building

```bash
# Clone the darkness repo
git clone <darkness-repo-url>
cd darkness

# Fetch and bootstrap vcpkg into ./vcpkg/ (gitignored)
git clone https://github.com/microsoft/vcpkg.git vcpkg
./vcpkg/bootstrap-vcpkg.sh   # or bootstrap-vcpkg.bat on Windows

# Configure and build (Debug)
cmake --preset default
cmake --build build/default

# Or Release
cmake --preset release
cmake --build build/release
```

The `default` preset wires vcpkg in via `vcpkg/scripts/buildsystems/vcpkg.cmake`; no extra toolchain flag is needed. The first configure builds every dependency from source and takes a while.

This produces two binaries:

| Binary | Path | Description |
|--------|------|-------------|
| `darknessRender` | `build/default/src/main/darknessRender` | World viewer (SDL2 + bgfx) |
| `darknessHeadless` | `build/default/src/main/darknessHeadless` | Mission inspector, no rendering deps |

### Platform presets

| Preset | Platform | Notes |
|--------|----------|-------|
| `default` | macOS / auto-detect | Debug build |
| `release` | macOS / auto-detect | Release build |
| `linux-x64` | Linux | Debug build |
| `windows-x64` | Windows | Debug build |

Day-to-day development happens on macOS. Linux and Windows are supported targets but currently not tested at all.

### Tests

```bash
cmake --build build/default --target darkness_tests
build/default/tests/darkness_tests            # all tests
build/default/tests/darkness_tests "[audio]"  # or a test name / [tag]
```

19 Catch2 files, ~450 cases, concentrated on world query, schema parsing, the sim/script infrastructure, and audio. Most of the shipped interaction systems (doors, moving terrain, frob, grab) have no direct coverage yet.

## Game data

Users must supply their own legally obtained game files. **`darkness` does not include any game assets.**

You need two directories:

| What | Contains | Needed for |
|------|----------|------------|
| `RES` | `fam.crf`, `obj.crf`, `snd.crf` | Required — textures, models, sounds |
| `SCHEMA` | `.sch` / `.spc` / `.arc` | Optional — sound schemas. Without it, audio is much reduced |

They can come from a GOG/Steam install directory, a mounted disc, or any directory containing those archives. On macOS, mounting the discs gives you `/Volumes/THIEF2_INSTALL_C/THIEF2/RES` and `/Volumes/THIEF2_CD2/EDITOR/SCHEMA`.

If `--schemas` is omitted, the audio service looks for `<res>/../EDITOR/SCHEMA`.

## Running

```bash
# Point at the data explicitly
darknessRender path/to/miss6.mis --res /path/to/THIEF2/RES --schemas /path/to/EDITOR/SCHEMA

# Or set paths.res / paths.schemas in the YAML once, then just:
darknessRender path/to/miss6.mis
```

The mission file is the only required argument. A `RES` directory must resolve from either `--res` or `paths.res` in the config — the program exits if it resolves to nothing or to a non-directory.

**First run on a mission bakes acoustic probes**, which takes minutes. The result is cached in a `.probes` file next to the mission and reused on subsequent runs.

## Configuration

Settings come from three places. Precedence is **YAML < CLI flags < runtime debug console**.

Copy `darknessRender.example.yaml` to `darknessRender.yaml` in your working directory, or pass `--config <path>`. The default path is relative to the current directory. A missing config is fine — defaults apply silently. A config that exists but fails to parse is fatal.

The file has six sections: `paths`, `graphics`, `water`, `physics`, `developer`, and `audio`. `audio` dominates — roughly 95 of the ~116 keys.

**`darknessRender --help` is the canonical config reference.** It lists every YAML key with its default, whether or not your local config mentions it, and is kept authoritative by convention. Prefer it over the example file if the two ever disagree. Note it prints to stderr, so redirect accordingly:

```bash
darknessRender --help 2>&1 | less
```

## Controls

Movement is suppressed while the console is open.

### Physics mode (default)

Walk-on-ground with gravity, collision, jumping, crouching, and leaning.

| Key | Action |
|-----|--------|
| WASD | Walk / strafe |
| Mouse | Look around |
| Space | Jump (when on ground) |
| LShift | Sneak (0.5x speed, hold) |
| LCtrl / RCtrl | Run (2x speed, hold) — sneak wins if both held |
| C | Toggle crouch |
| Q / E | **Lean** left / right |
| F | Throw held object, else grab, else frob |
| R | Drop held object |
| P | Toggle simulation pause |
| Home | Teleport to player spawn point |
| ` (backtick) | Open debug console |
| Esc | Quit |

### Fly mode

Noclip free-look camera. Enable via the debug console (`physics_mode` = off).

| Key | Action |
|-----|--------|
| WASD | Move forward / left / back / right |
| Mouse | Look around |
| Space, Q | Move **up** |
| LShift, E | Move **down** |
| LCtrl / RCtrl | Sprint (3x speed) |
| Scroll wheel | Adjust movement speed |
| F / R / P / Home | As above |
| ` (backtick) | Open debug console |
| Esc | Quit |

Note that **Q/E lean in physics mode but move vertically in fly mode**, and the Ctrl multiplier differs between the two (2x vs 3x).

## Debug console

Press **\` (backtick)** to open the in-game settings console. All runtime settings live here — there are no separate keyboard shortcuts for toggling features.

**73 settings** are registered across five groups: `Rendering` (20), `Movement` (12), `Water` (4), `Audio` (35), and `Interaction` (2). The console is self-documenting, so the authoritative list is the console itself rather than this file.

**Interaction:** type to filter by setting name *or* group name (typing `audio` surfaces the whole Audio group). Up/Down navigate, Enter selects and edits. Tab jumps straight to editing when exactly one setting matches. For bool/categorical settings, Left/Right cycle values. For float settings, type a value — the field is pre-filled with the current one. Enter applies and closes; an invalid or out-of-range value flashes red and stays open. Backtick or Esc cancels.

**Careful:** Esc only cancels while the console is open. With it closed, Esc quits the application.

A few settings are *actions* rather than state — set them to `on` to fire once (`bake_probes`, `classify_probes`, `dump_nearest_door`, `toggle_platforms`). Several are read-only reporters (`probe_count`, `refl_tri_count`, `refl_sample_rate`, `refl_ambi_order`).

Commonly used settings:

| Setting | Group | Type | Description |
|---------|-------|------|-------------|
| `texture_filter` | Rendering | point / bilinear / trilinear / anisotropic | Texture filtering mode |
| `lightmap_filter` | Rendering | bilinear / bicubic | Lightmap filter quality |
| `portal_culling` | Rendering | on / off | Portal/frustum culling |
| `show_objects` | Rendering | on / off | Render object meshes |
| `show_rooms` / `show_portals` | Rendering | on / off | Room / portal debug overlays |
| `isolate_model` | Rendering | all / (model names) | Isolate one object model for inspection |
| `physics_mode` | Movement | on / off | Physics (walk) vs fly (noclip) |
| `physics_rate` | Movement | 12.5Hz / 60Hz / 120Hz | Physics timestep |
| `move_speed` | Movement | 1.0 – 500.0 | Camera movement speed (fly mode) |
| `refl_enabled` | Audio | on / off | Real-time reflection reverb |
| `reverb_voices` | Audio | 1 – 32 | Max concurrent realtime reverb voices |
| `refl_rays` | Audio | 128 – 8192 | Rays per reflection sim step |
| `master_gain` | Audio | 0.0 – 4.0 | Master output gain |
| `show_pathing_graph` | Audio | on / off | Overlay the pathing-probe visibility graph |
| `bake_probes` | Audio | *(action)* | Re-bake acoustic probes |
| `record_audio` | Audio | on / off | Start/stop audio recording to file |
| `frob_distance` | Interaction | 1.0 – 30.0 | Frob reach |

## CLI reference

The CLI is deliberately narrow for normal use — tunables live in the YAML config and the debug console. The rest of the flags exist for automated performance runs.

| Flag | Default | Description |
|------|---------|-------------|
| `<mission.mis>` | *(required)* | Path to mission file, positional |
| `--res <path>` | `paths.res` | RES directory. Must resolve from here or YAML |
| `--schemas <path>` | `paths.schemas` | Schema directory. Falls back to `<res>/../EDITOR/SCHEMA` |
| `--config <path>` | `./darknessRender.yaml` | YAML config path |
| `--set <yaml.path>=<value>` | | Override any `audio.*` key. Repeatable |
| `--force-pathing-bake` | off | Re-bake the pathing probes even when a valid `.probes` exists; reflections carry forward |
| `--bake-quality <dev\|ship>` | *(YAML)* | Probe bake quality preset |
| `--audio-log` | off | Enable audio diagnostics |
| `--help` / `-h` | | Full help, including every YAML key. **Prints to stderr** |

Automation / profiling flags, documented in `--help`: `--perf-label`, `--exit-after-seconds`, `--audio-rng-seed`, `--capture-wav`, the `--auto-fly*` family (5 flags), the `--auto-run*` family (4), and the `--audio-capture*` family (3). `--stress-doors`, `--stress-door-ids`, and `--spawn-override` are dev-only.

Unknown flags print a warning and are ignored — they do not fail the run.

## Mission inspector (`darknessHeadless`)

```bash
darknessHeadless <database> [command] [args] [--scripts <path>]
```

Loads a `.mis`, `.gam`, or `.sav` database and inspects its contents. The command defaults to `info`. There is **no `--help`** — run it bare, or with an unknown command, to get usage. All diagnostics go to stderr so stdout stays pipeable.

**Database inspection**

| Command | Description |
|---------|-------------|
| `info` | File type, parent DB chain, chunk count *(default)* |
| `chunks` | List all chunks with name, version, size |
| `objects` | List all objects with ID, name, position |
| `properties [id]` | All property types, or a dump for one object |
| `links [id]` | All relation types, or a dump for one object |
| `prop-dump <name>` | Dump one named property across the mission |

**Geometry & rooms**

| Command | Description |
|---------|-------------|
| `room-info <id> [<id> ...]` | Detail for specific rooms |
| `room_graph` | Room/portal graph as `ROOM` / `PORTAL` / `PDIST` lines |
| `wr_cells` | Cell geometry as `WRCELL` / `WRVERT` / `WRPOLY` lines |
| `wr_portals` | Portal geometry as `WRPORTAL` lines |
| `trace-path <src> <dst> [maxDist]` | Per-hop sound-propagation BFS trace. `maxDist` defaults to 200 |
| `mesh_validate [--at X Y Z R]` | Validate acoustic geometry; writes `<mis>.acoustic.obj` |

**Audio data**

| Command | Description |
|---------|-------------|
| `sound-chunks` / `sound-desc` | Sound chunk listings |
| `ambients` | Ambient sound records |
| `sound_db [outpath]` | CSV of schema gain / attenuation / class. Needs no mission |
| `txlist_audit` | Texture-name → acoustic-material coverage audit |
| `probe_plan` | Dry-run probe placement; counts and histograms, no `.probes` written |
| `probe_cell_audit [positions.csv]` | Probe-vs-cell audit |

**Flags:** `--scripts <path>` (default `scripts/thief2`) selects the property/link schema directory; `scripts/thief1` and `scripts/shock2` also exist. `--res` / `--schemas` are used by `probe_plan` and `sound_db`. `--set` is accepted but resolves a much narrower key set than the renderer's.

## Development

Layout:

- `src/base/` — foundation: math, file I/O, logging, service manager, threading
- `src/services/` — the service stack and every game subsystem
- `src/main/` — the two binaries plus header-only renderer modules and data parsers
- `src/scripts/` — reimplemented object scripts
- `shaders/` — shader sources, compiled into `src/shaders/embedded_shaders.h`
- `tests/` — Catch2 suite
- `tools/` — performance sweep and audio-artifact analysis scripts

The engine is a service stack (`ServiceManager` singletons: database, property, link, object, inherit, physics, room, audio, sim, loop) driven by a frame loop, with the renderer built as header-only modules on top. Subsystems talk through interfaces — `IWorldQuery` for read-only world access, `IPhysicsWorld` for collision — rather than reaching into each other, so they can be replaced independently. As much as possible, boundary discipline motivates the design.

## Thanks

* The openDarkEngine team.
* TomNHarris (telliamed) - for the all the work he has done understanding the Dark Engine and its data formats. Also for the irreplaceable help in the past.
* ShadowSpawn - For the BIN mesh format and Movement database format descriptions.
* ataricom - For helping out with the (now defunct) sourceforge Wiki
* The wonderfully obsessive TTLG community
* Conor Armstrong (SilentSleep) for Equilibrium, our test fixture (and great FM!)
* ...and others not mentioned

# Disclaimer
`darkness` is a reimplementation of the Dark Engine, drawing heavily from `opde`, and is not affiliated with or endorsed by the original software's rights holders or the `opde` team.

This project does not contain, distribute, or incorporate proprietary source code, binaries, assets, or data files from the original software. All code in this repository has been written by the project's contributors through independent development efforts. Users must supply their own legally obtained copies of any original game data files required to operate this software. This project does not facilitate, encourage, or provide access to pirated or unauthorized copies of any proprietary material.

### Intellectual Property
All original code in this repository is released under the GPLv3 license. See LICENSE for full terms.

The Dark Engine and any associated trademarks are the property of their respective owners. The use of these names within this project is solely for purposes of identification and interoperability, and does not imply any claim of ownership or affiliation.

### Purpose
This project exists to preserve access to The Dark Engine-based games on modern hardware and operating systems, in a context where the original software is no longer maintained or supported by its rights holders. It also aims to provide the opportunity for the fan community to extend the core functionality of the game to take advantage of modern hardware for non commercial purposes only.

### Good Faith
This project is developed and distributed in good faith, on a non-commercial basis, by volunteer contributors. Should any rights holder have concerns regarding this project, we welcome direct communication at [joseph at viviano dot ca] and are committed to addressing any legitimate concerns promptly and in good faith.
