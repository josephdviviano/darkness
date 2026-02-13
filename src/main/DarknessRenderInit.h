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

// DarknessRenderInit.h — One-time init and cleanup functions

#pragma once

// ── Initialize service stack and load mission database ──
// Creates the ServiceManager with all 12 services, registers property/relation
// schemas from PLDef/DType files, loads the mission database (.gam + .mis),
// constructs the IWorldQuery facade, and runs verification diagnostics.
// Logger objects must be constructed before calling this (they live in main).
static std::unique_ptr<Darkness::ObjSysWorldState> initServiceStack(
    const char *misPath, const std::string &scriptsDir)
{
    // Create service manager with all 12 services
    {
        auto *svcMgr = new Darkness::ServiceManager(SERVICE_ALL);

        svcMgr->registerFactory<Darkness::PlatformServiceFactory>();
        svcMgr->registerFactory<Darkness::ConfigServiceFactory>();
        svcMgr->registerFactory<Darkness::DatabaseServiceFactory>();
        svcMgr->registerFactory<Darkness::GameServiceFactory>();
        svcMgr->registerFactory<Darkness::InheritServiceFactory>();
        svcMgr->registerFactory<Darkness::LinkServiceFactory>();
        svcMgr->registerFactory<Darkness::LoopServiceFactory>();
        svcMgr->registerFactory<Darkness::ObjectServiceFactory>();
        svcMgr->registerFactory<Darkness::PropertyServiceFactory>();
        svcMgr->registerFactory<Darkness::RoomServiceFactory>();
        svcMgr->registerFactory<Darkness::SimServiceFactory>();
        svcMgr->registerFactory<Darkness::PhysicsServiceFactory>();

        svcMgr->bootstrapFinished();
    }

    // Load schema definitions (property types, link relations)
    {
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        Darkness::LinkServicePtr linkSvc = GET_SERVICE(Darkness::LinkService);

        Darkness::PLDefResult propDefs = Darkness::parsePLDef(scriptsDir + "/t2-props.pldef");
        Darkness::PLDefResult linkDefs = Darkness::parsePLDef(scriptsDir + "/t2-links.pldef");
        auto dtypeSizes = Darkness::parseDTypeSizes(scriptsDir + "/t2-types.dtype");

        int propCount = 0;
        for (const auto &pd : propDefs.properties) {
            if (propSvc->getProperty(pd.name))
                continue;
            Darkness::DataStoragePtr storage;
            if (pd.isVarStr) {
                storage = Darkness::DataStoragePtr(new Darkness::StringDataStorage());
            } else {
                storage = Darkness::DataStoragePtr(new Darkness::RawDataStorage());
            }
            Darkness::Property *prop = propSvc->createProperty(
                pd.name, pd.name, pd.inheritor, storage);
            if (prop) {
                prop->setChunkVersions(pd.verMaj, pd.verMin);
                ++propCount;
            }
        }

        int relCount = 0;
        for (const auto &rd : linkDefs.relations) {
            if (linkSvc->getRelation(rd.name))
                continue;
            Darkness::DataStoragePtr storage;
            if (!rd.noData) {
                auto it = dtypeSizes.find(rd.name);
                size_t dataSize = (it != dtypeSizes.end()) ? it->second : 0;
                storage = Darkness::DataStoragePtr(new Darkness::RawDataStorage(dataSize));
            }
            Darkness::RelationPtr rel = linkSvc->createRelation(rd.name, storage, rd.hidden);
            if (rel) {
                rel->setChunkVersions(rd.lVerMaj, rd.lVerMin,
                                      rd.dVerMaj, rd.dVerMin);
                if (rd.fakeSize >= 0)
                    rel->setFakeSize(rd.fakeSize);
                ++relCount;
            }
        }

        // Register rendering properties that are commented out in t2-props.pldef
        // (normally created by RenderService, which the standalone viewer doesn't use).
        // These must exist before database loading so their P$ chunks get read.
        auto registerRawProp = [&](const char *name, const char *chunk,
                                   const char *inheritor) {
            if (!propSvc->getProperty(name)) {
                propSvc->createProperty(name, chunk, inheritor,
                    Darkness::DataStoragePtr(new Darkness::RawDataStorage()));
                ++propCount;
            }
        };
        registerRawProp("ModelName",   "ModelName", "always");
        registerRawProp("RenderType",  "RenderTyp", "always");
        registerRawProp("RenderAlpha", "RenderAlp", "always");
        // Scale uses "never" inheritor — kPropertyNoInherit in the original engine.
        // Archetype scales are physics bounding boxes, not visual model scales.
        registerRawProp("ModelScale",  "Scale",     "never");

        std::fprintf(stderr, "Schema: registered %d properties, %d relations\n",
                     propCount, relCount);
    }

    // Load the mission database (.gam + .mis) via service stack.
    // GameService::load() recursively loads the parent .gam first, then the .mis.
    // This populates PropertyService with all P$ data and LinkService with all L$ data.
    {
        Darkness::GameServicePtr gameSvc = GET_SERVICE(Darkness::GameService);
        gameSvc->load(misPath);
    }

    // Construct IWorldQuery facade — the read-only interface for downstream subsystems.
    // Currently used for verification only; will be passed to AI, audio, scripts.
    Darkness::ObjectServicePtr objSvcPtr = GET_SERVICE(Darkness::ObjectService);
    Darkness::PropertyServicePtr propSvcPtr = GET_SERVICE(Darkness::PropertyService);
    Darkness::LinkServicePtr linkSvcPtr = GET_SERVICE(Darkness::LinkService);
    Darkness::RoomServicePtr roomSvcPtr = GET_SERVICE(Darkness::RoomService);

    auto worldQuery = std::make_unique<Darkness::ObjSysWorldState>(
        objSvcPtr.get(), propSvcPtr.get(), linkSvcPtr.get(), roomSvcPtr.get());

    // Verification: exercise IWorldQuery methods to ensure correctness
    {
        int entityCount = 0, positionedCount = 0, roomCount = 0, linkCount = 0;

        // Count positioned entities via property access
        auto positionedIDs = worldQuery->getAllWithProperty("Position");
        for (auto eid : positionedIDs) {
            if (!worldQuery->exists(eid))
                continue;
            ++entityCount;

            Darkness::Vector3 pos = worldQuery->getPosition(eid);
            // Verify consistency: getPosition matches getProperty<T> raw data
            if (pos.x != 0.0f || pos.y != 0.0f || pos.z != 0.0f)
                ++positionedCount;
        }

        // Count rooms and their portals
        if (roomSvcPtr->isLoaded()) {
            const auto &rooms = roomSvcPtr->getAllRooms();
            for (const auto &r : rooms) {
                if (!r)
                    continue;
                ++roomCount;
            }
        }

        // Count links via MetaProp relation — test back links (incoming to -1).
        // Archetype -1 is the root; MetaProp links go TO it from other archetypes.
        // Uses string-based getBackLinks which resolves ~MetaProp internally.
        {
            auto backLinks = worldQuery->getBackLinks(-1, "MetaProp", 0);
            linkCount = static_cast<int>(backLinks.size());
        }

        // Test property handle caching (hot-path variant)
        auto posHandle = worldQuery->resolveProperty("Position");
        int handleHits = 0;
        if (posHandle) {
            for (auto eid : positionedIDs) {
                if (worldQuery->hasProperty(eid, posHandle))
                    ++handleHits;
            }
        }

        std::fprintf(stderr,
                     "IWorldQuery: verified %d entities (%d positioned), "
                     "%d rooms, %d MetaProp backlinks to -1, "
                     "handle cache: %d/%zu hits\n",
                     entityCount, positionedCount, roomCount, linkCount,
                     handleHits, positionedIDs.size());
    }

    return worldQuery;
}

// ── Load mission data from .mis file ──
// Parses WR geometry, portal graph, spawn point, animated lights, sky/fog/flow
// parameters, and extracts mission base name. Returns false if WR parse fails.
static bool loadMissionData(const char *misPath, bool forceFlicker,
                            Darkness::MissionData &mission)
{
    // Extract mission base name (e.g. "miss6" from "path/to/miss6.mis")
    // for constructing skybox texture filenames like "skyhw/miss6n.PCX"
    {
        std::string p(misPath);
        size_t slash = p.find_last_of("/\\");
        std::string base = (slash != std::string::npos) ? p.substr(slash + 1) : p;
        size_t dot = base.find('.');
        mission.missionName = (dot != std::string::npos) ? base.substr(0, dot) : base;
        // Lowercase for case-insensitive CRF matching
        std::transform(mission.missionName.begin(), mission.missionName.end(),
                       mission.missionName.begin(),
                       [](unsigned char c) { return std::tolower(c); });
    }

    // Parse WR geometry
    std::fprintf(stderr, "Loading WR geometry from %s...\n", misPath);
    try {
        mission.wrData = Darkness::parseWRChunk(misPath);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "Failed to parse WR chunk: %s\n", e.what());
        return false;
    }
    std::fprintf(stderr, "Loaded %u cells\n", mission.wrData.numCells);

    // Build portal adjacency graph for portal culling
    mission.cellPortals = buildPortalGraph(mission.wrData);
    {
        int totalPortals = 0;
        for (const auto &pl : mission.cellPortals)
            totalPortals += static_cast<int>(pl.size());
        std::fprintf(stderr, "Portal graph: %d portals across %u cells\n",
                     totalPortals, mission.wrData.numCells);
    }

    // Find player spawn point from L$PlayerFactory + P$Position chunks
    mission.spawnInfo = Darkness::findSpawnPoint(misPath);

    // Parse animated light properties from mission database
    mission.lightSources = Darkness::parseAnimLightProperties(misPath);

    // Build reverse index: lightnum → list of (cellIdx, polyIdx) affected
    mission.animLightIndex = Darkness::buildAnimLightIndex(mission.wrData);

    // Ensure all lightnums referenced in WR data have a LightSource entry.
    // Lights without P$AnimLight properties default to mode 4 (max brightness).
    for (const auto &kv : mission.animLightIndex) {
        if (mission.lightSources.find(kv.first) == mission.lightSources.end()) {
            Darkness::LightSource ls = {};
            ls.lightNum = kv.first;
            ls.mode = Darkness::ANIM_MAX_BRIGHT;
            ls.maxBright = 1.0f;
            ls.minBright = 0.0f;
            ls.brightness = 1.0f;
            ls.prevIntensity = 1.0f;
            mission.lightSources[kv.first] = ls;
        }
    }

    // Apply --force-flicker: override all lights to flicker mode for debugging
    if (forceFlicker) {
        for (auto &[num, ls] : mission.lightSources) {
            ls.mode = Darkness::ANIM_FLICKER;
            ls.inactive = false;
            ls.minBright = 0.0f;
            ls.maxBright = 1.0f;
            ls.brightenTime = 0.15f;
            ls.dimTime = 0.15f;
            ls.brightness = ls.maxBright;
            ls.countdown = 0.1f;
            ls.isRising = false;
        }
        std::fprintf(stderr, "Force-flicker: all %zu lights set to flicker mode\n",
                     mission.lightSources.size());
    }

    // Animated light diagnostics
    {
        int modeCounts[10] = {};
        int inactiveCount = 0;
        int fromMIS = 0, fromDefault = 0;
        for (const auto &[num, ls] : mission.lightSources) {
            if (ls.mode < 10) modeCounts[ls.mode]++;
            if (ls.inactive) inactiveCount++;
            if (ls.objectId != 0) fromMIS++; else fromDefault++;
        }
        std::fprintf(stderr, "Animated lights: %zu sources (%d from MIS, %d defaulted), "
                     "%zu indexed lightnums\n",
                     mission.lightSources.size(), fromMIS, fromDefault,
                     mission.animLightIndex.size());
        const char *modeNames[] = {
            "flip", "smooth", "random", "min_bright", "max_bright",
            "zero", "brighten", "dim", "semi_random", "flicker"
        };
        for (int m = 0; m < 10; ++m) {
            if (modeCounts[m] > 0)
                std::fprintf(stderr, "  mode %d (%s): %d lights\n",
                             m, modeNames[m], modeCounts[m]);
        }
        if (inactiveCount > 0)
            std::fprintf(stderr, "  inactive: %d lights\n", inactiveCount);
    }

    // Parse sky dome parameters from SKYOBJVAR chunk (if present)
    mission.skyParams = parseSkyObjVar(misPath);
    mission.skyDome = buildSkyDome(mission.skyParams);

    // Parse global fog parameters from FOG chunk (if present)
    mission.fogParams = parseFogChunk(misPath);

    // Parse water flow data — FLOW_TEX (texture mapping) and CELL_MOTION (animation state)
    mission.flowData = parseFlowData(misPath);

    return true;
}

// ── Load world textures from CRF archives ──
// Parses TXLIST, loads world textures from fam.crf, water flow textures,
// and skybox face textures. All images stored in mission for later GPU upload.
static void loadWorldTextures(const char *misPath, const std::string &resPath,
                              Darkness::MissionData &mission)
{
    // Parse TXLIST if in textured mode
    if (mission.texturedMode) {
        try {
            mission.txList = Darkness::parseTXList(misPath);
            std::fprintf(stderr, "TXLIST: %zu textures, %zu families\n",
                         mission.txList.textures.size(), mission.txList.families.size());
        } catch (const std::exception &e) {
            std::fprintf(stderr, "Failed to parse TXLIST: %s (falling back to flat)\n",
                         e.what());
            mission.texturedMode = false;
        }
    }

    // Collect unique texture indices used by world geometry
    std::unordered_set<uint8_t> usedTextures;
    if (mission.texturedMode) {
        for (const auto &cell : mission.wrData.cells) {
            for (int pi = 0; pi < cell.numTextured; ++pi) {
                uint8_t txt = cell.texturing[pi].txt;
                if (txt != 0 && txt != 249)
                    usedTextures.insert(txt);
            }
        }
        // Note: FLOW_TEX texture indices are runtime palette positions, NOT TXLIST
        // indices. Water textures are loaded separately by name from fam.crf below.
        std::fprintf(stderr, "Unique texture indices used: %zu\n", usedTextures.size());
    }

    // Load world textures from fam.crf (indexed by TXLIST)
    if (mission.texturedMode) {
        Darkness::CRFTextureLoader loader(resPath);
        if (!loader.isOpen()) {
            std::fprintf(stderr, "CRF not available, falling back to flat shading\n");
            mission.texturedMode = false;
        } else {
            int loaded = 0;
            for (uint8_t idx : usedTextures) {
                if (idx >= mission.txList.textures.size()) continue;
                const auto &entry = mission.txList.textures[idx];
                auto img = loader.loadTexture(entry.family, entry.name);
                mission.texDims[idx] = { img.width, img.height };
                mission.loadedTextures[idx] = std::move(img);
                ++loaded;
            }
            std::fprintf(stderr, "Loaded %d/%zu textures from CRF\n",
                         loaded, usedTextures.size());
        }
    }

    // Load water flow textures from fam.crf by name.
    // FLOW_TEX name field (e.g. "gr") maps to "water/<name>in.PCX" for the
    // air-side texture and "water/<name>out.PCX" for the underwater side.
    // Keyed by flow group index (1-255).
    if (mission.texturedMode && mission.flowData.hasFlowTex) {
        Darkness::CRFTextureLoader loader(resPath);
        if (loader.isOpen()) {
            // Collect unique flow groups used by cells
            std::unordered_set<uint8_t> usedFlowGroups;
            for (const auto &cell : mission.wrData.cells) {
                if (cell.flowGroup > 0)
                    usedFlowGroups.insert(cell.flowGroup);
            }

            int loaded = 0;
            for (uint8_t fg : usedFlowGroups) {
                const auto &fe = mission.flowData.textures[fg];
                if (fe.name[0] == '\0') continue;

                // Extract base name, trimming trailing nulls/spaces
                std::string baseName(fe.name, strnlen(fe.name, 28));
                while (!baseName.empty() && (baseName.back() == ' ' || baseName.back() == '\0'))
                    baseName.pop_back();
                if (baseName.empty()) continue;

                // Air-side texture: "water/<name>in" (e.g. "gr" → "water/grin")
                std::string inName = baseName + "in";
                auto img = loader.loadTexture("water", inName);

                // Check if we got a real texture (not the 8x8 fallback)
                if (img.width > 8 || img.height > 8) {
                    std::fprintf(stderr, "Flow group %d: loaded water/%s.PCX (%ux%u)\n",
                                 fg, inName.c_str(), img.width, img.height);
                    mission.flowTexDims[fg] = { img.width, img.height };
                    mission.flowLoadedTextures[fg] = std::move(img);
                    ++loaded;
                } else {
                    std::fprintf(stderr, "Flow group %d: water/%s.PCX not found, trying waterhw/\n",
                                 fg, inName.c_str());
                    // Some missions may use WATERHW family instead
                    auto img2 = loader.loadTexture("waterhw", inName);
                    if (img2.width > 8 || img2.height > 8) {
                        std::fprintf(stderr, "Flow group %d: loaded waterhw/%s.PCX (%ux%u)\n",
                                     fg, inName.c_str(), img2.width, img2.height);
                        mission.flowTexDims[fg] = { img2.width, img2.height };
                        mission.flowLoadedTextures[fg] = std::move(img2);
                        ++loaded;
                    } else {
                        std::fprintf(stderr, "Flow group %d: no water texture found for '%s'\n",
                                     fg, baseName.c_str());
                    }
                }
            }
            if (loaded > 0) {
                std::fprintf(stderr, "Loaded %d flow group water textures from CRF\n", loaded);
            }
        }
    }

    // Load skybox face textures (old sky system).
    // Missions without SKYOBJVAR use a textured skybox with per-mission PCX
    // textures in fam.crf under skyhw/ (e.g. skyhw/miss6n.PCX for north face).
    if (mission.texturedMode) {
        Darkness::CRFTextureLoader skyLoader(resPath);
        if (skyLoader.isOpen()) {
            // 5 faces: n=north(+Y), s=south(-Y), e=east(+X), w=west(-X), t=top(+Z)
            const char *suffixes[] = { "n", "s", "e", "w", "t" };
            int loaded = 0;
            for (const char *suf : suffixes) {
                std::string texName = mission.missionName + suf;
                auto img = skyLoader.loadTexture("skyhw", texName);
                // Real texture is larger than the 8x8 fallback checkerboard
                if (img.width > 8 || img.height > 8) {
                    mission.skyboxImages[suf] = std::move(img);
                    ++loaded;
                }
            }
            // Skybox available if at least the 4 side faces loaded (top optional)
            mission.hasSkybox = mission.skyboxImages.count("n") && mission.skyboxImages.count("s")
                     && mission.skyboxImages.count("e") && mission.skyboxImages.count("w");
            if (mission.hasSkybox) {
                std::fprintf(stderr, "Skybox: loaded %d/5 faces for %s (textured skybox active)\n",
                             loaded, mission.missionName.c_str());
                for (auto &kv : mission.skyboxImages) {
                    std::fprintf(stderr, "  face '%s': %ux%u\n",
                                 kv.first.c_str(), kv.second.width, kv.second.height);
                }
            } else if (loaded > 0) {
                std::fprintf(stderr, "Skybox: partial load (%d faces), falling back to dome\n", loaded);
            }
        }
    }
}

// ── Load object assets: properties, .bin models, textures from obj.crf ──
// Parses object placements via PropertyService, precomputes per-object cell IDs
// for portal culling, loads .bin meshes and object textures from obj.crf.
static void loadObjectAssets(const char *misPath, const std::string &resPath,
                             const Darkness::RenderConfig &cfg,
                             Darkness::MissionData &mission,
                             Darkness::RuntimeState &state)
{
    // Parse object placements from .mis via PropertyService
    if (state.showObjects) {
        try {
            Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
            mission.objData = Darkness::parseObjectProps(propSvc.get(), misPath,
                                                    cfg.debugObjects);
        } catch (const std::exception &e) {
            std::fprintf(stderr, "Failed to parse object props: %s\n", e.what());
            state.showObjects = false;
        }
        if (mission.objData.objects.empty()) {
            std::fprintf(stderr, "No objects to render\n");
            state.showObjects = false;
        }
    }

    // Precompute which cell each object is in for portal culling.
    // Objects don't move (yet), so this is a one-time lookup at load time.
    // -1 = outside all cells (always rendered to avoid popping).
    if (state.showObjects) {
        mission.objCellIDs.resize(mission.objData.objects.size());
        for (size_t i = 0; i < mission.objData.objects.size(); ++i) {
            const auto &obj = mission.objData.objects[i];
            if (obj.hasPosition) {
                mission.objCellIDs[i] = findCameraCell(mission.wrData, obj.x, obj.y, obj.z);
            } else {
                mission.objCellIDs[i] = -1;
            }
        }
    }

    // Load .bin models from obj.crf (if --res provided and objects enabled)
    if (state.showObjects && !resPath.empty()) {
        Darkness::CRFModelLoader modelLoader(resPath);
        if (modelLoader.isOpen()) {
            int loaded = 0, failed = 0;
            for (const auto &name : mission.objData.uniqueModels) {
                auto binData = modelLoader.loadModel(name);
                if (binData.empty()) {
                    ++failed;
                    continue;
                }
                try {
                    auto mesh = Darkness::parseBinModel(binData.data(), binData.size());
                    if (mesh.valid) {
                        mission.parsedModels[name] = std::move(mesh);
                        ++loaded;
                    } else {
                        // Log first few failures for debugging
                        if (failed < 5) {
                            // Show magic header of failed file
                            char hdr[5] = {};
                            if (binData.size() >= 4)
                                std::memcpy(hdr, binData.data(), 4);
                            std::fprintf(stderr, "  model '%s': parse failed "
                                         "(size=%zu, magic='%s')\n",
                                         name.c_str(), binData.size(), hdr);
                        }
                        ++failed;
                    }
                } catch (const std::exception &e) {
                    // Some .bin files may be AI meshes (LGMM) or corrupt
                    if (failed < 5) {
                        std::fprintf(stderr, "  model '%s': exception: %s\n",
                                     name.c_str(), e.what());
                    }
                    ++failed;
                }
            }
            std::fprintf(stderr, "Loaded %d/%zu models from obj.crf (%d failed)\n",
                         loaded, mission.objData.uniqueModels.size(), failed);
        } else {
            std::fprintf(stderr, "obj.crf not available, using fallback cubes\n");
        }
    }

    // Load object textures from obj.crf (txt16/ and txt/ subdirectories inside it).
    // Dark Engine stores object textures as GIF/PCX files within obj.crf, not in
    // separate txt16.crf/txt.crf archives.

    // Collect unique MD_MAT_TMAP material names from all parsed models
    std::unordered_set<std::string> objMatNames;
    for (const auto &kv : mission.parsedModels) {
        for (const auto &mat : kv.second.materials) {
            if (mat.type == Darkness::MD_MAT_TMAP) {
                // Lowercase the name for case-insensitive matching
                std::string lname(mat.name);
                std::transform(lname.begin(), lname.end(), lname.begin(),
                               [](unsigned char c) { return std::tolower(c); });
                objMatNames.insert(lname);
            }
        }
    }

    if (!objMatNames.empty() && !resPath.empty()) {
        // Reuse obj.crf for texture lookup (same archive that holds .bin models)
        Darkness::CRFTextureLoader objTexLoader(resPath, "obj.crf");

        int loaded = 0;
        for (const auto &name : objMatNames) {
            if (!objTexLoader.isOpen()) break;
            auto img = objTexLoader.loadObjectTexture(name);
            // Check if we got a real texture (not the 8x8 fallback checkerboard)
            if (img.width > 8 || img.height > 8) {
                mission.objTexImages[name] = std::move(img);
                ++loaded;
            }
        }
        std::fprintf(stderr, "Loaded %d/%zu object textures from obj.crf\n",
                     loaded, objMatNames.size());
    }
}

// Initialize SDL2 window and bgfx rendering context.
// Sets up 3 views: sky (0), world+objects (1), debug overlay (2).
// Returns the SDL window, or nullptr on failure.
static SDL_Window *initWindow(const Darkness::FogParams &fogParams,
                               uint32_t &outSkyClearColor) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return nullptr;
    }

    SDL_Window *window = SDL_CreateWindow(
        "darkness — lightmapped renderer",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH, WINDOW_HEIGHT,
        SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI
    );

    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return nullptr;
    }

    SDL_SysWMinfo wmi;
    SDL_VERSION(&wmi.version);
    if (!SDL_GetWindowWMInfo(window, &wmi)) {
        std::fprintf(stderr, "SDL_GetWindowWMInfo failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return nullptr;
    }

    bgfx::renderFrame(); // single-threaded mode

    bgfx::Init bInit;
    bInit.type = bgfx::RendererType::Count; // auto-detect: Metal/D3D11/OpenGL/Vulkan
    bInit.resolution.width  = WINDOW_WIDTH;
    bInit.resolution.height = WINDOW_HEIGHT;
    bInit.resolution.reset  = BGFX_RESET_VSYNC;

#if BX_PLATFORM_OSX
    bInit.platformData.nwh = wmi.info.cocoa.window;
#elif BX_PLATFORM_LINUX
    bInit.platformData.ndt = wmi.info.x11.display;
    bInit.platformData.nwh = (void *)(uintptr_t)wmi.info.x11.window;
#elif BX_PLATFORM_WINDOWS
    bInit.platformData.nwh = wmi.info.win.window;
#endif

    if (!bgfx::init(bInit)) {
        std::fprintf(stderr, "bgfx::init failed\n");
        SDL_DestroyWindow(window);
        SDL_Quit();
        return nullptr;
    }

    // View 0: Sky pass — clears colour + depth, renders sky dome with no depth writes
    // When fog is enabled, use fog colour as clear colour so uncovered sky matches
    outSkyClearColor = 0x1a1a2eFF;
    if (fogParams.enabled) {
        uint8_t fr = static_cast<uint8_t>(fogParams.r * 255.0f);
        uint8_t fg = static_cast<uint8_t>(fogParams.g * 255.0f);
        uint8_t fb = static_cast<uint8_t>(fogParams.b * 255.0f);
        outSkyClearColor = (uint32_t(fr) << 24) | (uint32_t(fg) << 16) | (uint32_t(fb) << 8) | 0xFF;
    }
    bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                        outSkyClearColor, 1.0f, 0);
    bgfx::setViewRect(0, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    // View 1: World + objects pass — clears depth only, preserves sky colour
    bgfx::setViewClear(1, BGFX_CLEAR_DEPTH, 0, 1.0f, 0);
    bgfx::setViewRect(1, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    // View 2: Debug overlay — no clear, renders on top of view 1.
    // Separate view ensures debug lines are drawn AFTER all world geometry,
    // regardless of bgfx's internal draw call sorting within a view.
    bgfx::setViewClear(2, BGFX_CLEAR_NONE, 0, 1.0f, 0);
    bgfx::setViewRect(2, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    Darkness::PosColorVertex::init();
    Darkness::PosColorUVVertex::init();
    Darkness::PosUV2Vertex::init();

    return window;
}

// ── Create all GPU resources: shaders, lightmap atlas, world/object/sky buffers ──
// Builds mesh data from MissionData, uploads to bgfx, creates shader programs
// and uniform handles. Returns false if world geometry is empty (fatal).
// centroidX/Y/Z receive the world geometry centroid for camera init.
static bool createGPUResources(const Darkness::MissionData &mission,
                               const Darkness::RenderConfig &cfg,
                               bool showObjects,
                               Darkness::BuiltMeshes &meshes,
                               Darkness::GPUResources &gpu,
                               float &centroidX, float &centroidY, float &centroidZ)
{
    bool linearMips = cfg.linearMips;
    bool sharpMips = cfg.sharpMips;

    // ── Shaders ──
    // Load from cross-platform embedded shader table (auto-selects Metal/D3D/GL/Vulkan)
    bgfx::RendererType::Enum rendererType = bgfx::getRendererType();

    // Flat-color program
    gpu.flatProgram = bgfx::createProgram(
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "vs_basic"),
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "fs_basic"),
        true);

    // Textured program
    gpu.texturedProgram = bgfx::createProgram(
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "vs_textured"),
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "fs_textured"),
        true);

    // Lightmapped program (bilinear — hardware LINEAR filtering, the default)
    gpu.lightmappedProgram = bgfx::createProgram(
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "vs_lightmapped"),
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "fs_lightmapped"),
        true);

    // Lightmapped bicubic program (cubic B-spline filtering via 4 bilinear taps)
    gpu.lightmappedBicubicProgram = bgfx::createProgram(
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "vs_lightmapped_bicubic"),
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "fs_lightmapped_bicubic"),
        true);
    if (!bgfx::isValid(gpu.lightmappedBicubicProgram)) {
        std::fprintf(stderr, "Warning: bicubic lightmap shader failed to compile, falling back to bilinear\n");
    }

    // Water program: vertex displacement + textured fragment with UV distortion
    gpu.waterProgram = bgfx::createProgram(
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "vs_water"),
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "fs_water"),
        true);

    gpu.s_texColor = bgfx::createUniform("s_texColor", bgfx::UniformType::Sampler);
    gpu.s_texLightmap = bgfx::createUniform("s_texLightmap", bgfx::UniformType::Sampler);
    gpu.u_waterParams = bgfx::createUniform("u_waterParams", bgfx::UniformType::Vec4);
    gpu.u_waterFlow = bgfx::createUniform("u_waterFlow", bgfx::UniformType::Vec4);
    gpu.u_fogColor = bgfx::createUniform("u_fogColor", bgfx::UniformType::Vec4);
    gpu.u_fogParams = bgfx::createUniform("u_fogParams", bgfx::UniformType::Vec4);
    // Per-object params: x = alpha (1.0 = opaque, < 1.0 = translucent via RenderAlpha property)
    gpu.u_objectParams = bgfx::createUniform("u_objectParams", bgfx::UniformType::Vec4);
    // Atlas dimensions for bicubic shader texel-space calculations
    gpu.u_lmAtlasSize = bgfx::createUniform("u_lmAtlasSize", bgfx::UniformType::Vec4);

    // ── Build lightmap atlas (if textured mode) ──

    if (mission.texturedMode) {
        gpu.lmAtlasSet = Darkness::buildLightmapAtlases(mission.wrData);
        if (!gpu.lmAtlasSet.atlases.empty()) {
            meshes.lightmappedMode = true;

            // Initial blend pass: apply animated overlays at initial intensities.
            // buildLightmapAtlases() only wrote static lightmaps into the atlas.
            // We must blend in overlay contributions so lights at max brightness
            // (mode 4, the default) have correct initial appearance.
            if (!mission.animLightIndex.empty()) {
                // Compute initial intensities for all lights
                std::unordered_map<int16_t, float> initIntensities;
                for (const auto &[lightNum, light] : mission.lightSources) {
                    initIntensities[lightNum] = (light.maxBright > 0.0f)
                        ? light.brightness / light.maxBright : 0.0f;
                }

                // Collect unique (cell, poly) pairs — a polygon may appear under
                // multiple lightnums, but blendAnimatedLightmap re-blends all
                // overlays at once, so we only need to call it once per polygon.
                std::unordered_set<uint64_t> blendedSet;
                int blendedPolys = 0;
                for (const auto &[lightNum, polys] : mission.animLightIndex) {
                    for (const auto &[ci, pi] : polys) {
                        uint64_t key = (static_cast<uint64_t>(ci) << 32)
                                      | static_cast<uint32_t>(pi);
                        if (!blendedSet.insert(key).second) continue;

                        Darkness::blendAnimatedLightmap(
                            gpu.lmAtlasSet.atlases[0], mission.wrData, ci, pi,
                            gpu.lmAtlasSet.entries[ci][pi],
                            initIntensities);
                        ++blendedPolys;
                    }
                }
                std::fprintf(stderr, "Initial lightmap blend: %d polygons\n",
                             blendedPolys);
            }

            for (const auto &atlas : gpu.lmAtlasSet.atlases) {
                // Create texture WITHOUT initial data so it stays mutable —
                // bgfx treats textures with initial mem as immutable.
                // We upload via updateTexture2D immediately after creation.
                // LINEAR filtering matches the original Dark Engine's hardware path.
                // Atlas has 2px edge-clamped padding to prevent seam bleeding.
                bgfx::TextureHandle th = bgfx::createTexture2D(
                    static_cast<uint16_t>(atlas.size),
                    static_cast<uint16_t>(atlas.size),
                    false, 1, bgfx::TextureFormat::RGBA8,
                    BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP);
                // Upload initial atlas data
                const bgfx::Memory *mem = bgfx::copy(atlas.rgba.data(),
                    static_cast<uint32_t>(atlas.rgba.size()));
                bgfx::updateTexture2D(th, 0, 0, 0, 0,
                    static_cast<uint16_t>(atlas.size),
                    static_cast<uint16_t>(atlas.size), mem);
                gpu.lightmapAtlasHandles.push_back(th);
            }
            std::fprintf(stderr, "Created %zu lightmap atlas GPU texture(s)\n",
                         gpu.lightmapAtlasHandles.size());
        }
    }

    // ── Build geometry and create GPU buffers ──

    if (meshes.lightmappedMode) {
        meshes.lmMesh = buildLightmappedMesh(mission.wrData, mission.texDims, gpu.lmAtlasSet);
        centroidX = meshes.lmMesh.cx; centroidY = meshes.lmMesh.cy; centroidZ = meshes.lmMesh.cz;

        std::fprintf(stderr, "Geometry (lightmapped): %zu vertices, %zu indices (%zu triangles), %zu texture groups\n",
                     meshes.lmMesh.vertices.size(), meshes.lmMesh.indices.size(),
                     meshes.lmMesh.indices.size() / 3, meshes.lmMesh.groups.size());

        if (meshes.lmMesh.vertices.empty()) {
            std::fprintf(stderr, "No geometry to render\n");
            return false;
        }

        const bgfx::Memory *vbMem = bgfx::copy(
            meshes.lmMesh.vertices.data(),
            static_cast<uint32_t>(meshes.lmMesh.vertices.size() * sizeof(PosUV2Vertex))
        );
        gpu.vbh = bgfx::createVertexBuffer(vbMem, PosUV2Vertex::layout);

        const bgfx::Memory *ibMem = bgfx::copy(
            meshes.lmMesh.indices.data(),
            static_cast<uint32_t>(meshes.lmMesh.indices.size() * sizeof(uint32_t))
        );
        gpu.ibh = bgfx::createIndexBuffer(ibMem, BGFX_BUFFER_INDEX32);

        // Create bgfx textures with full mip chains from loaded images
        for (const auto &kv : mission.loadedTextures) {
            uint8_t idx = kv.first;
            const auto &img = kv.second;
            gpu.textureHandles[idx] = createMipmappedTexture(
                img.rgba.data(), img.width, img.height,
                BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT
                | BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR,
                MipAlphaMode::ALPHA_TEST, linearMips, sharpMips);
        }
        std::fprintf(stderr, "Created %zu GPU textures (mipmapped)\n", gpu.textureHandles.size());

        // Create GPU textures for flow group water textures (loaded by name)
        // Water textures are fully opaque (alpha blending via vertex color),
        // so use ALPHA_BLEND (no coverage preservation needed).
        for (const auto &kv : mission.flowLoadedTextures) {
            uint8_t fg = kv.first;
            const auto &img = kv.second;
            gpu.flowTextureHandles[fg] = createMipmappedTexture(
                img.rgba.data(), img.width, img.height,
                BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR,
                MipAlphaMode::ALPHA_BLEND, linearMips, sharpMips);
        }
        if (!gpu.flowTextureHandles.empty()) {
            std::fprintf(stderr, "Created %zu flow water GPU textures\n", gpu.flowTextureHandles.size());
        }
    } else if (mission.texturedMode) {
        meshes.worldMesh = buildTexturedMesh(mission.wrData, mission.texDims);
        centroidX = meshes.worldMesh.cx; centroidY = meshes.worldMesh.cy; centroidZ = meshes.worldMesh.cz;

        std::fprintf(stderr, "Geometry (textured): %zu vertices, %zu indices (%zu triangles), %zu texture groups\n",
                     meshes.worldMesh.vertices.size(), meshes.worldMesh.indices.size(),
                     meshes.worldMesh.indices.size() / 3, meshes.worldMesh.groups.size());

        if (meshes.worldMesh.vertices.empty()) {
            std::fprintf(stderr, "No geometry to render\n");
            return false;
        }

        const bgfx::Memory *vbMem = bgfx::copy(
            meshes.worldMesh.vertices.data(),
            static_cast<uint32_t>(meshes.worldMesh.vertices.size() * sizeof(PosColorUVVertex))
        );
        gpu.vbh = bgfx::createVertexBuffer(vbMem, PosColorUVVertex::layout);

        const bgfx::Memory *ibMem = bgfx::copy(
            meshes.worldMesh.indices.data(),
            static_cast<uint32_t>(meshes.worldMesh.indices.size() * sizeof(uint32_t))
        );
        gpu.ibh = bgfx::createIndexBuffer(ibMem, BGFX_BUFFER_INDEX32);

        for (const auto &kv : mission.loadedTextures) {
            uint8_t idx = kv.first;
            const auto &img = kv.second;
            gpu.textureHandles[idx] = createMipmappedTexture(
                img.rgba.data(), img.width, img.height,
                BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT
                | BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR,
                MipAlphaMode::ALPHA_TEST, linearMips, sharpMips);
        }
        std::fprintf(stderr, "Created %zu GPU textures (mipmapped)\n", gpu.textureHandles.size());
    } else {
        meshes.flatMesh = buildFlatMesh(mission.wrData);
        centroidX = meshes.flatMesh.cx; centroidY = meshes.flatMesh.cy; centroidZ = meshes.flatMesh.cz;

        std::fprintf(stderr, "Geometry (flat): %zu vertices, %zu indices (%zu triangles)\n",
                     meshes.flatMesh.vertices.size(), meshes.flatMesh.indices.size(),
                     meshes.flatMesh.indices.size() / 3);

        if (meshes.flatMesh.vertices.empty()) {
            std::fprintf(stderr, "No geometry to render\n");
            return false;
        }

        const bgfx::Memory *vbMem = bgfx::copy(
            meshes.flatMesh.vertices.data(),
            static_cast<uint32_t>(meshes.flatMesh.vertices.size() * sizeof(PosColorVertex))
        );
        gpu.vbh = bgfx::createVertexBuffer(vbMem, PosColorVertex::layout);

        const bgfx::Memory *ibMem = bgfx::copy(
            meshes.flatMesh.indices.data(),
            static_cast<uint32_t>(meshes.flatMesh.indices.size() * sizeof(uint32_t))
        );
        gpu.ibh = bgfx::createIndexBuffer(ibMem, BGFX_BUFFER_INDEX32);
    }

    // ── Build water surface mesh from portal polygons ──
    meshes.waterMesh = buildWaterMesh(mission.wrData, mission.texDims, mission.flowData, mission.flowTexDims);
    meshes.hasWater = !meshes.waterMesh.vertices.empty();

    if (meshes.hasWater) {
        gpu.waterVBH = bgfx::createVertexBuffer(
            bgfx::copy(meshes.waterMesh.vertices.data(),
                        static_cast<uint32_t>(meshes.waterMesh.vertices.size() * sizeof(PosColorUVVertex))),
            PosColorUVVertex::layout);
        gpu.waterIBH = bgfx::createIndexBuffer(
            bgfx::copy(meshes.waterMesh.indices.data(),
                        static_cast<uint32_t>(meshes.waterMesh.indices.size() * sizeof(uint32_t))),
            BGFX_BUFFER_INDEX32);
    }

    // ── Create GPU buffers for object meshes ──

    if (showObjects) {
        // Build fallback cube
        {
            std::vector<PosColorVertex> cubeVerts;
            std::vector<uint32_t> cubeIndices;
            buildFallbackCube(cubeVerts, cubeIndices, 0xff808080u);
            gpu.fallbackCubeIndexCount = static_cast<uint32_t>(cubeIndices.size());
            gpu.fallbackCubeVBH = bgfx::createVertexBuffer(
                bgfx::copy(cubeVerts.data(),
                    static_cast<uint32_t>(cubeVerts.size() * sizeof(PosColorVertex))),
                PosColorVertex::layout);
            gpu.fallbackCubeIBH = bgfx::createIndexBuffer(
                bgfx::copy(cubeIndices.data(),
                    static_cast<uint32_t>(cubeIndices.size() * sizeof(uint32_t))),
                BGFX_BUFFER_INDEX32);
        }

        // Create bgfx texture handles with mip chains from loaded object texture images
        for (const auto &kv : mission.objTexImages) {
            const auto &img = kv.second;
            gpu.objTextureHandles[kv.first] = createMipmappedTexture(
                img.rgba.data(), img.width, img.height,
                BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT
                | BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR,
                MipAlphaMode::ALPHA_TEST, linearMips, sharpMips);
        }
        if (!gpu.objTextureHandles.empty()) {
            std::fprintf(stderr, "Created %zu object texture GPU handles\n",
                         gpu.objTextureHandles.size());
        }

        // Create GPU buffers for each parsed .bin model
        for (const auto &kv : mission.parsedModels) {
            const std::string &name = kv.first;
            const Darkness::ParsedBinMesh &mesh = kv.second;

            if (mesh.vertices.empty() || mesh.indices.empty()) continue;

            // Build a vertex-to-material map so we can colour vertices appropriately.
            // Each submesh covers a range of indices; map each index to its material.
            std::vector<int> vertexMatIndex(mesh.vertices.size(), -1);
            for (const auto &sm : mesh.subMeshes) {
                int mi = sm.matIndex;
                for (uint32_t ii = sm.firstIndex; ii < sm.firstIndex + sm.indexCount; ++ii) {
                    if (ii < mesh.indices.size()) {
                        uint32_t vi = mesh.indices[ii];
                        if (vi < vertexMatIndex.size()) {
                            vertexMatIndex[vi] = mi;
                        }
                    }
                }
            }

            // Fallback colour for non-textured models or Darkness::MD_MAT_COLOR materials
            uint32_t fallbackColor = colorFromName(name);

            // Build set of material indices that have actually-loaded textures,
            // so we only assign white to vertices that will be textured at draw time.
            std::unordered_set<int> loadedMatIndices;
            for (int mi = 0; mi < static_cast<int>(mesh.materials.size()); ++mi) {
                if (mesh.materials[mi].type == Darkness::MD_MAT_TMAP) {
                    std::string lname(mesh.materials[mi].name);
                    std::transform(lname.begin(), lname.end(), lname.begin(),
                                   [](unsigned char c) { return std::tolower(c); });
                    if (mission.objTexImages.count(lname)) {
                        loadedMatIndices.insert(mi);
                    }
                }
            }

            // Convert BinVert -> PosColorUVVertex
            std::vector<PosColorUVVertex> gpuVerts(mesh.vertices.size());
            for (size_t i = 0; i < mesh.vertices.size(); ++i) {
                const auto &bv = mesh.vertices[i];

                // Apply simple directional lighting using the vertex normal
                float dot = bv.nx * 0.3f + bv.ny * 0.8f + bv.nz * 0.4f;
                float brightness = std::max(dot, 0.0f) * 0.7f + 0.3f;

                // Determine vertex colour based on material type
                uint32_t vertColor;
                int mi = vertexMatIndex[i];
                if (mi >= 0 && loadedMatIndices.count(mi)) {
                    // Textured material with loaded texture: white * brightness
                    // (texture will modulate the final colour at draw time)
                    vertColor = packABGR(brightness, brightness, brightness);
                } else if (mi >= 0 && mi < static_cast<int>(mesh.materials.size()) &&
                           mesh.materials[mi].type == Darkness::MD_MAT_COLOR) {
                    // Solid-colour material: BGRA from material data * brightness
                    const uint8_t *c = mesh.materials[mi].colour;
                    vertColor = packABGR(c[2] / 255.0f * brightness,   // R (colour is BGRA)
                                         c[1] / 255.0f * brightness,   // G
                                         c[0] / 255.0f * brightness);  // B
                } else {
                    // No loaded texture, unknown material, or palette — use hash colour
                    uint8_t r = fallbackColor & 0xFF;
                    uint8_t g = (fallbackColor >> 8) & 0xFF;
                    uint8_t b = (fallbackColor >> 16) & 0xFF;
                    vertColor = packABGR(r / 255.0f * brightness,
                                         g / 255.0f * brightness,
                                         b / 255.0f * brightness);
                }

                gpuVerts[i] = {
                    bv.x, bv.y, bv.z,
                    vertColor,
                    bv.u, bv.v
                };
            }

            ObjectModelGPU objGPUEntry;
            objGPUEntry.vbh = bgfx::createVertexBuffer(
                bgfx::copy(gpuVerts.data(),
                    static_cast<uint32_t>(gpuVerts.size() * sizeof(PosColorUVVertex))),
                PosColorUVVertex::layout);
            objGPUEntry.ibh = bgfx::createIndexBuffer(
                bgfx::copy(mesh.indices.data(),
                    static_cast<uint32_t>(mesh.indices.size() * sizeof(uint32_t))),
                BGFX_BUFFER_INDEX32);

            // Build per-submesh GPU draw info from parsed submeshes
            for (const auto &sm : mesh.subMeshes) {
                ObjectSubMeshGPU gsm;
                gsm.firstIndex = sm.firstIndex;
                gsm.indexCount = sm.indexCount;
                gsm.textured = false;
                gsm.matTrans = 0.0f;
                if (sm.matIndex >= 0 && sm.matIndex < static_cast<int>(mesh.materials.size())) {
                    const auto &mat = mesh.materials[sm.matIndex];
                    gsm.textured = (mat.type == Darkness::MD_MAT_TMAP);
                    gsm.matTrans = mat.trans;  // per-material translucency
                    // Lowercase material name for texture lookup
                    gsm.matName = mat.name;
                    std::transform(gsm.matName.begin(), gsm.matName.end(),
                                   gsm.matName.begin(),
                                   [](unsigned char c) { return std::tolower(c); });
                }
                objGPUEntry.subMeshes.push_back(gsm);
            }

            objGPUEntry.valid = true;
            gpu.objModelGPU[name] = std::move(objGPUEntry);
        }
        std::fprintf(stderr, "Object GPU buffers: %zu models + fallback cube\n",
                     gpu.objModelGPU.size());

        // Count translucent objects/materials for diagnostics
        int translucentObjCount = 0;
        int translucentMatCount = 0;
        for (const auto &obj : mission.objData.objects) {
            if (obj.renderAlpha < 1.0f) ++translucentObjCount;
        }
        for (const auto &kv : gpu.objModelGPU) {
            if (!kv.second.valid) continue;
            for (const auto &sm : kv.second.subMeshes) {
                if (sm.matTrans > 0.0f) ++translucentMatCount;
            }
        }
        if (translucentObjCount > 0 || translucentMatCount > 0) {
            std::fprintf(stderr, "Translucent: %d objects (RenderAlpha), "
                         "%d material submeshes (mat_extra)\n",
                         translucentObjCount, translucentMatCount);
        }
    }

    // ── Create sky dome GPU buffers ──

    if (!mission.skyDome.vertices.empty()) {
        gpu.skyVBH = bgfx::createVertexBuffer(
            bgfx::copy(mission.skyDome.vertices.data(),
                static_cast<uint32_t>(mission.skyDome.vertices.size() * sizeof(PosColorVertex))),
            PosColorVertex::layout);
        gpu.skyIBH = bgfx::createIndexBuffer(
            bgfx::copy(mission.skyDome.indices.data(),
                static_cast<uint32_t>(mission.skyDome.indices.size() * sizeof(uint16_t))));
        gpu.skyIndexCount = static_cast<uint32_t>(mission.skyDome.indices.size());
    }

    // ── Create textured skybox GPU buffers (old sky system) ──

    if (mission.hasSkybox) {
        meshes.skyboxCube = buildSkyboxCube();

        gpu.skyboxVBH = bgfx::createVertexBuffer(
            bgfx::copy(meshes.skyboxCube.vertices.data(),
                static_cast<uint32_t>(meshes.skyboxCube.vertices.size() * sizeof(PosColorUVVertex))),
            PosColorUVVertex::layout);
        gpu.skyboxIBH = bgfx::createIndexBuffer(
            bgfx::copy(meshes.skyboxCube.indices.data(),
                static_cast<uint32_t>(meshes.skyboxCube.indices.size() * sizeof(uint16_t))));

        // Create GPU textures with mip chains for each loaded skybox face
        // Skybox faces are fully opaque so use ALPHA_BLEND (no coverage preservation)
        for (const auto &kv : mission.skyboxImages) {
            const auto &img = kv.second;
            gpu.skyboxTexHandles[kv.first] = createMipmappedTexture(
                img.rgba.data(), img.width, img.height,
                BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP,
                MipAlphaMode::ALPHA_BLEND, linearMips, sharpMips);
        }
        std::fprintf(stderr, "Skybox GPU: %zu face textures created\n",
                     gpu.skyboxTexHandles.size());
    }

    return true;
}

// ── Initialize runtime state ──
// Sets render mode string, model isolation data, spawn/camera position,
// SDL mouse capture, and portal culling stats.
static void initRuntimeState(
    const Darkness::MissionData &mission,
    const Darkness::BuiltMeshes &meshes,
    const Darkness::GPUResources &gpu,
    float centroidX, float centroidY, float centroidZ,
    Darkness::RuntimeState &state)
{
    // Render mode description for title bar
    state.modeStr = meshes.lightmappedMode ? "lightmapped" :
                    mission.texturedMode ? "textured" : "flat-shaded";
    std::fprintf(stderr, "Render window opened (%dx%d, %s, %s)\n",
                 WINDOW_WIDTH, WINDOW_HEIGHT,
                 bgfx::getRendererName(bgfx::getRendererType()), state.modeStr);
    std::fprintf(stderr, "Portal culling: %s (toggle with C key)\n",
                 state.portalCulling ? "ON" : "OFF");

    // ── Model isolation state for debugging ──
    // Sorted list of model names that have loaded GPU data, with instance counts.
    // Press N to cycle through, isolating one model at a time for identification.
    if (state.showObjects) {
        // Count instances per model name
        for (const auto &obj : mission.objData.objects) {
            if (!obj.hasPosition) continue;
            std::string mn(obj.modelName);
            state.modelInstanceCounts[mn]++;
        }

        // Build sorted list of models that have GPU data (loaded successfully)
        for (const auto &kv : gpu.objModelGPU) {
            if (kv.second.valid)
                state.sortedModelNames.push_back(kv.first);
        }
        std::sort(state.sortedModelNames.begin(), state.sortedModelNames.end());

        // Count unloaded models (fallback cube candidates)
        int unloadedCount = 0;
        for (const auto &kv : state.modelInstanceCounts) {
            if (gpu.objModelGPU.find(kv.first) == gpu.objModelGPU.end() || !gpu.objModelGPU.at(kv.first).valid)
                ++unloadedCount;
        }
        std::fprintf(stderr, "Model isolation: %zu loaded, %d unloaded (M=next, N=prev)\n",
                     state.sortedModelNames.size(), unloadedCount);
    }

    // Use spawn point if found, otherwise fall back to centroid
    state.spawnX = centroidX; state.spawnY = centroidY; state.spawnZ = centroidZ;
    state.spawnYaw = 0.0f;
    if (mission.spawnInfo.found) {
        state.spawnX = mission.spawnInfo.x;
        state.spawnY = mission.spawnInfo.y;
        state.spawnZ = mission.spawnInfo.z;
        state.spawnYaw = mission.spawnInfo.yaw;
        std::fprintf(stderr, "Camera at spawn (%.1f, %.1f, %.1f)\n",
                     state.spawnX, state.spawnY, state.spawnZ);
    } else {
        std::fprintf(stderr, "Camera at centroid (%.1f, %.1f, %.1f)\n",
                     centroidX, centroidY, centroidZ);
    }

    std::fprintf(stderr, "Controls: WASD=move, mouse=look, Space=jump, LShift=crouch, Q/E=lean, "
                 "scroll=speed, `=console, Home=spawn, BS+P=walk, BS+C/F/V/M/N/R=debug, Esc=quit\n");

    // Initialize camera at spawn position
    state.cam.init(state.spawnX, state.spawnY, state.spawnZ);
    state.cam.yaw = state.spawnYaw;

    SDL_SetRelativeMouseMode(SDL_TRUE);

    // Portal culling stats for title bar display
    state.cullTotalCells = mission.wrData.numCells;
}

// ── Cleanup functions ──
// Destroy all bgfx GPU resources created during initialization.
static void destroyGPUResources(Darkness::GPUResources &gpu)
{
    // Water surface buffers and flow textures
    if (bgfx::isValid(gpu.waterVBH)) bgfx::destroy(gpu.waterVBH);
    if (bgfx::isValid(gpu.waterIBH)) bgfx::destroy(gpu.waterIBH);
    for (auto &kv : gpu.flowTextureHandles)
        bgfx::destroy(kv.second);

    // Textured skybox buffers
    if (bgfx::isValid(gpu.skyboxVBH)) bgfx::destroy(gpu.skyboxVBH);
    if (bgfx::isValid(gpu.skyboxIBH)) bgfx::destroy(gpu.skyboxIBH);
    for (auto &kv : gpu.skyboxTexHandles)
        bgfx::destroy(kv.second);

    // Sky dome buffers
    if (bgfx::isValid(gpu.skyVBH)) bgfx::destroy(gpu.skyVBH);
    if (bgfx::isValid(gpu.skyIBH)) bgfx::destroy(gpu.skyIBH);

    // Object GPU buffers and textures
    for (auto &kv : gpu.objModelGPU) {
        if (bgfx::isValid(kv.second.vbh)) bgfx::destroy(kv.second.vbh);
        if (bgfx::isValid(kv.second.ibh)) bgfx::destroy(kv.second.ibh);
    }
    for (auto &kv : gpu.objTextureHandles) {
        bgfx::destroy(kv.second);
    }
    if (bgfx::isValid(gpu.fallbackCubeVBH)) bgfx::destroy(gpu.fallbackCubeVBH);
    if (bgfx::isValid(gpu.fallbackCubeIBH)) bgfx::destroy(gpu.fallbackCubeIBH);

    // World geometry and textures
    for (auto &h : gpu.lightmapAtlasHandles)
        bgfx::destroy(h);
    for (auto &kv : gpu.textureHandles)
        bgfx::destroy(kv.second);
    bgfx::destroy(gpu.s_texLightmap);
    bgfx::destroy(gpu.s_texColor);
    bgfx::destroy(gpu.u_waterParams);
    bgfx::destroy(gpu.u_waterFlow);
    bgfx::destroy(gpu.u_fogColor);
    bgfx::destroy(gpu.u_fogParams);
    bgfx::destroy(gpu.u_objectParams);
    bgfx::destroy(gpu.u_lmAtlasSize);

    bgfx::destroy(gpu.ibh);
    bgfx::destroy(gpu.vbh);
    bgfx::destroy(gpu.flatProgram);
    bgfx::destroy(gpu.texturedProgram);
    bgfx::destroy(gpu.lightmappedProgram);
    if (bgfx::isValid(gpu.lightmappedBicubicProgram))
        bgfx::destroy(gpu.lightmappedBicubicProgram);
    bgfx::destroy(gpu.waterProgram);
}

// Shut down bgfx and SDL2 window.
static void shutdownWindow(SDL_Window *window) {
    bgfx::shutdown();
    SDL_DestroyWindow(window);
    SDL_Quit();
}
