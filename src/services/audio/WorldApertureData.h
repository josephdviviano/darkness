/******************************************************************************
 *
 *    This file is part of the darkness project
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

#ifndef __WORLD_APERTURE_DATA_H
#define __WORLD_APERTURE_DATA_H

/// @file WorldApertureData.h
/// Renderer-computed ground truth about the level's REAL acoustic apertures,
/// consumed by the pathing-probe placement in
/// AudioService::prepareProbeBakeParams.
///
/// WHY THIS EXISTS: ROOM_DB is the original engine's dampening partition, not
/// an aperture graph. Its "portals" are designer box-box boundaries that
/// frequently stand in open air modelling nothing (measured on MISS2: 164 of
/// 668 portals — 24.6% — have no world-geometry opening anywhere near them,
/// and the worst room's degree drops 53 -> 4 when the fiction is removed),
/// while the boundaries that ARE real report fictional sizes (portals reading
/// 6.3 ft inradius over real 1.9-2.8 ft arches). Probe placement keyed on
/// room centroids and portal centers therefore over-produces probes and puts
/// them in the wrong places — including inside solid rock (a ROOM_DB portal
/// center was measured 42.66 ft inside a wall).
///
/// The compiled WR cell geometry is the truth: cells ARE air, and a cell
/// portal polygon at a doorway measures the opening exactly (a standard door
/// reads half-width 2.00 ft, no tuning). The renderer — which owns the WR
/// data — cross-references every ROOM_DB portal against the WR portal set
/// and hands the result here: which ROOM_DB portals correspond to a real
/// opening, where that opening actually is, and how the level's air
/// decomposes into REGIONS (air volumes bounded by real apertures — what
/// ROOM_DB rooms pretend to be; MISS2 has ~47 such spaces, not 404 rooms).
///
/// Full derivation and measurements: .claude/PLAN.PATHING_DESIGN.md §33-37.
///
/// Population: renderer-side (WRAcousticTopology.h) — WR data lives there,
/// mirroring the setRaycaster / setPointInAirFn precedent. When unset, the
/// bake must refuse portal-first placement LOUDLY (a silent fallback would
/// quietly rebuild the fictional layout this exists to replace).

#include <cstdint>
#include <vector>

#include "DarknessMath.h"

namespace Darkness {

/// One oracle-MATCHED ROOM_DB portal: the ROOM_DB portal named by
/// (roomAID, roomBID, rdbCenter) has a real world-geometry opening, and that
/// opening is described by the WR fields. ROOM_DB portals with NO match are
/// simply absent from the list — absence IS the fiction verdict.
struct WorldApertureRecord {
    /// In-air probe anchor AT the aperture: the WR portal polygon centroid
    /// nudged off the face into one of its two cells and verified against
    /// the cell network. In-air BY CONSTRUCTION (cells are air), which makes
    /// the in-solid probe class placement used to suffer (3.6% of probes)
    /// impossible rather than filtered after the fact.
    Vector3 probePos{0.0f, 0.0f, 0.0f};
    /// The aperture polygon centroid itself (ON the shared cell face).
    Vector3 wrCentroid{0.0f, 0.0f, 0.0f};
    /// Real aperture size: half the opening's narrow dimension, measured on
    /// the WR polygon (centroid-to-edge-segment minimum). A standard Thief
    /// doorway reads 2.00.
    float apertureInradiusFt = -1.0f;

    /// The ROOM_DB portal this record answers for. portalID is the lookup
    /// key (RoomPortal::getPortalID(), unique per mission) — an exact
    /// identity, deliberately NOT a quantized-position match: both sides
    /// enumerate the same RoomService portals through the same back-link
    /// dedup, so the ID carries with zero geometric assumptions and
    /// survives any future serialization round-trip.
    /// NOTE: one portalID can carry SEVERAL records — a ROOM_DB portal's
    /// match ball can span two distinct physical openings (e.g. a doorway
    /// plus a nearby window joining a different region pair); each opening
    /// gets its own record so none is silently unprobed.
    int32_t portalID = -1;
    Vector3 rdbCenter{0.0f, 0.0f, 0.0f};
    int roomAID = -1;   ///< ROOM_DB room pair (unordered; min stored first)
    int roomBID = -1;

    /// Region IDs of the two air volumes this aperture joins (indices in
    /// [0, numRegions)). Equal when the aperture is internal to one region
    /// (possible for multi-portal compound openings).
    int regionA = -1;
    int regionB = -1;

    /// Identity of the PHYSICAL opening. A doorway's frame is its own chain
    /// of thin WR cells, so one opening appears as several same-size portal
    /// polygons (measured: median 2 per door); all of them — and every
    /// ROOM_DB portal matching any of them — share one key. "One aperture,
    /// one probe (or door pair)" dedups on this, replacing the old geometric
    /// same-opening heuristics with an identity. Composed from the match
    /// union root AND the region pair, so two distinct openings caught in
    /// one match ball (different region pairs) keep distinct keys.
    uint64_t apertureKey = 0;
};

/// The full renderer-computed topology handed to AudioService before the
/// pathing bake. `valid` guards half-constructed data: default-constructed
/// (never set) reads invalid.
struct WorldApertureData {
    std::vector<WorldApertureRecord> apertures;
    /// Number of air regions (connected components of the WR cell graph
    /// after cutting every matched aperture). Region IDs in the records and
    /// from the region-of-point callback index this range.
    int numRegions = 0;
    bool valid = false;
};

} // namespace Darkness

#endif // __WORLD_APERTURE_DATA_H
