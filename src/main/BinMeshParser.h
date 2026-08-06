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

// BinMeshParser — parses LGMD .bin model files into bgfx-ready triangle lists.
//
// Uses the existing BinFormat.h structs (BinHeader, SubObjectHeader, ObjPolygon,
// ObjLight, NodeSplit/Call/Raw, etc.) with File I/O operators. Follows the same
// BSP tree walk algorithm from LGMD model format specification.
//
// Input: raw .bin file bytes (from CRFModelLoader or file read).
// Output: ParsedBinMesh with vertex/index arrays ready for bgfx VBH/IBH.

#pragma once

#include "File.h"
#include "FileCompat.h"
#include "BinFormat.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

namespace Darkness {

// Output vertex with position, normal, and UV
struct BinVert {
    float x, y, z;
    float nx, ny, nz;
    float u, v;
};

// Material info extracted from the .bin file
struct BinMatInfo {
    char name[16];
    uint8_t type;      // MD_MAT_TMAP (0) or MD_MAT_COLOR (1)
    uint8_t colour[4]; // BGRA colour for MD_MAT_COLOR materials
    float trans;       // per-material translucency (0.0=opaque, >0=translucent)
    float illum;       // self-illumination (0.0=none, 1.0=full)
};

// Per-(material, sub-object) submesh range within the index buffer.
//
// The split is on BOTH keys, not material alone: a sub-object is a moving part
// (door handle, chest lid, clock hand) with its own transform, so every draw
// range must have exactly one constant sub-object matrix. Splitting costs a few
// extra draw calls on the 241-of-1782 stock models that have more than one part.
struct BinSubMesh {
    uint32_t firstIndex;
    uint32_t indexCount;
    int matIndex; // index into ParsedBinMesh::materials, or -1 for palette
    int subObj;   // index into ParsedBinMesh::subObjects
};

// One entry of the LGMD sub-object table — a rigid part of the model that the
// engine can pose independently (the "joints" the JointPos property drives).
//
// Vertices are stored in each sub-object's OWN local space; `rot` + `axle` place
// that space inside the parent's. We keep the record instead of baking the
// transform into the vertices, because baking freezes every model in its rest
// pose and makes joint animation impossible by construction.
struct BinSubObject {
    char     name[9];     // "@sNN<label>"; NN is the joint number, zero-padded
    uint8_t  movement;    // 0 = static, 1 = rotate, 2 = slide
    int32_t  jointIdx;    // joint slot driving this part, or -1 when static
    float    minRange;    // authored travel limits: radians (rotate),
    float    maxRange;    //   world units (slide). NOT ordered — see NOTES.
    float    rot[9];      // 3x3 basis, column-major: cols are the images of
                          //   the part's local X/Y/Z in the parent's space
    float    axle[3];     // joint origin, in the parent's space
    int16_t  child;       // first child sub-object, or -1
    int16_t  next;        // next sibling sub-object, or -1
};

// One LGMD attachment point — a "vhot" in the engine's vocabulary. Artists
// place these to mark a spot on the model that gameplay needs to name: where a
// torch's flame sits, where an AI holds a sword, where a lamp emits light.
//
// `index` is the engine's SLOT NUMBER, not the position in the array. The two
// disagree in practice and the distinction is load-bearing: STRLANT.BIN stores
// index 3 at slot 0 and index 1 at slot 1, while GASLITE2.BIN stores the same
// two indices in the opposite order. A consumer that took "the first vhot"
// would read the wrong point on one of them. Always match on `index`.
//
// Slot 1 is the light position — see kVHotLight in CoronaSystem.h for the
// measurement that established it.
struct BinVHot {
    uint32_t index;
    // Owning sub-object. `point` is in THAT part's local space, so a vhot on a
    // moving part must be pushed through the part's frame to reach model space;
    // findVHotModelSpace() in SubObjectPose.h does that. LITEBEAK.BIN is the
    // case that makes it matter — both its vhots hang off a rotating part whose
    // axle sits 4.27 units off the model origin.
    int      subObj;
    float    point[3];
};

// Result of parsing a single .bin model file
struct ParsedBinMesh {
    std::vector<BinVert> vertices;
    std::vector<uint32_t> indices;
    float bboxMin[3];
    float bboxMax[3];
    // Bounding sphere radius from the .bin header (Dark Engine `sphere_rad`).
    // Used by per-object lighting to position the virtual sun far enough
    // away that the object isn't "inside" it. Falls back to 1.0 if the
    // field wasn't read.
    float sphereRadius = 1.0f;
    std::vector<BinSubMesh> subMeshes;
    std::vector<BinMatInfo> materials;
    // The sub-object table, in file order. Entry 0 is the model root. Empty
    // only when the model has no geometry at all.
    std::vector<BinSubObject> subObjects;
    // Attachment points, in file order. Empty for the 1531 of 1783 stock
    // models that carry none.
    std::vector<BinVHot> vhots;
    bool valid; // false if parsing failed
};

// ── Internal parser state ──

namespace detail {

// Unpack a 10-bit-per-axis packed normal (same algorithm as ManualBinFileLoader)
//
// Bit layout (32 bits, MSB to LSB):
//   XXXX XXXX | XXYY YYYY || YYYY ZZZZ | ZZZZ ZZ00
// Each axis is a 10-bit signed fixed-point number.
inline Vertex unpackNormal(uint32_t src) {
    Vertex res;
    res.z = static_cast<int16_t>(static_cast<int16_t>(src & 0x0FFC) << 4) / 16384.0f;
    res.y = static_cast<int16_t>((src >> 6) & 0x0FFC0) / 16384.0f;
    res.x = static_cast<int16_t>((src >> 16) & 0x0FFC0) / 16384.0f;
    return res;
}

// Internal parser class — holds state during BSP tree walk
class BinParser {
public:
    BinParser(const uint8_t *data, size_t size)
        : mValid(false)
    {
        // Wrap raw bytes in a MemoryFile for use with BinFormat.h I/O operators.
        // Write the data in, then seek back to 0 for reading.
        mFile = FilePtr(new MemoryFile("binmesh", File::FILE_RW));
        mFile->write(data, size);
        mFile->seek(static_cast<file_pos_t>(0));
    }

    ParsedBinMesh parse() {
        ParsedBinMesh result = {};
        result.valid = false;

        try {
            return parseInternal(result);
        } catch (const std::exception &e) {
            // File I/O error (e.g. seek past end) — return invalid mesh
            std::fprintf(stderr, "    [BIN] EXCEPTION: %s\n", e.what());
            result.valid = false;
            return result;
        }
    }

private:
    ParsedBinMesh &parseInternal(ParsedBinMesh &result) {
        mFileSize = mFile->size();
        if (mFileSize < 8) {
            return result;
        }

        // Read and validate magic header (same as ManualBinFileLoader)
        char magic[5] = {};
        mFile->read(magic, 4);
        uint32_t version;
        mFile->read(&version, 4);

        if (std::string(magic) != "LGMD") {
            return result;
        }

        if (version != 3 && version != 4) {
            return result;
        }

        mVersion = version;

        // Read the main header
        mHdr.read(*mFile, version);

        // Sanity check: offsets must be within file bounds
        if (mHdr.offset_pgons >= mFileSize || mHdr.offset_nodes >= mFileSize ||
            mHdr.offset_verts >= mFileSize || mHdr.offset_mats >= mFileSize) {
            return result;
        }

        // Copy bounding box and bsphere radius
        for (int i = 0; i < 3; ++i) {
            result.bboxMin[i] = mHdr.bbox_min[i];
            result.bboxMax[i] = mHdr.bbox_max[i];
        }
        result.sphereRadius = (mHdr.sphere_rad > 0.0f) ? mHdr.sphere_rad : 1.0f;

        // Calculate derived counts (same as ObjectMeshLoader::load())
        mNumUVs = (mHdr.offset_vhots - mHdr.offset_uv) / (sizeof(float) * 2);
        mNumLights = (mHdr.offset_norms - mHdr.offset_light) / ObjLight_Size;

        // Read materials
        readMaterials();
        for (size_t i = 0; i < mMaterials.size(); ++i) {
            const auto &mat = mMaterials[i];
            BinMatInfo info = {};
            std::memcpy(info.name, mat.name, 16);
            info.name[15] = '\0';
            info.type = mat.type;
            if (mat.type == MD_MAT_COLOR) {
                std::memcpy(info.colour, mat.colour, 4);
            }
            // Copy auxiliary material data (transparency, illumination)
            if (i < mMaterialsExtra.size()) {
                info.trans = mMaterialsExtra[i].trans;
                info.illum = mMaterialsExtra[i].illum;
            }
            result.materials.push_back(info);
        }

        // Read UVs
        readUVs();

        // Read vertices
        readVertices();

        // Read ObjLight entries (per-vertex normals as packed normals)
        readLights();

        // Read sub-object headers
        readSubObjects();

        exportSubObjects(result);

        // Attachment points. After readSubObjects, because each vhot's local
        // space is its owning sub-object's and only the sub-object table says
        // which one that is.
        readVHots(result);

        // Walk BSP tree for each sub-object, collecting polygons per
        // (sub-object, material) pair.
        mResult = &result;
        loadSubObject(0, -1);

        // Build submesh ranges. The map is ordered by (subObj, matIndex), so
        // ranges come out grouped by part — draw order follows the model tree.
        for (auto &kv : mMatTriangles) {
            BinSubMesh sm;
            sm.subObj = kv.first.first;
            sm.matIndex = kv.first.second;
            sm.firstIndex = static_cast<uint32_t>(result.indices.size());
            sm.indexCount = static_cast<uint32_t>(kv.second.size());
            result.subMeshes.push_back(sm);
            result.indices.insert(result.indices.end(),
                                  kv.second.begin(), kv.second.end());
        }

        result.valid = !result.vertices.empty();
        mValid = result.valid;
        return result;
    }

private:
    // ── Data reading methods (mirror ObjectMeshLoader) ──

    void readMaterials() {
        mFile->seek(mHdr.offset_mats);
        mMaterials.resize(mHdr.num_mats);

        for (auto &mat : mMaterials) {
            mFile->read(mat.name, 16);
            *mFile >> mat.type >> mat.slot_num;

            if (mat.type == MD_MAT_COLOR) {
                mFile->read(mat.colour, 4);
                *mFile >> mat.ipal_index;
            } else if (mat.type == MD_MAT_TMAP) {
                *mFile >> mat.handle >> mat.uvscale;
            } else {
                // Unknown type — read 8 bytes to stay aligned
                uint8_t skip[8];
                mFile->read(skip, 8);
            }
        }

        // Build slot -> material index mapping
        for (int i = 0; i < mHdr.num_mats; ++i) {
            mSlotToMat[mMaterials[i].slot_num] = i;
        }

        // Read auxiliary material data (transparency + illumination) for v4+ models
        readMaterialsExtra();
    }

    // Read per-material transparency and illumination from mat_extra section.
    // Only present in version 4+ models. Either MD_MAT_TRANS (=0x1) or
    // MD_MAT_ILLUM (=0x2) being set means the section exists; lamps with
    // illum-only materials skip MD_MAT_TRANS but still need mat_extra
    // parsed, otherwise their casings load with illum=0 and the per-vertex
    // path's additive self-illumination floor is missing.
    // Each record is {float trans, float illum} (8 bytes minimum).
    void readMaterialsExtra() {
        mMaterialsExtra.resize(mHdr.num_mats);
        for (auto &mext : mMaterialsExtra) {
            mext.trans = 0.0f;
            mext.illum = 0.0f;
        }

        if (mVersion < 4) return;
        if (!(mHdr.mat_flags & (MD_MAT_TRANS | MD_MAT_ILLUM))) return;
        if (mHdr.offset_mat_extra <= 0 || mHdr.size_mat_extra < 8) return;

        uint32_t extraOffset = static_cast<uint32_t>(mHdr.offset_mat_extra);
        if (extraOffset >= mFileSize) return;

        mFile->seek(extraOffset);

        // Bytes to skip per record beyond the 8-byte {trans, illum} we read
        int skipBytes = mHdr.size_mat_extra - 8;
        if (skipBytes < 0) skipBytes = 0;

        for (auto &mext : mMaterialsExtra) {
            *mFile >> mext.trans >> mext.illum;
            if (skipBytes > 0) {
                mFile->seek(static_cast<file_offset_t>(skipBytes), File::FSEEK_CUR);
            }
        }
    }

    void readUVs() {
        if (mNumUVs > 0) {
            mUVs.resize(mNumUVs);
            mFile->seek(mHdr.offset_uv);
            *mFile >> mUVs;
        }
    }

    void readVertices() {
        if (mHdr.num_verts > 0) {
            mVertices.resize(mHdr.num_verts);
            mFile->seek(mHdr.offset_verts);
            *mFile >> mVertices;
        }
    }

    void readLights() {
        if (mNumLights > 0) {
            mLights.resize(mNumLights);
            mFile->seek(mHdr.offset_light);
            *mFile >> mLights;

            // Unpack normals from ObjLight entries
            mNormals.resize(mNumLights);
            for (int i = 0; i < mNumLights; ++i) {
                mNormals[i] = unpackNormal(mLights[i].packed_normal);
            }
        }
    }

    void readSubObjects() {
        mFile->seek(mHdr.offset_objs);
        mSubObjects.resize(mHdr.num_objs);
        *mFile >> mSubObjects;
    }

    // Copy the sub-object table into the parsed result, so the pose can be
    // composed at draw time instead of baked into the vertices at load.
    void exportSubObjects(ParsedBinMesh &result) {
        result.subObjects.reserve(mSubObjects.size());
        for (const auto &s : mSubObjects) {
            BinSubObject bs = {};
            std::memcpy(bs.name, s.name, 8);
            bs.name[8] = '\0';
            bs.movement = s.movement;
            bs.jointIdx = s.trans.joint_idx;
            bs.minRange = s.trans.min_range;
            bs.maxRange = s.trans.max_range;
            std::memcpy(bs.rot, s.trans.rot, sizeof(bs.rot));
            bs.axle[0] = s.trans.axle_point.x;
            bs.axle[1] = s.trans.axle_point.y;
            bs.axle[2] = s.trans.axle_point.z;
            bs.child = s.child_sub_obj;
            bs.next = s.next_sub_obj;
            result.subObjects.push_back(bs);
        }
    }

    // Read the vhot table and attribute each entry to the sub-object that owns
    // it, via that sub-object's [vhot_start, vhot_start + sub_num_vhots) range.
    //
    // Ranges are applied in table order and later writes win. That is not
    // arbitrary: a parent whose own count is zero still stores a start index
    // (LITEBEAK.BIN's root says start=0 count=0 while its child says start=0
    // count=2), so "last non-empty range covering this slot" is the reading
    // that lands both vhots on the child, where the geometry actually is.
    void readVHots(ParsedBinMesh &result) {
        if (mHdr.num_vhots == 0) return;

        const size_t start = static_cast<size_t>(mHdr.offset_vhots);
        const size_t bytes = static_cast<size_t>(mHdr.num_vhots) * 16;
        if (start >= mFileSize || bytes > mFileSize - start) {
            std::fprintf(stderr,
                "[FALLBACK] BinMeshParser: vhot table (%u entries at %zu) runs "
                "past the end of a %zu-byte model — no attachment points read, "
                "so anything anchored to one falls back to the model origin\n",
                mHdr.num_vhots, start, mFileSize);
            return;
        }

        result.vhots.resize(mHdr.num_vhots);
        mFile->seek(mHdr.offset_vhots);
        for (auto &v : result.vhots) {
            VHotObj raw;
            *mFile >> raw;
            v.index    = raw.index;
            v.subObj   = 0;
            v.point[0] = raw.point.x;
            v.point[1] = raw.point.y;
            v.point[2] = raw.point.z;
        }

        for (size_t s = 0; s < mSubObjects.size(); ++s) {
            const int first = mSubObjects[s].vhot_start;
            const int count = mSubObjects[s].sub_num_vhots;
            if (first < 0 || count <= 0) continue;
            for (int j = first;
                 j < first + count && j < static_cast<int>(result.vhots.size());
                 ++j) {
                result.vhots[static_cast<size_t>(j)].subObj =
                    static_cast<int>(s);
            }
        }
    }

    // ── BSP tree walk (mirrors ObjectMeshLoader::loadSubObject/loadSubNode) ──

    // Recursively process sub-objects following the child/next linked list
    void loadSubObject(int obj, int parent) {
        int subobj = obj;

        while (subobj >= 0 && subobj < static_cast<int>(mSubObjects.size())) {
            // Recurse into children first
            if (mSubObjects[subobj].child_sub_obj >= 0) {
                loadSubObject(mSubObjects[subobj].child_sub_obj, subobj);
            }

            // Process this sub-object's BSP tree
            mCurrentSubObj = subobj;
            loadSubNode(subobj, mSubObjects[subobj].node_start);

            // Move to next sibling
            subobj = mSubObjects[subobj].next_sub_obj;
        }
    }

    // Walk a BSP node tree, collecting polygon offsets
    void loadSubNode(int obj, size_t offset) {
        size_t absOffset = mHdr.offset_nodes + offset;
        if (absOffset >= mFileSize || absOffset < mHdr.offset_nodes) {
            return;
        }

        mFile->seek(absOffset);

        uint8_t type;
        mFile->read(&type, 1);

        if (type == MD_NODE_HDR) {
            NodeHeader ndhdr;
            *mFile >> ndhdr;

            if (obj == ndhdr.subObjectID) {
                // Skip header, recurse at next offset
                // NOTE: NodeHeader::SIZE includes the type byte offset
                // (matching ManualBinFileLoader.cpp reference)
                loadSubNode(obj, offset + NodeHeader::SIZE);
            }
        } else if (type == MD_NODE_SPLIT) {
            NodeSplit ns;
            *mFile >> ns;

            // Collect polygons listed before and after the split
            loadPolygons(obj, ns.pgon_before_count + ns.pgon_after_count);

            // Recurse into behind and front children
            loadSubNode(obj, ns.behind_node);
            loadSubNode(obj, ns.front_node);
        } else if (type == MD_NODE_CALL) {
            NodeCall nc;
            *mFile >> nc;

            // Collect polygons but do NOT recurse call_node
            // (as per ManualBinFileLoader.cpp comment: call_node recursion
            // doesn't work — seeks past end of file)
            loadPolygons(obj, nc.pgon_before_count + nc.pgon_after_count);
        } else if (type == MD_NODE_RAW) {
            NodeRaw nr;
            *mFile >> nr;

            loadPolygons(obj, nr.pgon_count);
        }
        // Unknown node types are silently ignored
    }

    // Read polygon offset list and process each polygon
    void loadPolygons(int obj, size_t count) {
        if (count == 0) return;

        // Read polygon offset indices from current file position
        std::vector<uint16_t> polyOffsets(count);
        *mFile >> polyOffsets;

        for (size_t n = 0; n < count; ++n) {
            // Seek to polygon data
            mFile->seek(mHdr.offset_pgons + polyOffsets[n]);

            // Read polygon header
            ObjPolygon op;
            *mFile >> op;

            if (op.num_verts < 3) continue;

            // Determine material and whether UVs are used
            int polyType = op.type & 0x07;
            bool useUV = (polyType == MD_PGON_TMAP);

            // Resolve material index
            int matIndex = 0;
            if (polyType == MD_PGON_TMAP || (op.type & 0x60) == MD_PGON_SOLID_COLOR_VCOLOR) {
                auto it = mSlotToMat.find(op.data);
                if (it != mSlotToMat.end()) {
                    matIndex = it->second;
                }
            }

            // Read vertex indices
            std::vector<uint16_t> vertIndices(op.num_verts);
            *mFile >> vertIndices;

            // Read light/normal indices
            std::vector<uint16_t> lightIndices(op.num_verts);
            *mFile >> lightIndices;

            // Read UV indices (only for TMAP polygons)
            std::vector<uint16_t> uvIndices;
            if (useUV) {
                uvIndices.resize(op.num_verts);
                *mFile >> uvIndices;
            }

            // Emit vertices and fan-triangulate
            // Use the same winding as ManualBinFileLoader: (last, i, i-1)
            // which is equivalent to fan from last vertex
            uint32_t baseVert = static_cast<uint32_t>(mResult->vertices.size());

            for (int vi = 0; vi < op.num_verts; ++vi) {
                BinVert bv = {};

                // Position from vertex table, in the sub-object's OWN local
                // space. The sub-object transform used to be applied here, once,
                // at load — which froze every model in its rest pose and left the
                // vertex normals (which were never transformed) inconsistent with
                // the positions. It is now composed per draw call from
                // ParsedBinMesh::subObjects, so parts can move.
                if (vertIndices[vi] < mVertices.size()) {
                    const Vertex &v = mVertices[vertIndices[vi]];
                    bv.x = v.x;
                    bv.y = v.y;
                    bv.z = v.z;
                }

                // Normal from light/ObjLight table (packed normals)
                if (lightIndices[vi] < mNormals.size()) {
                    const Vertex &n = mNormals[lightIndices[vi]];
                    bv.nx = n.x;
                    bv.ny = n.y;
                    bv.nz = n.z;
                }

                // UV coordinates from UV table
                if (useUV && vi < static_cast<int>(uvIndices.size()) &&
                    uvIndices[vi] < mUVs.size()) {
                    bv.u = mUVs[uvIndices[vi]].u;
                    bv.v = mUVs[uvIndices[vi]].v;
                }

                mResult->vertices.push_back(bv);
            }

            // Fan-triangulate: same winding as ManualBinFileLoader
            // (last, i, i-1) for i = 1..numverts-2
            auto &triList = mMatTriangles[{mCurrentSubObj, matIndex}];
            uint32_t lastVert = baseVert + op.num_verts - 1;

            for (int i = 1; i < op.num_verts - 1; ++i) {
                triList.push_back(lastVert);
                triList.push_back(baseVert + i);
                triList.push_back(baseVert + i - 1);
            }
        }
    }

    // ── Member state ──

    FilePtr mFile;
    size_t mFileSize = 0;
    unsigned mVersion = 0;
    bool mValid;

    BinHeader mHdr = {};
    int mNumUVs = 0;
    int mNumLights = 0;
    int mCurrentSubObj = 0;

    std::vector<MeshMaterial> mMaterials;
    std::vector<MeshMaterialExtra> mMaterialsExtra;  // per-material trans + illum
    std::map<int, int> mSlotToMat; // slot_num -> material index
    std::vector<UVMap> mUVs;
    std::vector<Vertex> mVertices;
    std::vector<ObjLight> mLights;
    std::vector<Vertex> mNormals;         // unpacked from ObjLight
    std::vector<SubObjectHeader> mSubObjects;

    // Triangle index lists keyed by (sub-object, material), built during the
    // BSP walk. std::map keeps the emitted submesh order deterministic.
    std::map<std::pair<int, int>, std::vector<uint32_t>> mMatTriangles;

    // Output pointer (set during parse())
    ParsedBinMesh *mResult = nullptr;
};

} // namespace detail

// ── Public API ──

// Parse an LGMD .bin model from a raw memory buffer.
// Returns ParsedBinMesh with valid=true on success.
inline ParsedBinMesh parseBinModel(const uint8_t *data, size_t size) {
    detail::BinParser parser(data, size);
    return parser.parse();
}

} // namespace Darkness
