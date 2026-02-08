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
// BSP tree walk algorithm as ManualBinFileLoader.cpp (ObjectMeshLoader).
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

namespace Opde {

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
};

// Per-material submesh range within the index buffer
struct BinSubMesh {
    uint32_t firstIndex;
    uint32_t indexCount;
    int matIndex; // index into ParsedBinMesh::materials, or -1 for palette
};

// Result of parsing a single .bin model file
struct ParsedBinMesh {
    std::vector<BinVert> vertices;
    std::vector<uint32_t> indices;
    float bboxMin[3];
    float bboxMax[3];
    std::vector<BinSubMesh> subMeshes;
    std::vector<BinMatInfo> materials;
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

        // Copy bounding box
        for (int i = 0; i < 3; ++i) {
            result.bboxMin[i] = mHdr.bbox_min[i];
            result.bboxMax[i] = mHdr.bbox_max[i];
        }

        // Calculate derived counts (same as ObjectMeshLoader::load())
        mNumUVs = (mHdr.offset_vhots - mHdr.offset_uv) / (sizeof(float) * 2);
        mNumLights = (mHdr.offset_norms - mHdr.offset_light) / ObjLight_Size;

        // Read materials
        readMaterials();
        for (const auto &mat : mMaterials) {
            BinMatInfo info = {};
            std::memcpy(info.name, mat.name, 16);
            info.name[15] = '\0';
            info.type = mat.type;
            if (mat.type == MD_MAT_COLOR) {
                std::memcpy(info.colour, mat.colour, 4);
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

        // Walk BSP tree for each sub-object, collecting polygons per material
        // Key: material index, Value: list of triangulated indices
        mResult = &result;
        loadSubObject(0, -1);

        // Build submesh ranges from per-material triangle lists
        for (auto &kv : mMatTriangles) {
            BinSubMesh sm;
            sm.matIndex = kv.first;
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

            // Get sub-object transform for non-root sub-objects
            const SubObjectHeader &sob = mSubObjects[mCurrentSubObj];

            // Emit vertices and fan-triangulate
            // Use the same winding as ManualBinFileLoader: (last, i, i-1)
            // which is equivalent to fan from last vertex
            uint32_t baseVert = static_cast<uint32_t>(mResult->vertices.size());

            for (int vi = 0; vi < op.num_verts; ++vi) {
                BinVert bv = {};

                // Position from vertex table
                if (vertIndices[vi] < mVertices.size()) {
                    const Vertex &v = mVertices[vertIndices[vi]];
                    if (mCurrentSubObj == 0) {
                        // Root sub-object — no transform needed
                        bv.x = v.x;
                        bv.y = v.y;
                        bv.z = v.z;
                    } else {
                        // Apply sub-object transform: rotate + translate
                        // rot is a 3x3 matrix stored column-major in the struct
                        const float *r = sob.trans.rot;
                        const Vertex &axle = sob.trans.axle_point;
                        bv.x = r[0]*v.x + r[3]*v.y + r[6]*v.z + axle.x;
                        bv.y = r[1]*v.x + r[4]*v.y + r[7]*v.z + axle.y;
                        bv.z = r[2]*v.x + r[5]*v.y + r[8]*v.z + axle.z;
                    }
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
            auto &triList = mMatTriangles[matIndex];
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
    std::map<int, int> mSlotToMat; // slot_num -> material index
    std::vector<UVMap> mUVs;
    std::vector<Vertex> mVertices;
    std::vector<ObjLight> mLights;
    std::vector<Vertex> mNormals;         // unpacked from ObjLight
    std::vector<SubObjectHeader> mSubObjects;

    // Per-material triangle index lists, built during BSP walk
    std::map<int, std::vector<uint32_t>> mMatTriangles;

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

} // namespace Opde
