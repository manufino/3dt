/*
 * mesh.h
 *
 * Copyright (C) 2023-2026 Manuel Finessi
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once
// Shared mesh interface for the 3dt viewer.
// Triangle-soup mesh: 3 consecutive vertices per triangle.

#include <string>
#include <vector>

struct Vec3f {
    float x = 0.f, y = 0.f, z = 0.f;
};

// One node of the model tree (assemblies/parts). Nodes form a tree via
// 'parent' indices (parent always precedes its children in Mesh::nodes).
// Leaf nodes own a contiguous triangle range [triStart, triStart+triCount);
// pure group nodes have triCount == 0. An empty Mesh::nodes vector means a
// single unnamed object covering all triangles.
struct MeshNode {
    std::string name;      // UTF-8 display name (product/part name)
    int parent = -1;       // index into Mesh::nodes, -1 = top level
    size_t triStart = 0;
    size_t triCount = 0;
    bool visible = true;   // runtime visibility toggle (UI state)
};

struct Mesh {
    // size of both vectors = 3 * triangleCount (normals may be empty before
    // computeNormalsIfMissing() is called)
    std::vector<Vec3f> positions;
    std::vector<Vec3f> normals;

    // Model tree for multi-object files (STEP assemblies). May be empty.
    std::vector<MeshNode> nodes;

    Vec3f bboxMin{}, bboxMax{};

    size_t triangleCount() const { return positions.size() / 3; }

    // Recompute bboxMin/bboxMax from positions.
    void computeBounds();

    // If normals is empty or mismatched, fill with flat per-face normals.
    void computeNormalsIfMissing();
};

// Loaders: return true on success and fill 'out'; on failure return false
// and put a human-readable message in 'err'.
bool loadSTL(const std::string& path, Mesh& out, std::string& err);
bool loadSTEP(const std::string& path, Mesh& out, std::string& err);
bool loadOBJ(const std::string& path, Mesh& out, std::string& err);
