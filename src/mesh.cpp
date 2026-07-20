/*
 * mesh.cpp
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

#include "mesh.h"

#include <cmath>
#include <limits>

void Mesh::computeBounds() {
    const float inf = std::numeric_limits<float>::infinity();
    bboxMin = {inf, inf, inf};
    bboxMax = {-inf, -inf, -inf};
    for (const Vec3f& p : positions) {
        if (p.x < bboxMin.x) bboxMin.x = p.x;
        if (p.y < bboxMin.y) bboxMin.y = p.y;
        if (p.z < bboxMin.z) bboxMin.z = p.z;
        if (p.x > bboxMax.x) bboxMax.x = p.x;
        if (p.y > bboxMax.y) bboxMax.y = p.y;
        if (p.z > bboxMax.z) bboxMax.z = p.z;
    }
    if (positions.empty()) {
        bboxMin = bboxMax = {0.f, 0.f, 0.f};
    }
}

void Mesh::computeNormalsIfMissing() {
    if (normals.size() == positions.size() && !normals.empty()) return;
    normals.assign(positions.size(), Vec3f{0.f, 0.f, 1.f});
    for (size_t i = 0; i + 2 < positions.size(); i += 3) {
        const Vec3f& a = positions[i];
        const Vec3f& b = positions[i + 1];
        const Vec3f& c = positions[i + 2];
        float ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
        float vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
        float nx = uy * vz - uz * vy;
        float ny = uz * vx - ux * vz;
        float nz = ux * vy - uy * vx;
        float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        Vec3f n{0.f, 0.f, 1.f};
        if (len > 1e-20f) n = {nx / len, ny / len, nz / len};
        normals[i] = normals[i + 1] = normals[i + 2] = n;
    }
}
