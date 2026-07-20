/*
 * obj_loader.cpp
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

// obj_loader.cpp -- Wavefront OBJ loader for the 3dt viewer.
//
// Output is a triangle soup (see mesh.h): 3 consecutive positions per
// triangle, with an optional parallel normals array.
//
// Supported directives:
//   v  x y z            (optional w / vertex-color extras are ignored)
//   vn x y z            (normalized on load)
//   f  i i i ...        index forms: v, v/vt, v//vn, v/vt/vn, freely mixed
//                       within one face; 1-based and negative (end-relative)
//                       indices; quads and convex n-gons are triangulated
//                       with a fan (v0, vi, vi+1).
// Silently ignored: vt, vp, o, g, s, mtllib, usemtl, l, p, comments (#),
// blank lines. Backslash line continuations, \r\n endings, tabs and
// repeated whitespace are handled. Parsing works directly on a char buffer
// with strtof/strtol (no per-line stringstreams) so multi-million-face
// files load quickly.
//
// Faces with malformed or out-of-range indices are skipped and counted;
// loading fails (with the skip count in 'err') only if no triangle at all
// could be produced.

#include "mesh.h"

#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

inline bool isSpaceTab(char c) {
    return c == ' ' || c == '\t' || c == '\r';
}

inline const char* skipWs(const char* p, const char* end) {
    while (p < end && isSpaceTab(*p)) ++p;
    return p;
}

// Resolve a 1-based (positive) or end-relative (negative) OBJ index into a
// 0-based index; returns false if the index is 0 or out of range.
inline bool resolveIndex(long idx, size_t count, size_t& out) {
    if (idx > 0) {
        if (static_cast<unsigned long>(idx) > count) return false;
        out = static_cast<size_t>(idx - 1);
        return true;
    }
    if (idx < 0) {
        if (idx == LONG_MIN) return false;
        if (static_cast<unsigned long>(-idx) > count) return false;
        out = count - static_cast<size_t>(-idx);
        return true;
    }
    return false; // index 0 is invalid in OBJ
}

inline Vec3f triNormal(const Vec3f& a, const Vec3f& b, const Vec3f& c) {
    const float ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
    const float vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
    float nx = uy * vz - uz * vy;
    float ny = uz * vx - ux * vz;
    float nz = ux * vy - uy * vx;
    const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len > 1e-20f) return {nx / len, ny / len, nz / len};
    return {0.f, 0.f, 1.f};
}

// Parse exactly n whitespace-separated floats from [p, end).
inline bool parseFloats(const char*& p, const char* end, float* dst, int n) {
    for (int i = 0; i < n; ++i) {
        p = skipWs(p, end);
        if (p >= end) return false;
        char* ep = nullptr;
        const float v = std::strtof(p, &ep);
        if (ep == p || ep > end) return false;
        dst[i] = v;
        p = ep;
    }
    return true;
}

} // namespace

bool loadOBJ(const std::string& path, Mesh& out, std::string& err) {
    err.clear();
    out.positions.clear();
    out.normals.clear();

    // Read the whole file into memory in one shot.
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) {
        err = "OBJ: cannot open file '" + path + "'";
        return false;
    }
    std::string buf;
    {
        std::fseek(f, 0, SEEK_END);
        const long sz = std::ftell(f);
        std::fseek(f, 0, SEEK_SET);
        if (sz > 0) {
            buf.resize(static_cast<size_t>(sz));
            const size_t got = std::fread(&buf[0], 1, buf.size(), f);
            buf.resize(got);
        }
        std::fclose(f);
    }
    if (buf.empty()) {
        err = "OBJ: file is empty: '" + path + "'";
        return false;
    }

    // Splice physical lines joined by a trailing backslash (line
    // continuation): replace the backslash and the newline with spaces so
    // the logical line is parsed as one.
    for (size_t i = 0; i + 1 < buf.size(); ++i) {
        if (buf[i] != '\\') continue;
        if (buf[i + 1] == '\n') {
            buf[i] = ' ';
            buf[i + 1] = ' ';
        } else if (buf[i + 1] == '\r' && i + 2 < buf.size() &&
                   buf[i + 2] == '\n') {
            buf[i] = ' ';
            buf[i + 1] = ' ';
            buf[i + 2] = ' ';
        }
    }

    std::vector<Vec3f> verts;
    std::vector<Vec3f> norms;
    // Rough estimates from the file size ("v -0.123456 ..." is ~25-40
    // bytes; a face line yields >= 3 output vertices).
    verts.reserve(buf.size() / 40 + 16);
    norms.reserve(buf.size() / 40 + 16);
    out.positions.reserve(buf.size() / 16 + 64);

    std::vector<size_t> faceV; // resolved position indices of current face
    std::vector<long> faceN;   // resolved normal indices, -1 if absent
    faceV.reserve(8);
    faceN.reserve(8);

    size_t skippedFaces = 0;
    bool writingNormals = false; // set at the first face referencing vn
    size_t lineNo = 0;

    const char* p = buf.data();
    const char* fileEnd = p + buf.size();

    while (p < fileEnd) {
        ++lineNo;
        const char* nl = static_cast<const char*>(
            std::memchr(p, '\n', static_cast<size_t>(fileEnd - p)));
        const char* lineEnd = nl ? nl : fileEnd;
        const char* next = nl ? nl + 1 : fileEnd;
        while (lineEnd > p && lineEnd[-1] == '\r') --lineEnd;

        const char* q = skipWs(p, lineEnd);
        p = next;
        if (q >= lineEnd || *q == '#') continue; // blank line or comment

        if (*q == 'v') {
            if (q + 1 < lineEnd && (q[1] == ' ' || q[1] == '\t')) {
                // v x y z [w] [r g b] -- extra values are ignored
                const char* t = q + 1;
                float xyz[3];
                if (!parseFloats(t, lineEnd, xyz, 3)) {
                    err = "OBJ: malformed 'v' directive at line " +
                          std::to_string(lineNo) + " in '" + path + "'";
                    return false;
                }
                verts.push_back({xyz[0], xyz[1], xyz[2]});
            } else if (q + 2 < lineEnd && q[1] == 'n' &&
                       (q[2] == ' ' || q[2] == '\t')) {
                const char* t = q + 2;
                float xyz[3];
                if (!parseFloats(t, lineEnd, xyz, 3)) {
                    err = "OBJ: malformed 'vn' directive at line " +
                          std::to_string(lineNo) + " in '" + path + "'";
                    return false;
                }
                const float len = std::sqrt(xyz[0] * xyz[0] +
                                            xyz[1] * xyz[1] +
                                            xyz[2] * xyz[2]);
                if (len > 1e-20f) {
                    xyz[0] /= len;
                    xyz[1] /= len;
                    xyz[2] /= len;
                }
                norms.push_back({xyz[0], xyz[1], xyz[2]});
            }
            // else: vt, vp, bare "v" -> ignored
            continue;
        }

        if (*q != 'f' || q + 1 >= lineEnd || (q[1] != ' ' && q[1] != '\t'))
            continue; // o, g, s, mtllib, usemtl, l, p, unknown -> ignored

        // ---- face ----
        faceV.clear();
        faceN.clear();
        bool bad = false;
        const char* t = q + 1;
        while (true) {
            t = skipWs(t, lineEnd);
            if (t >= lineEnd) break;
            char* ep = nullptr;
            const long vi = std::strtol(t, &ep, 10);
            if (ep == t || ep > lineEnd) {
                bad = true;
                break;
            }
            t = ep;
            long ni = 0;
            bool hasN = false;
            if (t < lineEnd && *t == '/') {
                ++t; // v/...
                if (t < lineEnd && *t != '/' && !isSpaceTab(*t)) {
                    (void)std::strtol(t, &ep, 10); // vt index, ignored
                    if (ep == t) {
                        bad = true;
                        break;
                    }
                    t = ep;
                }
                if (t < lineEnd && *t == '/') {
                    ++t; // .../vn
                    ni = std::strtol(t, &ep, 10);
                    if (ep == t || ep > lineEnd) {
                        bad = true;
                        break;
                    }
                    t = ep;
                    hasN = true;
                }
            }
            if (t < lineEnd && !isSpaceTab(*t)) {
                bad = true; // junk glued to the token
                break;
            }

            size_t vIdx = 0;
            if (!resolveIndex(vi, verts.size(), vIdx)) {
                bad = true;
                break;
            }
            long nIdx = -1;
            if (hasN) {
                size_t tmp = 0;
                if (!resolveIndex(ni, norms.size(), tmp)) {
                    bad = true;
                    break;
                }
                nIdx = static_cast<long>(tmp);
            }
            faceV.push_back(vIdx);
            faceN.push_back(nIdx);
        }

        if (bad || faceV.size() < 3) {
            ++skippedFaces;
            continue;
        }

        // First face that carries vn references: switch to explicit
        // normals and backfill triangles emitted so far with their
        // geometric (flat) normals.
        bool faceHasN = false;
        for (long n : faceN) {
            if (n >= 0) {
                faceHasN = true;
                break;
            }
        }
        if (faceHasN && !writingNormals) {
            writingNormals = true;
            out.normals.reserve(out.positions.capacity());
            for (size_t i = 0; i + 2 < out.positions.size(); i += 3) {
                const Vec3f gn = triNormal(out.positions[i],
                                           out.positions[i + 1],
                                           out.positions[i + 2]);
                out.normals.push_back(gn);
                out.normals.push_back(gn);
                out.normals.push_back(gn);
            }
        }

        // Fan triangulation: (v0, vi, vi+1) -- fine for convex polygons.
        for (size_t i = 1; i + 1 < faceV.size(); ++i) {
            const size_t corner[3] = {0, i, i + 1};
            const Vec3f a = verts[faceV[0]];
            const Vec3f b = verts[faceV[i]];
            const Vec3f c = verts[faceV[i + 1]];
            out.positions.push_back(a);
            out.positions.push_back(b);
            out.positions.push_back(c);
            if (writingNormals) {
                Vec3f gn{};
                bool gnReady = false;
                for (size_t k : corner) {
                    const long n = faceN[k];
                    if (n >= 0) {
                        out.normals.push_back(norms[static_cast<size_t>(n)]);
                    } else {
                        // Corner without vn: fall back to the geometric
                        // normal of this triangle.
                        if (!gnReady) {
                            gn = triNormal(a, b, c);
                            gnReady = true;
                        }
                        out.normals.push_back(gn);
                    }
                }
            }
        }
    }

    if (out.positions.empty()) {
        if (skippedFaces > 0) {
            err = "OBJ: no valid geometry in '" + path + "' (" +
                  std::to_string(skippedFaces) +
                  " face(s) skipped due to malformed or out-of-range indices)";
        } else {
            err = "OBJ: no geometry ('f' faces) found in '" + path + "'";
        }
        return false;
    }

    if (!writingNormals) {
        // File had no usable vn at all.
        out.normals.clear();
        out.computeNormalsIfMissing();
    }
    out.computeBounds();
    return true;
}
