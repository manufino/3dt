/*
 * stl_loader.cpp
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

// STL loader (ASCII and binary) for the 3dt viewer.
// Implements loadSTL() declared in mesh.h. C++17, standard library only.

#include "mesh.h"

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace {

constexpr std::uint64_t kBinaryHeaderSize = 84;          // 80-byte header + uint32 count
constexpr std::uint64_t kTriangleRecordSize = 50;        // 12 floats + uint16 attribute
constexpr std::uint32_t kMaxTriangles = 200'000'000;     // sanity limit

// Fix up normals after parsing: if the normal array is missing or mismatched,
// compute flat per-face normals; if present, replace near-zero normals with
// the geometric normal of the triangle.
void finalizeNormals(Mesh& mesh) {
    if (mesh.normals.size() != mesh.positions.size()) {
        mesh.normals.clear();
        mesh.computeNormalsIfMissing();
        return;
    }
    for (size_t i = 0; i + 2 < mesh.positions.size(); i += 3) {
        const Vec3f& n = mesh.normals[i];
        float len2 = n.x * n.x + n.y * n.y + n.z * n.z;
        if (len2 > 1e-12f) continue; // declared normal looks usable
        const Vec3f& a = mesh.positions[i];
        const Vec3f& b = mesh.positions[i + 1];
        const Vec3f& c = mesh.positions[i + 2];
        float ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
        float vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
        float nx = uy * vz - uz * vy;
        float ny = uz * vx - ux * vz;
        float nz = ux * vy - uy * vx;
        float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        Vec3f g{0.f, 0.f, 1.f};
        if (len > 1e-20f) g = {nx / len, ny / len, nz / len};
        mesh.normals[i] = mesh.normals[i + 1] = mesh.normals[i + 2] = g;
    }
}

std::uint32_t readU32LE(const unsigned char* p) {
    // x86 is little-endian; memcpy avoids type-punning UB and works anyway
    // because we read byte by byte.
    return static_cast<std::uint32_t>(p[0]) |
           (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) |
           (static_cast<std::uint32_t>(p[3]) << 24);
}

bool parseBinary(std::ifstream& file, std::uint64_t fileSize, std::uint32_t count,
                 Mesh& out, std::string& err) {
    if (count == 0) {
        err = "Binary STL contains zero triangles";
        return false;
    }
    if (count > kMaxTriangles) {
        err = "Binary STL triangle count is implausibly large (" +
              std::to_string(count) + ")";
        return false;
    }
    if (fileSize < kBinaryHeaderSize + kTriangleRecordSize * count) {
        err = "Binary STL file is truncated (header declares " +
              std::to_string(count) + " triangles)";
        return false;
    }

    file.clear();
    file.seekg(static_cast<std::streamoff>(kBinaryHeaderSize), std::ios::beg);

    out.positions.clear();
    out.normals.clear();
    out.positions.reserve(static_cast<size_t>(count) * 3);
    out.normals.reserve(static_cast<size_t>(count) * 3);

    // Read triangle records in chunks to avoid holding the whole file in memory.
    constexpr size_t kChunkTris = 16384;
    std::vector<char> buf(kChunkTris * kTriangleRecordSize);

    std::uint32_t remaining = count;
    while (remaining > 0) {
        const size_t n = remaining < kChunkTris ? remaining : kChunkTris;
        const std::streamsize bytes =
            static_cast<std::streamsize>(n * kTriangleRecordSize);
        file.read(buf.data(), bytes);
        if (file.gcount() != bytes) {
            err = "Unexpected end of file while reading binary STL triangles";
            return false;
        }
        const char* rec = buf.data();
        for (size_t i = 0; i < n; ++i, rec += kTriangleRecordSize) {
            float f[12]; // normal + 3 vertices
            std::memcpy(f, rec, sizeof(f));
            out.normals.push_back({f[0], f[1], f[2]});
            out.normals.push_back({f[0], f[1], f[2]});
            out.normals.push_back({f[0], f[1], f[2]});
            out.positions.push_back({f[3], f[4], f[5]});
            out.positions.push_back({f[6], f[7], f[8]});
            out.positions.push_back({f[9], f[10], f[11]});
            // 2 trailing attribute bytes are ignored.
        }
        remaining -= static_cast<std::uint32_t>(n);
    }
    return true;
}

bool isIdentChar(char c) {
    return std::isalpha(static_cast<unsigned char>(c)) != 0;
}

// Tolerant ASCII parser. Scans keywords case-insensitively, accepts any
// whitespace, missing solid names and scientific notation. Returns true if
// the text was structurally parseable; triangles found may still be zero.
bool parseAscii(const std::string& text, Mesh& out, std::string& err) {
    out.positions.clear();
    out.normals.clear();

    const char* p = text.c_str();
    const char* const end = p + text.size();

    Vec3f currentNormal{};
    size_t approxTris = text.size() / 200 + 16; // rough guess for reserve
    out.positions.reserve(approxTris * 3);
    out.normals.reserve(approxTris * 3);

    auto skipWs = [&]() {
        while (p < end && std::isspace(static_cast<unsigned char>(*p))) ++p;
    };
    auto skipLine = [&]() {
        while (p < end && *p != '\n') ++p;
    };
    // Read the next alphabetic keyword, lowercased. Non-keyword garbage is
    // skipped one character at a time.
    auto readKeyword = [&](std::string& w) -> bool {
        skipWs();
        if (p >= end) return false;
        w.clear();
        if (!isIdentChar(*p)) {
            ++p; // skip unexpected character, stay tolerant
            return true;
        }
        while (p < end && isIdentChar(*p)) {
            w.push_back(static_cast<char>(
                std::tolower(static_cast<unsigned char>(*p))));
            ++p;
        }
        return true;
    };
    // Parse one float at the cursor. strtof handles signs and exponents;
    // text.c_str() guarantees null termination at the end of the buffer.
    auto readFloat = [&](float& v) -> bool {
        skipWs();
        if (p >= end) return false;
        char* stop = nullptr;
        v = std::strtof(p, &stop);
        if (stop == p) return false;
        p = stop;
        return true;
    };

    std::string word;
    while (readKeyword(word)) {
        if (word == "vertex") {
            Vec3f v;
            if (!readFloat(v.x) || !readFloat(v.y) || !readFloat(v.z)) {
                err = "Malformed vertex line in ASCII STL";
                return false;
            }
            out.positions.push_back(v);
            out.normals.push_back(currentNormal);
        } else if (word == "facet") {
            // Expect "normal nx ny nz"; tolerate a facet without one.
            const char* save = p;
            std::string kw;
            Vec3f n{};
            if (readKeyword(kw) && kw == "normal" && readFloat(n.x) &&
                readFloat(n.y) && readFloat(n.z)) {
                currentNormal = n;
            } else {
                currentNormal = Vec3f{};
                p = save;
            }
        } else if (word == "solid" || word == "endsolid") {
            skipLine(); // name is optional and may contain arbitrary words
        }
        // "outer", "loop", "endloop", "endfacet" and anything else: ignore.
    }

    // Drop a trailing incomplete triangle, if any.
    size_t usable = (out.positions.size() / 3) * 3;
    out.positions.resize(usable);
    out.normals.resize(usable);
    return true;
}

} // namespace

bool loadSTL(const std::string& path, Mesh& out, std::string& err) {
    err.clear();
    out.positions.clear();
    out.normals.clear();

    std::ifstream file(path, std::ios::binary);
    if (!file) {
        err = "Cannot open file: " + path;
        return false;
    }

    file.seekg(0, std::ios::end);
    const std::streamoff sizeOff = file.tellg();
    if (sizeOff < 0) {
        err = "Cannot determine file size: " + path;
        return false;
    }
    const std::uint64_t fileSize = static_cast<std::uint64_t>(sizeOff);
    file.seekg(0, std::ios::beg);

    if (fileSize == 0) {
        err = "File is empty: " + path;
        return false;
    }

    // Read the binary header (if the file is big enough) to check whether the
    // declared triangle count matches the file size exactly. This is far more
    // reliable than checking for a leading "solid" keyword, since many binary
    // STL files start with "solid" in their 80-byte header.
    std::uint32_t binCount = 0;
    bool haveBinHeader = false;
    if (fileSize >= kBinaryHeaderSize) {
        unsigned char header[84];
        file.read(reinterpret_cast<char*>(header), sizeof(header));
        if (file.gcount() == static_cast<std::streamsize>(sizeof(header))) {
            binCount = readU32LE(header + 80);
            haveBinHeader = true;
        }
        file.clear();
        file.seekg(0, std::ios::beg);
    }

    const bool sizeMatchesBinary =
        haveBinHeader &&
        fileSize == kBinaryHeaderSize + kTriangleRecordSize *
                        static_cast<std::uint64_t>(binCount);

    if (sizeMatchesBinary) {
        if (!parseBinary(file, fileSize, binCount, out, err)) return false;
    } else {
        // Try ASCII. Read the whole file into memory; ASCII STL files are
        // typically much smaller than binary ones.
        std::string text;
        text.resize(static_cast<size_t>(fileSize));
        file.read(&text[0], static_cast<std::streamsize>(fileSize));
        if (file.gcount() != static_cast<std::streamsize>(fileSize)) {
            err = "Failed to read file: " + path;
            return false;
        }

        std::string asciiErr;
        const bool asciiOk = parseAscii(text, out, asciiErr) &&
                             !out.positions.empty();
        if (!asciiOk) {
            // ASCII yielded nothing: retry as binary if the header count is
            // at least consistent with the file size (extra trailing bytes
            // are tolerated here).
            if (haveBinHeader && binCount > 0 && binCount <= kMaxTriangles &&
                fileSize >= kBinaryHeaderSize +
                                kTriangleRecordSize *
                                    static_cast<std::uint64_t>(binCount)) {
                if (!parseBinary(file, fileSize, binCount, out, err))
                    return false;
            } else {
                err = asciiErr.empty()
                          ? "File is not a valid STL (no triangles found): " +
                                path
                          : asciiErr;
                return false;
            }
        }
    }

    finalizeNormals(out);
    out.computeBounds();
    return true;
}
