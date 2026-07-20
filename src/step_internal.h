/*
 * step_internal.h
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
// step_internal.h
// Shared geometric data model between the STEP Part 21 parser
// (step_loader.cpp) and the surface tessellator (step_tess.cpp).
// Internal to the STEP module; not part of the public viewer API.

#include <cmath>
#include <string>
#include <vector>

#include "mesh.h"

namespace step {

// ---------------------------------------------------------------------------
// Small double-precision vector math (tessellation runs in double, the mesh
// is converted to float only at the very end).
// ---------------------------------------------------------------------------

struct Vec3d {
    double x = 0.0, y = 0.0, z = 0.0;
};

inline Vec3d operator+(const Vec3d& a, const Vec3d& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3d operator-(const Vec3d& a, const Vec3d& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3d operator-(const Vec3d& a) { return {-a.x, -a.y, -a.z}; }
inline Vec3d operator*(const Vec3d& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3d operator*(double s, const Vec3d& a) { return a * s; }
inline double dot(const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3d cross(const Vec3d& a, const Vec3d& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(const Vec3d& a) { return std::sqrt(dot(a, a)); }
inline Vec3d normalized(const Vec3d& a) {
    double n = norm(a);
    return (n > 1e-300) ? a * (1.0 / n) : Vec3d{0.0, 0.0, 1.0};
}
inline double dist(const Vec3d& a, const Vec3d& b) { return norm(a - b); }

// Right-handed orthonormal placement frame (from AXIS2_PLACEMENT_3D etc.).
struct Frame {
    Vec3d o;                                        // origin / location
    Vec3d ax{1, 0, 0}, ay{0, 1, 0}, az{0, 0, 1};    // local X, Y, Z axes
};

// ---------------------------------------------------------------------------
// Affine placement transform (assembly instancing)
// ---------------------------------------------------------------------------

// Column-form affine transform: p' = cx*p.x + cy*p.y + cz*p.z + t.
// For STEP assemblies this is rigid (rotation + translation), possibly with a
// uniform scale from CARTESIAN_TRANSFORMATION_OPERATOR_3D. Defaults to
// identity.
struct Mat43 {
    Vec3d cx{1, 0, 0}, cy{0, 1, 0}, cz{0, 0, 1}, t{0, 0, 0};
};

inline Vec3d xfVector(const Mat43& m, const Vec3d& v) {
    return m.cx * v.x + m.cy * v.y + m.cz * v.z;
}

inline Vec3d xfPoint(const Mat43& m, const Vec3d& p) {
    return xfVector(m, p) + m.t;
}

// Composition a∘b: apply b first, then a.
inline Mat43 xfCompose(const Mat43& a, const Mat43& b) {
    Mat43 r;
    r.cx = xfVector(a, b.cx);
    r.cy = xfVector(a, b.cy);
    r.cz = xfVector(a, b.cz);
    r.t = xfPoint(a, b.t);
    return r;
}

inline Mat43 xfFromFrame(const Frame& f) {
    Mat43 m;
    m.cx = f.ax;
    m.cy = f.ay;
    m.cz = f.az;
    m.t = f.o;
    return m;
}

inline double xfDet(const Mat43& m) {
    return dot(m.cx, cross(m.cy, m.cz));
}

// General affine inverse via the adjugate; sets ok=false (and returns the
// identity) for a singular linear part.
inline Mat43 xfInverse(const Mat43& m, bool& ok) {
    double det = xfDet(m);
    if (std::fabs(det) < 1e-12) {
        ok = false;
        return Mat43{};
    }
    ok = true;
    double inv = 1.0 / det;
    // rows of the inverse linear part
    Vec3d r0 = cross(m.cy, m.cz) * inv;
    Vec3d r1 = cross(m.cz, m.cx) * inv;
    Vec3d r2 = cross(m.cx, m.cy) * inv;
    Mat43 o;
    o.cx = {r0.x, r1.x, r2.x};
    o.cy = {r0.y, r1.y, r2.y};
    o.cz = {r0.z, r1.z, r2.z};
    o.t = -xfVector(o, m.t);
    return o;
}

inline bool xfIsIdentity(const Mat43& m) {
    return m.cx.x == 1.0 && m.cx.y == 0.0 && m.cx.z == 0.0 &&
           m.cy.x == 0.0 && m.cy.y == 1.0 && m.cy.z == 0.0 &&
           m.cz.x == 0.0 && m.cz.y == 0.0 && m.cz.z == 1.0 &&
           m.t.x == 0.0 && m.t.y == 0.0 && m.t.z == 0.0;
}

// ---------------------------------------------------------------------------
// Curves (3D edge geometry and profile curves of swept surfaces)
// ---------------------------------------------------------------------------

enum class CurveKind {
    None,      // missing / unsupported: fall back to straight vertex segment
    Line,
    Circle,
    Ellipse,
    BSpline,
    Polyline
};

struct BSCurve {
    int degree = 1;
    bool closed = false;
    std::vector<Vec3d> ctrl;
    std::vector<double> weights;   // empty -> non-rational
    std::vector<double> knots;     // full knot vector (size = ctrl + degree + 1)
};

struct Curve {
    CurveKind kind = CurveKind::None;
    Frame frame;               // circle / ellipse placement
    double r1 = 0, r2 = 0;     // radius / semi-axes
    Vec3d p0, dir;             // line: point + direction (magnitude included)
    BSCurve bs;                // b-spline data
    std::vector<Vec3d> poly;   // polyline points
};

// ---------------------------------------------------------------------------
// Surfaces
// ---------------------------------------------------------------------------

enum class SurfKind {
    None,
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    Extrusion,     // SURFACE_OF_LINEAR_EXTRUSION
    Revolution,    // SURFACE_OF_REVOLUTION
    BSpline
};

struct BSSurf {
    int udeg = 1, vdeg = 1;
    int nu = 0, nv = 0;            // control net size (u-major)
    bool uclosed = false, vclosed = false;
    std::vector<Vec3d> ctrl;       // ctrl[i * nv + j], i along u
    std::vector<double> weights;   // empty -> non-rational
    std::vector<double> uknots, vknots;
};

struct Surface {
    SurfKind kind = SurfKind::None;
    Frame frame;                   // placement (plane/cyl/cone/sphere/torus, revolution axis)
    double r1 = 0;                 // radius / major radius
    double r2 = 0;                 // minor radius (torus) / second semi-axis
    double angle = 0;              // cone semi-angle (radians)
    Curve base;                    // profile curve for extrusion / revolution
    Vec3d sweepDir;                // extrusion direction (magnitude included)
    BSSurf bs;
};

// ---------------------------------------------------------------------------
// Topology
// ---------------------------------------------------------------------------

struct Edge {
    Curve curve;                 // CurveKind::None -> straight segment pstart..pend
    Vec3d pstart, pend;          // vertex points (raw, in curve order)
    bool curveSense = true;      // EDGE_CURVE.same_sense
    bool orientation = true;     // ORIENTED_EDGE.orientation (false -> traverse reversed)
};

struct Loop {
    bool outer = false;          // came from FACE_OUTER_BOUND
    std::vector<Edge> edges;
};

struct Face {
    Surface surf;
    bool sameSense = true;       // ADVANCED_FACE.same_sense
    std::vector<Loop> loops;
};

// One unique piece of geometry (typically the faces of one shape
// representation), tessellated once in its local coordinates and replicated
// for every placed instance.
struct Part {
    std::vector<Face> faces;
    std::vector<Mat43> instances;   // one entry per placed copy (>= 1)
};

// One node of the logical model tree (product structure / multi-body files),
// resolved by the parser. Nodes form a tree via 'parent' indices and a parent
// always precedes its children in Model::nodes. Leaf nodes reference one
// placed instance (parts[part].instances[instance]); group nodes keep
// part == -1. The tessellator converts this tree into Mesh::nodes, filling in
// the triangle ranges and pruning nodes whose geometry produced no triangles.
// An empty vector means a single unnamed object.
struct ModelNode {
    std::string name;      // UTF-8 display name (product / occurrence / body)
    int parent = -1;       // index into Model::nodes, -1 = top level
    int part = -1;         // leaf: index into Model::parts, -1 = group node
    int instance = -1;     // leaf: index into parts[part].instances
};

struct Model {
    std::vector<Part> parts;
    std::vector<ModelNode> nodes;   // component tree (may be empty)
    int totalFaces = 0;          // unique faces found in the file
    int skippedFaces = 0;        // faces that could not be converted
    int assemblyWarnings = 0;    // missing/malformed assembly transforms
};

// ---------------------------------------------------------------------------
// Module entry points
// ---------------------------------------------------------------------------

// Parse a STEP Part 21 file and build the geometric model (step_loader.cpp).
bool parseStep(const std::string& path, Model& model, std::string& err);

// Tessellate the model into a triangle mesh (step_tess.cpp).
// facesOk / facesApprox / facesFailed are optional out-counters
// (approx = face emitted through the untrimmed-domain fallback).
bool tessellate(const Model& model, Mesh& out, std::string& err,
                int* facesOk, int* facesApprox, int* facesFailed);

} // namespace step
