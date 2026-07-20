/*
 * step_tess.cpp
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

// step_tess.cpp
// Surface evaluation, trimming and triangulation for the STEP loader.
//
// Per face: sample boundary loops in 3D, project them into the (u,v)
// parameter space of the supporting surface (with seam unwrapping for
// periodic surfaces), triangulate the trimmed region (ear clipping with
// hole bridging), refine large triangles with a conforming edge-split
// scheme so curved surfaces stay smooth, then evaluate positions and
// analytic normals at every vertex. All math in double; converted to
// float only when appended to the Mesh.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "mesh.h"
#include "step_internal.h"

namespace {

using step::Vec3d;
using step::Frame;
using step::Curve;
using step::CurveKind;
using step::Surface;
using step::SurfKind;
using step::BSCurve;
using step::BSSurf;

const double kPi = 3.14159265358979323846;
const double kTwoPi = 2.0 * kPi;
const double kAngStep = kPi / 24.0;      // ~7.5 degrees per curved segment
const double kBig = 1e30;

struct V2 {
    double u = 0, v = 0;
};

double clampd(double x, double lo, double hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

// Move 'val' to the branch of its period closest to 'ref'.
double unwrapNear(double val, double ref, double period) {
    if (period <= 0) return val;
    double d = val - ref;
    d -= period * std::floor(d / period + 0.5);
    return ref + d;
}

bool nearly3(const Vec3d& a, const Vec3d& b) {
    double tol = 1e-6 * (1.0 + step::norm(a) + step::norm(b));
    return step::dist(a, b) <= tol;
}

// ---------------------------------------------------------------------------
// B-spline evaluation (Cox-de Boor)
// ---------------------------------------------------------------------------

int findSpan(int n, int deg, double t, const std::vector<double>& U) {
    // n = number of control points - 1
    if (t >= U[static_cast<size_t>(n + 1)]) return n;
    if (t <= U[static_cast<size_t>(deg)]) return deg;
    int lo = deg, hi = n + 1, mid = (lo + hi) / 2;
    while (t < U[static_cast<size_t>(mid)] || t >= U[static_cast<size_t>(mid + 1)]) {
        if (t < U[static_cast<size_t>(mid)]) hi = mid;
        else lo = mid;
        mid = (lo + hi) / 2;
    }
    return mid;
}

void basisFuns(int span, double t, int deg, const std::vector<double>& U, double* N) {
    double left[16], right[16];
    N[0] = 1.0;
    for (int j = 1; j <= deg; ++j) {
        left[j] = t - U[static_cast<size_t>(span + 1 - j)];
        right[j] = U[static_cast<size_t>(span + j)] - t;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            double denom = right[r + 1] + left[j - r];
            double temp = (std::fabs(denom) > 1e-300) ? N[r] / denom : 0.0;
            N[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        N[j] = saved;
    }
}

double wrapClampParam(double t, double t0, double t1, bool closed) {
    if (t1 <= t0) return t0;
    if (closed) {
        double T = t1 - t0;
        t = std::fmod(t - t0, T);
        if (t < 0) t += T;
        return t + t0;
    }
    return clampd(t, t0, t1);
}

Vec3d evalBSCurve(const BSCurve& c, double t) {
    int deg = c.degree;
    int n = static_cast<int>(c.ctrl.size()) - 1;
    double t0 = c.knots[static_cast<size_t>(deg)];
    double t1 = c.knots[c.knots.size() - 1 - static_cast<size_t>(deg)];
    t = wrapClampParam(t, t0, t1, c.closed);
    int span = findSpan(n, deg, t, c.knots);
    double N[16];
    basisFuns(span, t, deg, c.knots, N);
    Vec3d acc{0, 0, 0};
    double wsum = 0.0;
    for (int i = 0; i <= deg; ++i) {
        int ci = span - deg + i;
        double w = c.weights.empty() ? 1.0 : c.weights[static_cast<size_t>(ci)];
        double bw = N[i] * w;
        acc = acc + c.ctrl[static_cast<size_t>(ci)] * bw;
        wsum += bw;
    }
    return acc * (1.0 / std::max(wsum, 1e-300));
}

Vec3d evalBSSurf(const BSSurf& b, double u, double v) {
    double u0 = b.uknots[static_cast<size_t>(b.udeg)];
    double u1 = b.uknots[b.uknots.size() - 1 - static_cast<size_t>(b.udeg)];
    double v0 = b.vknots[static_cast<size_t>(b.vdeg)];
    double v1 = b.vknots[b.vknots.size() - 1 - static_cast<size_t>(b.vdeg)];
    u = wrapClampParam(u, u0, u1, b.uclosed);
    v = wrapClampParam(v, v0, v1, b.vclosed);
    int uspan = findSpan(b.nu - 1, b.udeg, u, b.uknots);
    int vspan = findSpan(b.nv - 1, b.vdeg, v, b.vknots);
    double Nu[16], Nv[16];
    basisFuns(uspan, u, b.udeg, b.uknots, Nu);
    basisFuns(vspan, v, b.vdeg, b.vknots, Nv);
    Vec3d acc{0, 0, 0};
    double wsum = 0.0;
    for (int i = 0; i <= b.udeg; ++i) {
        int ci = uspan - b.udeg + i;
        for (int j = 0; j <= b.vdeg; ++j) {
            int cj = vspan - b.vdeg + j;
            size_t idx = static_cast<size_t>(ci) * static_cast<size_t>(b.nv) +
                         static_cast<size_t>(cj);
            double w = b.weights.empty() ? 1.0 : b.weights[idx];
            double bw = Nu[i] * Nv[j] * w;
            acc = acc + b.ctrl[idx] * bw;
            wsum += bw;
        }
    }
    return acc * (1.0 / std::max(wsum, 1e-300));
}

// ---------------------------------------------------------------------------
// Curve evaluation
// ---------------------------------------------------------------------------

void curveDomain(const Curve& c, double& t0, double& t1) {
    switch (c.kind) {
        case CurveKind::Circle:
        case CurveKind::Ellipse:
            t0 = 0;
            t1 = kTwoPi;
            break;
        case CurveKind::Polyline:
            t0 = 0;
            t1 = static_cast<double>(c.poly.size() - 1);
            break;
        case CurveKind::BSpline:
            t0 = c.bs.knots[static_cast<size_t>(c.bs.degree)];
            t1 = c.bs.knots[c.bs.knots.size() - 1 - static_cast<size_t>(c.bs.degree)];
            break;
        default:
            t0 = 0;
            t1 = 1;
            break;
    }
}

Vec3d evalCurve(const Curve& c, double t) {
    switch (c.kind) {
        case CurveKind::Line:
            return c.p0 + c.dir * t;
        case CurveKind::Circle:
            return c.frame.o + c.frame.ax * (c.r1 * std::cos(t)) +
                   c.frame.ay * (c.r1 * std::sin(t));
        case CurveKind::Ellipse:
            return c.frame.o + c.frame.ax * (c.r1 * std::cos(t)) +
                   c.frame.ay * (c.r2 * std::sin(t));
        case CurveKind::Polyline: {
            double tmax = static_cast<double>(c.poly.size() - 1);
            t = clampd(t, 0.0, tmax);
            size_t i = static_cast<size_t>(std::floor(t));
            if (i >= c.poly.size() - 1) i = c.poly.size() - 2;
            double f = t - static_cast<double>(i);
            return c.poly[i] * (1.0 - f) + c.poly[i + 1] * f;
        }
        case CurveKind::BSpline:
            return evalBSCurve(c.bs, t);
        default:
            return c.p0;
    }
}

// Parametric angle of a point on a circle/ellipse.
double angleOnConic(const Curve& c, const Vec3d& p) {
    Vec3d d = p - c.frame.o;
    double x = step::dot(d, c.frame.ax);
    double y = step::dot(d, c.frame.ay);
    if (c.kind == CurveKind::Ellipse) {
        x /= (c.r1 > 1e-300 ? c.r1 : 1.0);
        y /= (c.r2 > 1e-300 ? c.r2 : 1.0);
    }
    if (std::hypot(x, y) < 1e-12) return 0.0;
    return std::atan2(y, x);
}

// ---------------------------------------------------------------------------
// Surface context: precomputed data for evaluation and UV projection
// ---------------------------------------------------------------------------

struct SurfCtx {
    const Surface* s = nullptr;
    double uPeriod = 0, vPeriod = 0;   // 0 -> not periodic
    double uStep = kBig, vStep = kBig; // target parametric step for smoothness
    double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
    bool uFinite = false, vFinite = false;
    // sampled profile curve (extrusion / revolution)
    std::vector<double> cs;
    std::vector<Vec3d> cpts;
    std::vector<double> cang, crad, chgt; // revolution: per-sample angle/radius/height
    double DD = 1.0;                       // |sweepDir|^2
    // b-spline projection grid
    int gnu = 0, gnv = 0;
    std::vector<Vec3d> gpts;
};

Vec3d evalPos(const SurfCtx& c, double u, double v) {
    const Surface& s = *c.s;
    const Frame& F = s.frame;
    switch (s.kind) {
        case SurfKind::Plane:
            return F.o + F.ax * u + F.ay * v;
        case SurfKind::Cylinder:
            return F.o + (F.ax * std::cos(u) + F.ay * std::sin(u)) * s.r1 + F.az * v;
        case SurfKind::Cone: {
            double rad = s.r1 + v * std::tan(s.angle);
            return F.o + (F.ax * std::cos(u) + F.ay * std::sin(u)) * rad + F.az * v;
        }
        case SurfKind::Sphere:
            return F.o + (F.ax * std::cos(u) + F.ay * std::sin(u)) * (s.r1 * std::cos(v)) +
                   F.az * (s.r1 * std::sin(v));
        case SurfKind::Torus:
            return F.o + (F.ax * std::cos(u) + F.ay * std::sin(u)) * (s.r1 + s.r2 * std::cos(v)) +
                   F.az * (s.r2 * std::sin(v));
        case SurfKind::Extrusion:
            return evalCurve(s.base, u) + s.sweepDir * v;
        case SurfKind::Revolution: {
            Vec3d q = evalCurve(s.base, v) - F.o;
            const Vec3d& d = F.az;
            double cu = std::cos(u), su = std::sin(u);
            Vec3d qr = q * cu + step::cross(d, q) * su + d * (step::dot(d, q) * (1.0 - cu));
            return F.o + qr;
        }
        case SurfKind::BSpline:
            return evalBSSurf(s.bs, u, v);
        default:
            return {0, 0, 0};
    }
}

// Surface normal from Su x Sv (before applying the face same_sense flag).
Vec3d evalNormalRaw(const SurfCtx& c, double u, double v) {
    const Surface& s = *c.s;
    const Frame& F = s.frame;
    switch (s.kind) {
        case SurfKind::Plane:
            return F.az;
        case SurfKind::Cylinder:
            return F.ax * std::cos(u) + F.ay * std::sin(u);
        case SurfKind::Cone: {
            double rad = s.r1 + v * std::tan(s.angle);
            if (std::fabs(rad) < 1e-9) rad = 1e-9;
            double cu = std::cos(u), su = std::sin(u);
            Vec3d Su = (F.ax * (-su) + F.ay * cu) * rad;
            Vec3d Sv = (F.ax * cu + F.ay * su) * std::tan(s.angle) + F.az;
            return step::normalized(step::cross(Su, Sv));
        }
        case SurfKind::Sphere: {
            double cu = std::cos(u), su = std::sin(u);
            double cv = std::cos(v), sv = std::sin(v);
            return F.ax * (cu * cv) + F.ay * (su * cv) + F.az * sv;
        }
        case SurfKind::Torus: {
            double cu = std::cos(u), su = std::sin(u);
            double cv = std::cos(v), sv = std::sin(v);
            return F.ax * (cu * cv) + F.ay * (su * cv) + F.az * sv;
        }
        default: {
            // numeric partials
            double hu = (c.uFinite ? (c.u1 - c.u0) : 1.0) * 1e-5 + 1e-9;
            double hv = (c.vFinite ? (c.v1 - c.v0) : 1.0) * 1e-5 + 1e-9;
            Vec3d Su = evalPos(c, u + hu, v) - evalPos(c, u - hu, v);
            Vec3d Sv = evalPos(c, u, v + hv) - evalPos(c, u, v - hv);
            return step::normalized(step::cross(Su, Sv));
        }
    }
}

// Sample the profile curve of a swept surface into the context.
void sampleProfile(SurfCtx& c) {
    const Curve& b = c.s->base;
    double t0, t1;
    curveDomain(b, t0, t1);
    int n = 128;
    c.cs.resize(static_cast<size_t>(n + 1));
    c.cpts.resize(static_cast<size_t>(n + 1));
    for (int i = 0; i <= n; ++i) {
        double t = t0 + (t1 - t0) * i / n;
        c.cs[static_cast<size_t>(i)] = t;
        c.cpts[static_cast<size_t>(i)] = evalCurve(b, t);
    }
}

bool initCtx(const Surface& s, SurfCtx& c) {
    c.s = &s;
    switch (s.kind) {
        case SurfKind::Plane:
            return true;
        case SurfKind::Cylinder:
        case SurfKind::Cone:
            c.uPeriod = kTwoPi;
            c.uStep = kAngStep;
            c.u0 = 0;
            c.u1 = kTwoPi;
            c.uFinite = true;
            return true;
        case SurfKind::Sphere:
            c.uPeriod = kTwoPi;
            c.uStep = kAngStep;
            c.vStep = kAngStep;
            c.u0 = 0;
            c.u1 = kTwoPi;
            c.v0 = -kPi / 2;
            c.v1 = kPi / 2;
            c.uFinite = c.vFinite = true;
            return true;
        case SurfKind::Torus:
            c.uPeriod = c.vPeriod = kTwoPi;
            c.uStep = c.vStep = kAngStep;
            c.u0 = c.v0 = 0;
            c.u1 = c.v1 = kTwoPi;
            c.uFinite = c.vFinite = true;
            return true;
        case SurfKind::Extrusion: {
            if (s.base.kind == CurveKind::None) return false;
            double t0, t1;
            curveDomain(s.base, t0, t1);
            c.u0 = t0;
            c.u1 = t1;
            c.DD = std::max(step::dot(s.sweepDir, s.sweepDir), 1e-300);
            if (s.base.kind == CurveKind::Circle || s.base.kind == CurveKind::Ellipse) {
                c.uPeriod = kTwoPi;
                c.uStep = kAngStep;
                c.uFinite = true;
            } else if (s.base.kind == CurveKind::BSpline || s.base.kind == CurveKind::Polyline) {
                if (s.base.kind == CurveKind::BSpline && s.base.bs.closed)
                    c.uPeriod = t1 - t0;
                c.uStep = (t1 - t0) / 32.0;
                c.uFinite = true;
            }
            sampleProfile(c);
            return true;
        }
        case SurfKind::Revolution: {
            if (s.base.kind == CurveKind::None) return false;
            c.uPeriod = kTwoPi;
            c.uStep = kAngStep;
            c.u0 = 0;
            c.u1 = kTwoPi;
            c.uFinite = true;
            double t0, t1;
            curveDomain(s.base, t0, t1);
            c.v0 = t0;
            c.v1 = t1;
            if (s.base.kind == CurveKind::Circle || s.base.kind == CurveKind::Ellipse) {
                c.vStep = kAngStep;
                c.vFinite = true;
            } else if (s.base.kind == CurveKind::BSpline || s.base.kind == CurveKind::Polyline) {
                c.vStep = (t1 - t0) / 32.0;
                c.vFinite = true;
            }
            sampleProfile(c);
            // per-sample cylindrical coordinates around the axis
            size_t n = c.cpts.size();
            c.cang.resize(n);
            c.crad.resize(n);
            c.chgt.resize(n);
            for (size_t i = 0; i < n; ++i) {
                Vec3d d = c.cpts[i] - s.frame.o;
                double h = step::dot(d, s.frame.az);
                double x = step::dot(d, s.frame.ax);
                double y = step::dot(d, s.frame.ay);
                c.chgt[i] = h;
                c.crad[i] = std::hypot(x, y);
                c.cang[i] = (c.crad[i] > 1e-12) ? std::atan2(y, x) : 0.0;
            }
            return true;
        }
        case SurfKind::BSpline: {
            const BSSurf& b = s.bs;
            c.u0 = b.uknots[static_cast<size_t>(b.udeg)];
            c.u1 = b.uknots[b.uknots.size() - 1 - static_cast<size_t>(b.udeg)];
            c.v0 = b.vknots[static_cast<size_t>(b.vdeg)];
            c.v1 = b.vknots[b.vknots.size() - 1 - static_cast<size_t>(b.vdeg)];
            if (c.u1 - c.u0 < 1e-12 || c.v1 - c.v0 < 1e-12) return false;
            c.uFinite = c.vFinite = true;
            if (b.uclosed) c.uPeriod = c.u1 - c.u0;
            if (b.vclosed) c.vPeriod = c.v1 - c.v0;
            c.uStep = (c.u1 - c.u0) / 24.0;
            c.vStep = (c.v1 - c.v0) / 24.0;
            // projection grid
            c.gnu = std::min(33, std::max(9, 2 * (b.nu - b.udeg) + 1));
            c.gnv = std::min(33, std::max(9, 2 * (b.nv - b.vdeg) + 1));
            c.gpts.resize(static_cast<size_t>(c.gnu) * static_cast<size_t>(c.gnv));
            for (int i = 0; i < c.gnu; ++i) {
                double uu = c.u0 + (c.u1 - c.u0) * i / (c.gnu - 1);
                for (int j = 0; j < c.gnv; ++j) {
                    double vv = c.v0 + (c.v1 - c.v0) * j / (c.gnv - 1);
                    c.gpts[static_cast<size_t>(i) * static_cast<size_t>(c.gnv) +
                           static_cast<size_t>(j)] = evalBSSurf(b, uu, vv);
                }
            }
            return true;
        }
        default:
            return false;
    }
}

// ---------------------------------------------------------------------------
// Point -> (u,v) projection (canonical branch; unwrapping happens later)
// ---------------------------------------------------------------------------

// Ternary refinement of a 1-parameter distance-like cost.
template <typename Cost>
double refine1D(double lo, double hi, Cost cost, int iters) {
    for (int it = 0; it < iters; ++it) {
        double m1 = lo + (hi - lo) / 3.0;
        double m2 = hi - (hi - lo) / 3.0;
        if (cost(m1) <= cost(m2)) hi = m2;
        else lo = m1;
    }
    return 0.5 * (lo + hi);
}

V2 projectUV(const SurfCtx& c, const Vec3d& p, const V2* prev) {
    const Surface& s = *c.s;
    const Frame& F = s.frame;
    V2 uv;
    switch (s.kind) {
        case SurfKind::Plane: {
            Vec3d d = p - F.o;
            uv.u = step::dot(d, F.ax);
            uv.v = step::dot(d, F.ay);
            return uv;
        }
        case SurfKind::Cylinder:
        case SurfKind::Cone: {
            Vec3d d = p - F.o;
            double x = step::dot(d, F.ax), y = step::dot(d, F.ay);
            uv.u = (std::hypot(x, y) > 1e-12) ? std::atan2(y, x)
                                              : (prev ? prev->u : 0.0);
            uv.v = step::dot(d, F.az);
            return uv;
        }
        case SurfKind::Sphere: {
            Vec3d d = p - F.o;
            double x = step::dot(d, F.ax), y = step::dot(d, F.ay);
            double z = step::dot(d, F.az);
            double rl = std::hypot(x, y);
            uv.u = (rl > 1e-9 * (s.r1 + 1.0)) ? std::atan2(y, x) : (prev ? prev->u : 0.0);
            uv.v = std::asin(clampd(z / std::max(s.r1, 1e-300), -1.0, 1.0));
            return uv;
        }
        case SurfKind::Torus: {
            Vec3d d = p - F.o;
            double x = step::dot(d, F.ax), y = step::dot(d, F.ay);
            double z = step::dot(d, F.az);
            double rl = std::hypot(x, y);
            uv.u = (rl > 1e-12) ? std::atan2(y, x) : (prev ? prev->u : 0.0);
            uv.v = std::atan2(z, rl - s.r1);
            return uv;
        }
        case SurfKind::Extrusion: {
            const Vec3d& D = s.sweepDir;
            size_t best = 0;
            double bestCost = std::numeric_limits<double>::max();
            for (size_t i = 0; i < c.cpts.size(); ++i) {
                Vec3d d = p - c.cpts[i];
                double vi = step::dot(d, D) / c.DD;
                Vec3d r = d - D * vi;
                double cost = step::dot(r, r);
                if (cost < bestCost) { bestCost = cost; best = i; }
            }
            double lo = c.cs[best > 0 ? best - 1 : 0];
            double hi = c.cs[std::min(best + 1, c.cs.size() - 1)];
            auto cost = [&](double t) {
                Vec3d d = p - evalCurve(s.base, t);
                double vi = step::dot(d, D) / c.DD;
                Vec3d r = d - D * vi;
                return step::dot(r, r);
            };
            uv.u = refine1D(lo, hi, cost, 24);
            uv.v = step::dot(p - evalCurve(s.base, uv.u), D) / c.DD;
            return uv;
        }
        case SurfKind::Revolution: {
            Vec3d d = p - F.o;
            double h = step::dot(d, F.az);
            double x = step::dot(d, F.ax), y = step::dot(d, F.ay);
            double rp = std::hypot(x, y);
            size_t best = 0;
            double bestCost = std::numeric_limits<double>::max();
            for (size_t i = 0; i < c.cpts.size(); ++i) {
                double dr = c.crad[i] - rp, dh = c.chgt[i] - h;
                double cost = dr * dr + dh * dh;
                if (cost < bestCost) { bestCost = cost; best = i; }
            }
            double lo = c.cs[best > 0 ? best - 1 : 0];
            double hi = c.cs[std::min(best + 1, c.cs.size() - 1)];
            auto cost = [&](double t) {
                Vec3d q = evalCurve(s.base, t) - F.o;
                double qh = step::dot(q, F.az);
                double qr = std::hypot(step::dot(q, F.ax), step::dot(q, F.ay));
                double dr = qr - rp, dh = qh - h;
                return dr * dr + dh * dh;
            };
            uv.v = refine1D(lo, hi, cost, 24);
            Vec3d q = evalCurve(s.base, uv.v) - F.o;
            double qx = step::dot(q, F.ax), qy = step::dot(q, F.ay);
            double pang = (rp > 1e-12) ? std::atan2(y, x) : (prev ? prev->u : 0.0);
            double cang2 = (std::hypot(qx, qy) > 1e-12) ? std::atan2(qy, qx) : 0.0;
            uv.u = pang - cang2;
            uv.u = std::atan2(std::sin(uv.u), std::cos(uv.u)); // canonical branch
            return uv;
        }
        case SurfKind::BSpline: {
            const BSSurf& b = s.bs;
            double bu = c.u0, bv = c.v0;
            double bestCost = std::numeric_limits<double>::max();
            if (prev) {
                double pu = wrapClampParam(prev->u, c.u0, c.u1, b.uclosed);
                double pv = wrapClampParam(prev->v, c.v0, c.v1, b.vclosed);
                Vec3d d = p - evalBSSurf(b, pu, pv);
                bestCost = step::dot(d, d);
                bu = pu;
                bv = pv;
            }
            for (int i = 0; i < c.gnu; ++i) {
                double uu = c.u0 + (c.u1 - c.u0) * i / (c.gnu - 1);
                for (int j = 0; j < c.gnv; ++j) {
                    double vv = c.v0 + (c.v1 - c.v0) * j / (c.gnv - 1);
                    Vec3d d = p - c.gpts[static_cast<size_t>(i) * static_cast<size_t>(c.gnv) +
                                         static_cast<size_t>(j)];
                    double cost = step::dot(d, d);
                    if (cost < bestCost) { bestCost = cost; bu = uu; bv = vv; }
                }
            }
            // derivative-free local descent with shrinking step
            double du = (c.u1 - c.u0) / (c.gnu - 1);
            double dv = (c.v1 - c.v0) / (c.gnv - 1);
            for (int it = 0; it < 28; ++it) {
                bool moved = false;
                for (int di = -1; di <= 1 && !moved; ++di) {
                    for (int dj = -1; dj <= 1; ++dj) {
                        if (di == 0 && dj == 0) continue;
                        double tu = bu + di * du, tv = bv + dj * dv;
                        if (b.uclosed) tu = wrapClampParam(tu, c.u0, c.u1, true);
                        else tu = clampd(tu, c.u0, c.u1);
                        if (b.vclosed) tv = wrapClampParam(tv, c.v0, c.v1, true);
                        else tv = clampd(tv, c.v0, c.v1);
                        Vec3d d = p - evalBSSurf(b, tu, tv);
                        double cost = step::dot(d, d);
                        if (cost < bestCost) {
                            bestCost = cost;
                            bu = tu;
                            bv = tv;
                            moved = true;
                            break;
                        }
                    }
                }
                if (!moved) {
                    du *= 0.5;
                    dv *= 0.5;
                    if (du < (c.u1 - c.u0) * 1e-6 && dv < (c.v1 - c.v0) * 1e-6) break;
                }
            }
            uv.u = bu;
            uv.v = bv;
            return uv;
        }
        default:
            return uv;
    }
}

// ---------------------------------------------------------------------------
// Edge and loop sampling (3D)
// ---------------------------------------------------------------------------

// Points along the edge from pstart to pend, following curve orientation
// (curveSense already applied). The ORIENTED_EDGE flag is applied by the
// caller.
std::vector<Vec3d> sampleEdge(const step::Edge& e) {
    std::vector<Vec3d> pts;
    const Curve& c = e.curve;
    switch (c.kind) {
        case CurveKind::None:
        case CurveKind::Line:
            pts.push_back(e.pstart);
            pts.push_back(e.pend);
            return pts;
        case CurveKind::Circle:
        case CurveKind::Ellipse: {
            double ts = angleOnConic(c, e.pstart);
            double te = angleOnConic(c, e.pend);
            double rr = c.r1 + c.r2;
            bool full = step::dist(e.pstart, e.pend) < 1e-6 * (1.0 + rr);
            double sweep;
            if (full) {
                sweep = kTwoPi;
            } else {
                double d = e.curveSense ? (te - ts) : (ts - te);
                d = std::fmod(d, kTwoPi);
                if (d <= 1e-12) d += kTwoPi;
                sweep = d;
            }
            double sgn = e.curveSense ? 1.0 : -1.0;
            int nseg = std::max(1, static_cast<int>(std::ceil(sweep / kAngStep)));
            for (int i = 0; i <= nseg; ++i) {
                double t = ts + sgn * sweep * i / nseg;
                pts.push_back(evalCurve(c, t));
            }
            pts.front() = e.pstart;             // snap to exact vertex points
            if (!full) pts.back() = e.pend;
            return pts;
        }
        case CurveKind::Polyline: {
            pts = c.poly;
            if (!e.curveSense) std::reverse(pts.begin(), pts.end());
            if (step::dist(pts.front(), e.pstart) > step::dist(pts.back(), e.pstart))
                std::reverse(pts.begin(), pts.end());
            return pts;
        }
        case CurveKind::BSpline: {
            double t0, t1;
            curveDomain(c, t0, t1);
            int spans = std::max(1, static_cast<int>(c.bs.ctrl.size()) - c.bs.degree);
            int nseg = std::min(256, std::max(16, spans * 8));
            for (int i = 0; i <= nseg; ++i)
                pts.push_back(evalCurve(c, t0 + (t1 - t0) * i / nseg));
            if (!e.curveSense) std::reverse(pts.begin(), pts.end());
            if (step::dist(pts.front(), e.pstart) > step::dist(pts.back(), e.pstart))
                std::reverse(pts.begin(), pts.end());
            return pts;
        }
        default:
            pts.push_back(e.pstart);
            pts.push_back(e.pend);
            return pts;
    }
}

// Chain the edges of a loop into a closed 3D polyline (no repeated last pt).
bool sampleLoop(const step::Loop& lp, std::vector<Vec3d>& out) {
    for (const step::Edge& e : lp.edges) {
        std::vector<Vec3d> pts = sampleEdge(e);
        if (pts.size() < 2) continue;
        if (!e.orientation) std::reverse(pts.begin(), pts.end());
        size_t start = (!out.empty() && nearly3(out.back(), pts.front())) ? 1 : 0;
        out.insert(out.end(), pts.begin() + static_cast<std::ptrdiff_t>(start), pts.end());
    }
    if (out.size() >= 2 && nearly3(out.front(), out.back())) out.pop_back();
    return out.size() >= 3;
}

// ---------------------------------------------------------------------------
// 2D polygon utilities (coordinates normalized to roughly [0,1])
// ---------------------------------------------------------------------------

double cross2(const V2& a, const V2& b, const V2& c) {
    return (b.u - a.u) * (c.v - a.v) - (b.v - a.v) * (c.u - a.u);
}

double signedArea(const std::vector<V2>& p) {
    double a = 0;
    size_t n = p.size();
    for (size_t i = 0; i < n; ++i) {
        const V2& q = p[i];
        const V2& r = p[(i + 1) % n];
        a += q.u * r.v - r.u * q.v;
    }
    return 0.5 * a;
}

bool sameV2(const V2& a, const V2& b) {
    return std::fabs(a.u - b.u) < 1e-12 && std::fabs(a.v - b.v) < 1e-12;
}

// Strict (interior) segment crossing test.
bool segsCross(const V2& a, const V2& b, const V2& c, const V2& d) {
    const double eps = 1e-14;
    double d1 = cross2(c, d, a), d2 = cross2(c, d, b);
    double d3 = cross2(a, b, c), d4 = cross2(a, b, d);
    return ((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) &&
           ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps));
}

bool pointInTriCCW(const V2& a, const V2& b, const V2& c, const V2& p) {
    const double eps = 1e-14;
    return cross2(a, b, p) >= -eps && cross2(b, c, p) >= -eps && cross2(c, a, p) >= -eps;
}

// Bridge holes (CW) into the outer polygon (CCW) via keyhole slits.
std::vector<V2> mergeHoles(std::vector<V2> outer, std::vector<std::vector<V2>> holes) {
    // process rightmost holes first
    std::sort(holes.begin(), holes.end(), [](const std::vector<V2>& a, const std::vector<V2>& b) {
        auto maxu = [](const std::vector<V2>& p) {
            double m = -kBig;
            for (const V2& q : p) m = std::max(m, q.u);
            return m;
        };
        return maxu(a) > maxu(b);
    });
    for (size_t hi = 0; hi < holes.size(); ++hi) {
        const std::vector<V2>& h = holes[hi];
        if (h.size() < 3) continue;
        size_t mi = 0;
        for (size_t i = 1; i < h.size(); ++i)
            if (h[i].u > h[mi].u) mi = i;
        const V2& M = h[mi];
        // candidate outer vertices sorted by distance
        std::vector<size_t> cand(outer.size());
        for (size_t i = 0; i < outer.size(); ++i) cand[i] = i;
        std::sort(cand.begin(), cand.end(), [&](size_t a, size_t b) {
            double da = (outer[a].u - M.u) * (outer[a].u - M.u) +
                        (outer[a].v - M.v) * (outer[a].v - M.v);
            double db = (outer[b].u - M.u) * (outer[b].u - M.u) +
                        (outer[b].v - M.v) * (outer[b].v - M.v);
            return da < db;
        });
        auto visible = [&](size_t vi) {
            const V2& V = outer[vi];
            size_t n = outer.size();
            for (size_t i = 0; i < n; ++i) {
                size_t j = (i + 1) % n;
                if (i == vi || j == vi) continue;
                if (segsCross(M, V, outer[i], outer[j])) return false;
            }
            for (size_t k = hi; k < holes.size(); ++k) {
                const std::vector<V2>& hk = holes[k];
                size_t n2 = hk.size();
                for (size_t i = 0; i < n2; ++i) {
                    size_t j = (i + 1) % n2;
                    if (k == hi && (i == mi || j == mi)) continue;
                    if (segsCross(M, V, hk[i], hk[j])) return false;
                }
            }
            return true;
        };
        size_t chosen = cand.empty() ? 0 : cand[0];
        for (size_t idx : cand) {
            if (visible(idx)) { chosen = idx; break; }
        }
        // splice: outer[0..chosen], hole cycle starting at mi, back to
        // hole[mi] and outer[chosen], then rest of outer
        std::vector<V2> merged;
        merged.reserve(outer.size() + h.size() + 2);
        for (size_t i = 0; i <= chosen; ++i) merged.push_back(outer[i]);
        for (size_t i = 0; i <= h.size(); ++i) merged.push_back(h[(mi + i) % h.size()]);
        for (size_t i = chosen; i < outer.size(); ++i) merged.push_back(outer[i]);
        outer = std::move(merged);
    }
    return outer;
}

// Ear clipping of a CCW simple polygon (possibly with keyhole slits).
// Returns triangles as index triples into 'poly'. For small polygons the
// valid ear with the shortest diagonal (in the surface refinement metric
// uS/vS) is clipped first, which yields well-shaped strips on periodic
// surfaces and greatly reduces later refinement work.
std::vector<int> earClip(const std::vector<V2>& poly, double uS, double vS) {
    std::vector<int> tris;
    size_t n = poly.size();
    if (n < 3) return tris;
    std::vector<int> idx(n);
    for (size_t i = 0; i < n; ++i) idx[i] = static_cast<int>(i);
    const double epsA = 1e-14;
    const bool pickBest = (n <= 400); // O(n^3) worst case, keep it bounded
    size_t guard = 0, guardMax = 6 * n + 100;
    while (idx.size() > 2 && guard++ < guardMax) {
        size_t m = idx.size();
        bool clipped = false;
        size_t bestEar = m;
        double bestDiag = kBig;
        for (size_t i = 0; i < m; ++i) {
            int ia = idx[(i + m - 1) % m];
            int ib = idx[i];
            int ic = idx[(i + 1) % m];
            const V2& A = poly[static_cast<size_t>(ia)];
            const V2& B = poly[static_cast<size_t>(ib)];
            const V2& C = poly[static_cast<size_t>(ic)];
            double cr = cross2(A, B, C);
            if (cr <= epsA) continue; // reflex or degenerate corner
            bool blocked = false;
            for (size_t j = 0; j < m; ++j) {
                int ij = idx[j];
                if (ij == ia || ij == ib || ij == ic) continue;
                const V2& P = poly[static_cast<size_t>(ij)];
                if (sameV2(P, A) || sameV2(P, B) || sameV2(P, C)) continue;
                if (pointInTriCCW(A, B, C, P)) { blocked = true; break; }
            }
            if (blocked) continue;
            if (!pickBest) { bestEar = i; break; }
            double diag = std::hypot((A.u - C.u) / uS, (A.v - C.v) / vS);
            if (diag < bestDiag) { bestDiag = diag; bestEar = i; }
        }
        if (bestEar < m) {
            size_t i = bestEar;
            tris.push_back(idx[(i + m - 1) % m]);
            tris.push_back(idx[i]);
            tris.push_back(idx[(i + 1) % m]);
            idx.erase(idx.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
        }
        if (!clipped) {
            // Degenerate remainder: drop the least-reflex vertex to make
            // progress; emit only if it still has positive area.
            size_t besti = 0;
            double bestCr = -kBig;
            for (size_t i = 0; i < m; ++i) {
                int ia = idx[(i + m - 1) % m];
                int ib = idx[i];
                int ic = idx[(i + 1) % m];
                double cr = cross2(poly[static_cast<size_t>(ia)], poly[static_cast<size_t>(ib)],
                                   poly[static_cast<size_t>(ic)]);
                if (cr > bestCr) { bestCr = cr; besti = i; }
            }
            if (bestCr > epsA) {
                int ia = idx[(besti + m - 1) % m];
                int ib = idx[besti];
                int ic = idx[(besti + 1) % m];
                tris.push_back(ia);
                tris.push_back(ib);
                tris.push_back(ic);
            }
            idx.erase(idx.begin() + static_cast<std::ptrdiff_t>(besti));
        }
    }
    return tris;
}

// ---------------------------------------------------------------------------
// Conforming triangle refinement (edge-split with shared midpoint cache)
// ---------------------------------------------------------------------------

void refineTriangles(std::vector<V2>& verts, std::vector<int>& tris,
                     double uStepN, double vStepN) {
    if (uStepN >= kBig / 2 && vStepN >= kBig / 2) return;
    const double limit = 1.25;
    auto metric = [&](const V2& a, const V2& b) {
        return std::hypot((a.u - b.u) / uStepN, (a.v - b.v) / vStepN);
    };
    std::map<std::pair<int, int>, int> midCache;
    auto ekey = [](int a, int b) {
        return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
    };
    for (int round = 0; round < 12; ++round) {
        std::set<std::pair<int, int>> longEdges;
        for (size_t t = 0; t + 2 < tris.size(); t += 3) {
            int v[3] = {tris[t], tris[t + 1], tris[t + 2]};
            for (int k = 0; k < 3; ++k) {
                int a = v[k], b = v[(k + 1) % 3];
                if (metric(verts[static_cast<size_t>(a)], verts[static_cast<size_t>(b)]) > limit)
                    longEdges.insert(ekey(a, b));
            }
        }
        if (longEdges.empty()) break;
        if (verts.size() > 30000 || tris.size() > 180000) break;
        auto midpoint = [&](int a, int b) {
            auto key = ekey(a, b);
            auto it = midCache.find(key);
            if (it != midCache.end()) return it->second;
            V2 mv{0.5 * (verts[static_cast<size_t>(a)].u + verts[static_cast<size_t>(b)].u),
                  0.5 * (verts[static_cast<size_t>(a)].v + verts[static_cast<size_t>(b)].v)};
            int id = static_cast<int>(verts.size());
            verts.push_back(mv);
            midCache[key] = id;
            return id;
        };
        std::vector<int> next;
        next.reserve(tris.size() * 2);
        auto pushTri = [&](int a, int b, int c) {
            next.push_back(a);
            next.push_back(b);
            next.push_back(c);
        };
        for (size_t t = 0; t + 2 < tris.size(); t += 3) {
            int a = tris[t], b = tris[t + 1], c = tris[t + 2];
            bool sp[3] = {longEdges.count(ekey(a, b)) > 0, longEdges.count(ekey(b, c)) > 0,
                          longEdges.count(ekey(c, a)) > 0};
            int cnt = (sp[0] ? 1 : 0) + (sp[1] ? 1 : 0) + (sp[2] ? 1 : 0);
            // rotate so that patterns are canonical:
            //  cnt==1 -> split edge is (a,b);  cnt==2 -> intact edge is (c,a)
            for (int r = 0; r < 2; ++r) {
                if ((cnt == 1 && sp[0]) || (cnt == 2 && !sp[2]) || cnt == 0 || cnt == 3) break;
                int ta = a;
                a = b;
                b = c;
                c = ta;
                bool t0 = sp[0];
                sp[0] = sp[1];
                sp[1] = sp[2];
                sp[2] = t0;
            }
            if (cnt == 0) {
                pushTri(a, b, c);
            } else if (cnt == 1) {
                int m = midpoint(a, b);
                pushTri(a, m, c);
                pushTri(m, b, c);
            } else if (cnt == 2) {
                int m1 = midpoint(a, b);
                int m2 = midpoint(b, c);
                pushTri(m1, b, m2);
                // split quad (a, m1, m2, c) along its shorter diagonal
                double d1 = metric(verts[static_cast<size_t>(a)], verts[static_cast<size_t>(m2)]);
                double d2 = metric(verts[static_cast<size_t>(m1)], verts[static_cast<size_t>(c)]);
                if (d1 <= d2) {
                    pushTri(a, m1, m2);
                    pushTri(a, m2, c);
                } else {
                    pushTri(a, m1, c);
                    pushTri(m1, m2, c);
                }
            } else {
                int m1 = midpoint(a, b);
                int m2 = midpoint(b, c);
                int m3 = midpoint(c, a);
                pushTri(a, m1, m3);
                pushTri(m1, b, m2);
                pushTri(m3, m2, c);
                pushTri(m1, m2, m3);
            }
        }
        tris = std::move(next);
    }
}

// ---------------------------------------------------------------------------
// Emission
// ---------------------------------------------------------------------------

void emitTriangles(const SurfCtx& ctx, bool sameSense, const std::vector<V2>& uvs,
                   const std::vector<int>& tris, Mesh& out) {
    std::vector<Vec3d> pos(uvs.size());
    std::vector<Vec3d> nor(uvs.size());
    for (size_t i = 0; i < uvs.size(); ++i) {
        pos[i] = evalPos(ctx, uvs[i].u, uvs[i].v);
        Vec3d n = evalNormalRaw(ctx, uvs[i].u, uvs[i].v);
        nor[i] = sameSense ? n : -n;
    }
    for (size_t t = 0; t + 2 < tris.size(); t += 3) {
        size_t a = static_cast<size_t>(tris[t]);
        size_t b = static_cast<size_t>(tris[t + 1]);
        size_t c = static_cast<size_t>(tris[t + 2]);
        if (!sameSense) std::swap(b, c); // keep winding consistent with normal
        Vec3d ab = pos[b] - pos[a], ac = pos[c] - pos[a];
        Vec3d cr = step::cross(ab, ac);
        if (step::dot(cr, cr) < 1e-24) continue; // degenerate in 3D
        const size_t ids[3] = {a, b, c};
        for (size_t k = 0; k < 3; ++k) {
            const Vec3d& P = pos[ids[k]];
            const Vec3d& N = nor[ids[k]];
            out.positions.push_back({static_cast<float>(P.x), static_cast<float>(P.y),
                                     static_cast<float>(P.z)});
            out.normals.push_back({static_cast<float>(N.x), static_cast<float>(N.y),
                                   static_cast<float>(N.z)});
        }
    }
}

// Untrimmed rectangular-domain tessellation (fallback path).
bool tessDomain(const SurfCtx& ctx, bool sameSense, double u0, double u1, double v0,
                double v1, Mesh& out) {
    if (!(u1 > u0) || !(v1 > v0)) return false;
    int nu = 1, nv = 1;
    if (ctx.uStep < kBig / 2)
        nu = std::min(96, std::max(1, static_cast<int>(std::ceil((u1 - u0) / ctx.uStep))));
    if (ctx.vStep < kBig / 2)
        nv = std::min(96, std::max(1, static_cast<int>(std::ceil((v1 - v0) / ctx.vStep))));
    std::vector<V2> uvs;
    uvs.reserve(static_cast<size_t>(nu + 1) * static_cast<size_t>(nv + 1));
    for (int i = 0; i <= nu; ++i) {
        double uu = u0 + (u1 - u0) * i / nu;
        for (int j = 0; j <= nv; ++j) {
            double vv = v0 + (v1 - v0) * j / nv;
            uvs.push_back({uu, vv});
        }
    }
    std::vector<int> tris;
    for (int i = 0; i < nu; ++i) {
        for (int j = 0; j < nv; ++j) {
            int a = i * (nv + 1) + j;
            int b = (i + 1) * (nv + 1) + j;
            // CCW in (u,v)
            tris.push_back(a);
            tris.push_back(b);
            tris.push_back(b + 1);
            tris.push_back(a);
            tris.push_back(b + 1);
            tris.push_back(a + 1);
        }
    }
    size_t before = out.positions.size();
    emitTriangles(ctx, sameSense, uvs, tris, out);
    return out.positions.size() > before;
}

// ---------------------------------------------------------------------------
// Per-face tessellation
// ---------------------------------------------------------------------------

struct UVLoop {
    std::vector<V2> uv;
    bool outer = false;
    int windU = 0, windV = 0; // net seam crossings
};

// Fallback: tessellate the UV bounding box of the sampled loops.
bool fallbackFromLoops(const SurfCtx& ctx, bool sameSense, const std::vector<UVLoop>& loops,
                       Mesh& out) {
    double umin = kBig, umax = -kBig, vmin = kBig, vmax = -kBig;
    for (const UVLoop& l : loops) {
        for (const V2& p : l.uv) {
            umin = std::min(umin, p.u);
            umax = std::max(umax, p.u);
            vmin = std::min(vmin, p.v);
            vmax = std::max(vmax, p.v);
        }
    }
    if (umin > umax || vmin > vmax) {
        if (ctx.uFinite && ctx.vFinite)
            return tessDomain(ctx, sameSense, ctx.u0, ctx.u1, ctx.v0, ctx.v1, out);
        return false;
    }
    if (ctx.uPeriod > 0 && umax - umin > ctx.uPeriod) umax = umin + ctx.uPeriod;
    if (ctx.vPeriod > 0 && vmax - vmin > ctx.vPeriod) vmax = vmin + ctx.vPeriod;
    if (umax - umin < 1e-12 || vmax - vmin < 1e-12) return false;
    return tessDomain(ctx, sameSense, umin, umax, vmin, vmax, out);
}

// Returns 0 = failed, 1 = trimmed tessellation, 2 = fallback approximation.
int tessFace(const step::Face& f, Mesh& out) {
    SurfCtx ctx;
    if (!initCtx(f.surf, ctx)) return 0;

    // --- 1. sample and project boundary loops -----------------------------
    std::vector<UVLoop> loops;
    for (const step::Loop& lp : f.loops) {
        std::vector<Vec3d> pts;
        if (!sampleLoop(lp, pts)) continue;
        UVLoop ul;
        ul.outer = lp.outer;
        V2 prev;
        bool hasPrev = false;
        for (const Vec3d& p : pts) {
            V2 uv = projectUV(ctx, p, hasPrev ? &prev : nullptr);
            if (hasPrev) {
                uv.u = unwrapNear(uv.u, prev.u, ctx.uPeriod);
                uv.v = unwrapNear(uv.v, prev.v, ctx.vPeriod);
            }
            ul.uv.push_back(uv);
            prev = uv;
            hasPrev = true;
        }
        if (ul.uv.size() >= 2) {
            if (ctx.uPeriod > 0)
                ul.windU = static_cast<int>(
                    std::lround((ul.uv.back().u - ul.uv.front().u) / ctx.uPeriod));
            if (ctx.vPeriod > 0)
                ul.windV = static_cast<int>(
                    std::lround((ul.uv.back().v - ul.uv.front().v) / ctx.vPeriod));
        }
        if (ul.uv.size() >= 3 || ul.windU != 0 || ul.windV != 0) loops.push_back(std::move(ul));
    }

    if (loops.empty()) {
        // closed surface without boundary (full sphere / torus / spline)
        if (ctx.uFinite && ctx.vFinite &&
            tessDomain(ctx, f.sameSense, ctx.u0, ctx.u1, ctx.v0, ctx.v1, out))
            return 2;
        return 0;
    }

    // --- 2. assemble outer polygon + holes --------------------------------
    std::vector<V2> outerPoly;
    std::vector<std::vector<V2>> holes;

    std::vector<size_t> crossing;
    for (size_t i = 0; i < loops.size(); ++i)
        if (loops[i].windU != 0 || loops[i].windV != 0) crossing.push_back(i);

    if (!crossing.empty()) {
        // Handle the classic seamless-cylinder layout: exactly two loops that
        // each wrap once around the u seam (e.g. top+bottom circles).
        bool okMerge = false;
        if (crossing.size() == 2 && ctx.uPeriod > 0) {
            UVLoop& A = loops[crossing[0]];
            UVLoop& B = loops[crossing[1]];
            if (std::abs(A.windU) == 1 && std::abs(B.windU) == 1 && A.windV == 0 &&
                B.windV == 0) {
                std::vector<V2> a = A.uv, b2 = B.uv;
                if (A.windU < 0) std::reverse(a.begin(), a.end());   // travel +period
                if (B.windU > 0) std::reverse(b2.begin(), b2.end()); // travel -period
                double shift =
                    ctx.uPeriod * std::floor((a.back().u - b2.front().u) / ctx.uPeriod + 0.5);
                for (V2& p : b2) p.u += shift;
                outerPoly = a;
                outerPoly.insert(outerPoly.end(), b2.begin(), b2.end());
                for (size_t i = 0; i < loops.size(); ++i)
                    if (i != crossing[0] && i != crossing[1] && loops[i].uv.size() >= 3)
                        holes.push_back(loops[i].uv);
                okMerge = true;
            }
        }
        if (!okMerge)
            return fallbackFromLoops(ctx, f.sameSense, loops, out) ? 2 : 0;
    } else {
        // choose outer loop: prefer flagged FACE_OUTER_BOUND, else largest area
        size_t outerIdx = 0;
        double bestArea = -1;
        bool haveFlagged = false;
        for (size_t i = 0; i < loops.size(); ++i)
            if (loops[i].outer) haveFlagged = true;
        for (size_t i = 0; i < loops.size(); ++i) {
            if (haveFlagged && !loops[i].outer) continue;
            double a = std::fabs(signedArea(loops[i].uv));
            if (a > bestArea) { bestArea = a; outerIdx = i; }
        }
        outerPoly = loops[outerIdx].uv;
        for (size_t i = 0; i < loops.size(); ++i)
            if (i != outerIdx && loops[i].uv.size() >= 3) holes.push_back(loops[i].uv);
    }

    if (outerPoly.size() < 3) return fallbackFromLoops(ctx, f.sameSense, loops, out) ? 2 : 0;

    // --- 3. periodic shift of holes toward the outer loop -----------------
    auto centroid = [](const std::vector<V2>& p) {
        V2 cen;
        for (const V2& q : p) { cen.u += q.u; cen.v += q.v; }
        double n = static_cast<double>(p.size());
        cen.u /= n;
        cen.v /= n;
        return cen;
    };
    V2 oc = centroid(outerPoly);
    for (std::vector<V2>& h : holes) {
        V2 hc = centroid(h);
        if (ctx.uPeriod > 0) {
            double sh = ctx.uPeriod * std::floor((oc.u - hc.u) / ctx.uPeriod + 0.5);
            for (V2& p : h) p.u += sh;
        }
        if (ctx.vPeriod > 0) {
            double sh = ctx.vPeriod * std::floor((oc.v - hc.v) / ctx.vPeriod + 0.5);
            for (V2& p : h) p.v += sh;
        }
    }

    // --- 4. orientation + normalization ----------------------------------
    if (signedArea(outerPoly) < 0) std::reverse(outerPoly.begin(), outerPoly.end());
    for (std::vector<V2>& h : holes)
        if (signedArea(h) > 0) std::reverse(h.begin(), h.end());

    double umin = kBig, umax = -kBig, vmin = kBig, vmax = -kBig;
    auto extend = [&](const std::vector<V2>& p) {
        for (const V2& q : p) {
            umin = std::min(umin, q.u);
            umax = std::max(umax, q.u);
            vmin = std::min(vmin, q.v);
            vmax = std::max(vmax, q.v);
        }
    };
    extend(outerPoly);
    for (const std::vector<V2>& h : holes) extend(h);
    double w = umax - umin, hgt = vmax - vmin;
    if (w < 1e-12 || hgt < 1e-12)
        return fallbackFromLoops(ctx, f.sameSense, loops, out) ? 2 : 0;
    if (std::fabs(signedArea(outerPoly)) < 1e-9 * w * hgt)
        return fallbackFromLoops(ctx, f.sameSense, loops, out) ? 2 : 0;

    auto normalizeLoop = [&](std::vector<V2>& p) {
        for (V2& q : p) {
            q.u = (q.u - umin) / w;
            q.v = (q.v - vmin) / hgt;
        }
        // drop consecutive duplicates
        std::vector<V2> clean;
        for (const V2& q : p)
            if (clean.empty() || !sameV2(clean.back(), q)) clean.push_back(q);
        while (clean.size() >= 2 && sameV2(clean.front(), clean.back())) clean.pop_back();
        p = std::move(clean);
    };
    normalizeLoop(outerPoly);
    for (std::vector<V2>& h : holes) normalizeLoop(h);
    holes.erase(std::remove_if(holes.begin(), holes.end(),
                               [](const std::vector<V2>& h) { return h.size() < 3; }),
                holes.end());
    if (outerPoly.size() < 3) return fallbackFromLoops(ctx, f.sameSense, loops, out) ? 2 : 0;

    // --- 5. triangulate ----------------------------------------------------
    double uStepN = (ctx.uStep < kBig / 2) ? ctx.uStep / w : kBig;
    double vStepN = (ctx.vStep < kBig / 2) ? ctx.vStep / hgt : kBig;
    std::vector<V2> poly = mergeHoles(outerPoly, holes);
    std::vector<int> tris = earClip(poly, uStepN, vStepN);
    if (tris.empty()) return fallbackFromLoops(ctx, f.sameSense, loops, out) ? 2 : 0;

    // canonicalize duplicated vertices (bridge slits) so refinement stays
    // conforming across keyhole cuts
    {
        std::map<std::pair<long long, long long>, int> canon;
        std::vector<int> remap(poly.size());
        std::vector<V2> verts;
        for (size_t i = 0; i < poly.size(); ++i) {
            long long qu = static_cast<long long>(std::llround(poly[i].u * 1e9));
            long long qv = static_cast<long long>(std::llround(poly[i].v * 1e9));
            auto key = std::make_pair(qu, qv);
            auto it = canon.find(key);
            if (it == canon.end()) {
                int id = static_cast<int>(verts.size());
                canon[key] = id;
                verts.push_back(poly[i]);
                remap[i] = id;
            } else {
                remap[i] = it->second;
            }
        }
        std::vector<int> tris2;
        for (size_t t = 0; t + 2 < tris.size(); t += 3) {
            int a = remap[static_cast<size_t>(tris[t])];
            int b = remap[static_cast<size_t>(tris[t + 1])];
            int c = remap[static_cast<size_t>(tris[t + 2])];
            if (a == b || b == c || c == a) continue;
            tris2.push_back(a);
            tris2.push_back(b);
            tris2.push_back(c);
        }
        poly = std::move(verts);
        tris = std::move(tris2);
    }
    if (tris.empty()) return fallbackFromLoops(ctx, f.sameSense, loops, out) ? 2 : 0;

    // --- 6. refine for curvature ------------------------------------------
    refineTriangles(poly, tris, uStepN, vStepN);

    // --- 7. evaluate + emit -------------------------------------------------
    std::vector<V2> real(poly.size());
    for (size_t i = 0; i < poly.size(); ++i) {
        real[i].u = umin + poly[i].u * w;
        real[i].v = vmin + poly[i].v * hgt;
    }
    size_t before = out.positions.size();
    emitTriangles(ctx, f.sameSense, real, tris, out);
    if (out.positions.size() == before)
        return fallbackFromLoops(ctx, f.sameSense, loops, out) ? 2 : 0;
    return 1;
}

// ---------------------------------------------------------------------------
// Instance replication (assembly support)
// ---------------------------------------------------------------------------

// Append one placed copy of a part mesh (tessellated in local coordinates).
// Points get the full affine transform; normals are mapped with the cofactor
// matrix of the linear part -- for a triangle, cross(M*b - M*a, M*c - M*a) =
// C * cross(b - a, c - a), so this stays exact for any rotation, uniform
// scale, or even mirroring, without touching the winding order. Without scale
// the cofactor matrix equals the rotation itself (no inverse-transpose
// needed); the result is renormalized either way.
void appendInstance(const Mesh& src, const step::Mat43& M, Mesh& out) {
    if (step::xfIsIdentity(M)) { // untransformed copy stays bit-identical
        out.positions.insert(out.positions.end(), src.positions.begin(), src.positions.end());
        out.normals.insert(out.normals.end(), src.normals.begin(), src.normals.end());
        return;
    }
    const Vec3d nX = step::cross(M.cy, M.cz);
    const Vec3d nY = step::cross(M.cz, M.cx);
    const Vec3d nZ = step::cross(M.cx, M.cy);
    out.positions.reserve(out.positions.size() + src.positions.size());
    out.normals.reserve(out.normals.size() + src.normals.size());
    for (const Vec3f& p : src.positions) {
        Vec3d q = step::xfPoint(M, Vec3d{p.x, p.y, p.z});
        out.positions.push_back({static_cast<float>(q.x), static_cast<float>(q.y),
                                 static_cast<float>(q.z)});
    }
    for (const Vec3f& n : src.normals) {
        Vec3d q = step::normalized(nX * n.x + nY * n.y + nZ * n.z);
        out.normals.push_back({static_cast<float>(q.x), static_cast<float>(q.y),
                               static_cast<float>(q.z)});
    }
}

// Triangle range [start, start + count) of one placed instance in the
// output mesh.
using TriRange = std::pair<size_t, size_t>;

// Convert the parser's component tree (step::Model::nodes) into Mesh::nodes,
// filling each leaf with the triangle range of its placed instance. Leaves
// whose geometry produced no triangles are dropped, as are group nodes left
// without descendants; index order is preserved, so every parent still
// precedes its children. A tree reduced to a single node carries no more
// information than "one object" and is cleared.
void buildMeshNodes(const step::Model& model,
                    const std::vector<std::vector<TriRange>>& instRange, Mesh& out) {
    out.nodes.clear();
    if (model.nodes.empty()) return;
    const size_t n = model.nodes.size();
    std::vector<TriRange> range(n, {0, 0});
    for (size_t i = 0; i < n; ++i) {
        const step::ModelNode& mn = model.nodes[i];
        if (mn.part < 0) continue;
        if (static_cast<size_t>(mn.part) < instRange.size() && mn.instance >= 0 &&
            static_cast<size_t>(mn.instance) < instRange[static_cast<size_t>(mn.part)].size())
            range[i] = instRange[static_cast<size_t>(mn.part)][static_cast<size_t>(mn.instance)];
    }
    // keep leaves with triangles and groups with at least one kept
    // descendant (children always come after their parent, so one reverse
    // sweep propagates the flag)
    std::vector<char> keep(n, 0);
    for (size_t i = n; i-- > 0;) {
        if (model.nodes[i].part >= 0) keep[i] = (range[i].second > 0) ? 1 : 0;
        if (keep[i] && model.nodes[i].parent >= 0)
            keep[static_cast<size_t>(model.nodes[i].parent)] = 1;
    }
    std::vector<int> remap(n, -1);
    for (size_t i = 0; i < n; ++i) {
        if (!keep[i]) continue;
        const step::ModelNode& mn = model.nodes[i];
        remap[i] = static_cast<int>(out.nodes.size());
        MeshNode nd;
        nd.name = mn.name;
        nd.parent = (mn.parent >= 0) ? remap[static_cast<size_t>(mn.parent)] : -1;
        if (mn.part >= 0) {
            nd.triStart = range[i].first;
            nd.triCount = range[i].second;
        }
        out.nodes.push_back(std::move(nd));
    }
    if (out.nodes.size() <= 1) out.nodes.clear();
}

} // namespace

// ---------------------------------------------------------------------------
// Module entry point
// ---------------------------------------------------------------------------

namespace step {

bool tessellate(const Model& model, Mesh& out, std::string& err, int* facesOk,
                int* facesApprox, int* facesFailed) {
    int ok = 0, approx = 0, failed = 0;
    size_t totalFaces = 0;
    // triangle range of every placed instance: instRange[part][instance]
    std::vector<std::vector<TriRange>> instRange(model.parts.size());
    for (size_t pi = 0; pi < model.parts.size(); ++pi) {
        const Part& part = model.parts[pi];
        totalFaces += part.faces.size();
        // Tessellate the part ONCE in its local coordinates...
        Mesh pm;
        for (const Face& f : part.faces) {
            size_t beforeP = pm.positions.size();
            size_t beforeN = pm.normals.size();
            int r = 0;
            try {
                r = tessFace(f, pm);
            } catch (...) {
                r = 0;
            }
            if (r == 0) {
                pm.positions.resize(beforeP);
                pm.normals.resize(beforeN);
                ++failed;
            } else if (r == 1) {
                ++ok;
            } else {
                ++approx;
            }
        }
        if (pm.positions.empty()) {
            instRange[pi].assign(part.instances.size(), {0, 0});
            continue;
        }
        // ...then replicate it for every placed instance. Each appendInstance
        // call emits one contiguous block of triangles, recorded here so the
        // component tree can address it.
        instRange[pi].reserve(part.instances.size());
        for (const Mat43& xf : part.instances) {
            size_t start = out.positions.size() / 3;
            appendInstance(pm, xf, out);
            instRange[pi].push_back({start, out.positions.size() / 3 - start});
        }
    }
    buildMeshNodes(model, instRange, out);
    if (facesOk) *facesOk = ok;
    if (facesApprox) *facesApprox = approx;
    if (facesFailed) *facesFailed = failed;
    if (out.positions.empty()) {
        err = "STEP: no face could be tessellated (" + std::to_string(totalFaces) +
              " faces, " + std::to_string(model.skippedFaces) + " skipped by the parser)";
        return false;
    }
    out.computeBounds();
    return true;
}

} // namespace step
