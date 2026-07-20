/*
 * step_loader.cpp
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

// step_loader.cpp
// ISO 10303-21 (STEP Part 21) parser and B-rep entity construction for the
// 3dt viewer. Pure C++17, no external dependencies.
//
// Pipeline: raw text -> comment stripping -> DATA section -> instance map
// (#id -> record tree) -> lazy construction of faces/surfaces/curves starting
// from shell roots (with a global-face fallback).

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
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

// ---------------------------------------------------------------------------
// Part 21 value tree
// ---------------------------------------------------------------------------

struct PValue {
    enum Kind { KUnset, KDerived, KNum, KStr, KRef, KEnum, KList } k = KUnset;
    double num = 0.0;
    int ref = 0;
    std::string str;               // string literal or enum text (without dots)
    std::vector<PValue> list;
};

using Args = std::vector<PValue>;

struct PRec {
    std::string name;              // entity (sub-)type name, uppercase
    Args args;
};

// A #id instance: one record for simple instances, several for complex
// (multi-type) instances like "( B_SPLINE_SURFACE(...) ...WITH_KNOTS(...) )".
struct PEntity {
    std::vector<PRec> recs;
};

using EntityDB = std::unordered_map<int, PEntity>;

// ---------------------------------------------------------------------------
// Lexing / parsing of the DATA section
// ---------------------------------------------------------------------------

// Remove /* ... */ comments, preserving string literals ('...' with ''
// escapes). Comments are replaced by a single space.
std::string stripComments(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    bool inStr = false;
    for (size_t i = 0; i < in.size(); ++i) {
        char c = in[i];
        if (inStr) {
            out.push_back(c);
            if (c == '\'') {
                if (i + 1 < in.size() && in[i + 1] == '\'') {
                    out.push_back('\'');
                    ++i;
                } else {
                    inStr = false;
                }
            }
            continue;
        }
        if (c == '\'') {
            inStr = true;
            out.push_back(c);
            continue;
        }
        if (c == '/' && i + 1 < in.size() && in[i + 1] == '*') {
            size_t j = in.find("*/", i + 2);
            i = (j == std::string::npos) ? in.size() - 1 : j + 1;
            out.push_back(' ');
            continue;
        }
        out.push_back(c);
    }
    return out;
}

class P21Parser {
public:
    explicit P21Parser(const std::string& text) : s(text), n(text.size()) {}

    void parseInstances(EntityDB& db) {
        p = 0;
        while (p < n) {
            skipWs();
            if (p >= n) break;
            if (s[p] != '#') {
                skipStatement();
                continue;
            }
            ++p;
            long id = 0;
            if (!parseInt(id)) {
                skipStatement();
                continue;
            }
            skipWs();
            if (p >= n || s[p] != '=') {
                skipStatement();
                continue;
            }
            ++p;
            skipWs();
            PEntity e;
            bool ok = true;
            if (p < n && s[p] == '(') {
                // Complex (multi-type) instance.
                ++p;
                for (;;) {
                    skipWs();
                    if (p >= n) { ok = false; break; }
                    if (s[p] == ')') { ++p; break; }
                    PRec r;
                    if (!parseRecord(r)) { ok = false; break; }
                    e.recs.push_back(std::move(r));
                }
            } else {
                PRec r;
                if (parseRecord(r)) e.recs.push_back(std::move(r));
                else ok = false;
            }
            skipWs();
            if (p < n && s[p] == ';') ++p;
            else skipStatement();
            if (ok && id > 0 && !e.recs.empty()) db[static_cast<int>(id)] = std::move(e);
        }
    }

private:
    const std::string& s;
    size_t n = 0;
    size_t p = 0;

    void skipWs() {
        while (p < n && (s[p] == ' ' || s[p] == '\t' || s[p] == '\r' || s[p] == '\n' || s[p] == '\f'))
            ++p;
    }

    // Advance past the next top-level ';' (string-aware) to resynchronize.
    void skipStatement() {
        size_t start = p;
        while (p < n) {
            char c = s[p];
            if (c == '\'') {
                ++p;
                while (p < n) {
                    if (s[p] == '\'') {
                        if (p + 1 < n && s[p + 1] == '\'') p += 2;
                        else { ++p; break; }
                    } else {
                        ++p;
                    }
                }
                continue;
            }
            ++p;
            if (c == ';') break;
        }
        if (p == start) ++p; // always make progress
    }

    static bool identChar(char c) {
        return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
               (c >= '0' && c <= '9') || c == '_';
    }

    bool parseInt(long& out) {
        size_t start = p;
        while (p < n && s[p] >= '0' && s[p] <= '9') ++p;
        if (p == start) return false;
        out = std::strtol(s.c_str() + start, nullptr, 10);
        return true;
    }

    bool parseIdent(std::string& out) {
        size_t start = p;
        while (p < n && identChar(s[p])) ++p;
        if (p == start) return false;
        out.assign(s, start, p - start);
        for (char& c : out) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        return true;
    }

    bool parseString(std::string& out) {
        // assumes s[p] == '\''
        ++p;
        out.clear();
        while (p < n) {
            char c = s[p];
            if (c == '\'') {
                if (p + 1 < n && s[p + 1] == '\'') {
                    out.push_back('\'');
                    p += 2;
                } else {
                    ++p;
                    return true;
                }
            } else {
                out.push_back(c);
                ++p;
            }
        }
        return false;
    }

    bool parseNumber(double& out) {
        const char* begin = s.c_str() + p;
        char* end = nullptr;
        out = std::strtod(begin, &end);
        if (end == begin) return false;
        p += static_cast<size_t>(end - begin);
        return true;
    }

    // Parse a comma-separated value list; the opening '(' is already consumed.
    bool parseArgList(Args& out) {
        skipWs();
        if (p < n && s[p] == ')') { ++p; return true; }
        for (;;) {
            PValue v;
            if (!parseValue(v)) return false;
            out.push_back(std::move(v));
            skipWs();
            if (p < n && s[p] == ',') { ++p; skipWs(); continue; }
            if (p < n && s[p] == ')') { ++p; return true; }
            return false;
        }
    }

    bool parseRecord(PRec& r) {
        skipWs();
        if (!parseIdent(r.name)) return false;
        skipWs();
        if (p >= n || s[p] != '(') return false;
        ++p;
        return parseArgList(r.args);
    }

    bool parseValue(PValue& v) {
        skipWs();
        if (p >= n) return false;
        char c = s[p];
        if (c == '(') {
            ++p;
            v.k = PValue::KList;
            return parseArgList(v.list);
        }
        if (c == '#') {
            ++p;
            long id = 0;
            if (!parseInt(id)) return false;
            v.k = PValue::KRef;
            v.ref = static_cast<int>(id);
            return true;
        }
        if (c == '\'') {
            v.k = PValue::KStr;
            return parseString(v.str);
        }
        if (c == '$') { ++p; v.k = PValue::KUnset; return true; }
        if (c == '*') { ++p; v.k = PValue::KDerived; return true; }
        if (c == '.') {
            // Either a real like ".5" (unusual but possible) or an enum .XXX.
            if (p + 1 < n && (std::isdigit(static_cast<unsigned char>(s[p + 1])))) {
                v.k = PValue::KNum;
                return parseNumber(v.num);
            }
            ++p;
            size_t start = p;
            while (p < n && s[p] != '.') ++p;
            if (p >= n) return false;
            v.k = PValue::KEnum;
            v.str.assign(s, start, p - start);
            for (char& ch : v.str) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
            ++p; // closing '.'
            return true;
        }
        if (std::isdigit(static_cast<unsigned char>(c)) || c == '+' || c == '-') {
            v.k = PValue::KNum;
            return parseNumber(v.num);
        }
        if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
            // Typed parameter, e.g. PLANE_ANGLE_MEASURE(0.01745...) or a bare
            // keyword. A single wrapped value is unwrapped transparently.
            std::string name;
            if (!parseIdent(name)) return false;
            skipWs();
            if (p < n && s[p] == '(') {
                ++p;
                Args inner;
                if (!parseArgList(inner)) return false;
                if (inner.size() == 1) {
                    v = std::move(inner[0]);
                } else {
                    v.k = PValue::KList;
                    v.list = std::move(inner);
                }
                return true;
            }
            v.k = PValue::KEnum;
            v.str = std::move(name);
            return true;
        }
        return false;
    }
};

// ---------------------------------------------------------------------------
// Value accessors
// ---------------------------------------------------------------------------

const PValue kUnsetValue{};

const PValue& at(const Args& a, size_t i) {
    return (i < a.size()) ? a[i] : kUnsetValue;
}

double vnum(const PValue& v, double def = 0.0) {
    return (v.k == PValue::KNum) ? v.num : def;
}

int vint(const PValue& v, int def = 0) {
    return (v.k == PValue::KNum) ? static_cast<int>(v.num + (v.num >= 0 ? 0.5 : -0.5)) : def;
}

int vref(const PValue& v) {
    return (v.k == PValue::KRef) ? v.ref : 0;
}

bool vbool(const PValue& v, bool def) {
    if (v.k == PValue::KEnum) {
        if (v.str == "T" || v.str == "TRUE") return true;
        if (v.str == "F" || v.str == "FALSE") return false;
    }
    return def;
}

bool isList(const PValue& v) { return v.k == PValue::KList; }

// ---------------------------------------------------------------------------
// Part 21 string decoding (names shown in the component tree)
//
// ISO 10303-21 control directives are converted to UTF-8:
//   \\          literal backslash
//   \S\c       ISO 8859 character (code of c + 0x80); the \P?\ page selector
//              is skipped and ISO 8859-1 is assumed for the mapping
//   \X\hh      single ISO 8859-1 code point given as two hex digits
//   \X2\...\X0\  UTF-16BE code units, 4 hex digits each (surrogate aware)
//   \X4\...\X0\  UTF-32BE code points, 8 hex digits each
// Unrecognized directives are passed through unchanged.
// ---------------------------------------------------------------------------

void utf8Append(std::string& out, unsigned long cp) {
    if (cp < 0x80) {
        out.push_back(static_cast<char>(cp));
    } else if (cp < 0x800) {
        out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else if (cp < 0x10000) {
        out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
        out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else if (cp <= 0x10FFFF) {
        out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
        out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    }
}

int hexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// Parse 'digits' hex characters at in[i]; returns -1 on any non-hex char.
long hexRun(const std::string& in, size_t i, int digits) {
    if (i + static_cast<size_t>(digits) > in.size()) return -1;
    long v = 0;
    for (int k = 0; k < digits; ++k) {
        int h = hexVal(in[i + static_cast<size_t>(k)]);
        if (h < 0) return -1;
        v = v * 16 + h;
    }
    return v;
}

std::string decodeP21(const std::string& in) {
    if (in.find('\\') == std::string::npos) return in;
    std::string out;
    out.reserve(in.size());
    size_t i = 0;
    const size_t n = in.size();
    while (i < n) {
        char c = in[i];
        if (c != '\\') {
            out.push_back(c);
            ++i;
            continue;
        }
        if (i + 1 < n && in[i + 1] == '\\') {            // "\\" -> backslash
            out.push_back('\\');
            i += 2;
            continue;
        }
        if (i + 3 < n && in[i + 1] == 'S' && in[i + 2] == '\\') {   // \S\c
            utf8Append(out, static_cast<unsigned char>(in[i + 3]) + 0x80UL);
            i += 4;
            continue;
        }
        if (i + 3 < n && in[i + 1] == 'P' && in[i + 3] == '\\') {   // \P?\ page
            i += 4; // selector ignored (ISO 8859-1 assumed for \S\)
            continue;
        }
        if (i + 2 < n && in[i + 1] == 'X') {
            if (in[i + 2] == '\\') {                                 // \X\hh
                long v = hexRun(in, i + 3, 2);
                if (v >= 0) {
                    utf8Append(out, static_cast<unsigned long>(v));
                    i += 5;
                    continue;
                }
            } else if ((in[i + 2] == '2' || in[i + 2] == '4') && i + 3 < n &&
                       in[i + 3] == '\\') {                    // \X2\ or \X4\ run
                const int digits = (in[i + 2] == '2') ? 4 : 8;
                size_t j = i + 4;
                std::string tmp;
                unsigned long high = 0; // pending UTF-16 high surrogate
                bool ok = true;
                while (j < n && in[j] != '\\') {
                    long v = hexRun(in, j, digits);
                    if (v < 0) { ok = false; break; }
                    unsigned long cp = static_cast<unsigned long>(v);
                    if (digits == 4 && cp >= 0xD800 && cp <= 0xDBFF) {
                        high = cp;
                    } else if (digits == 4 && cp >= 0xDC00 && cp <= 0xDFFF && high) {
                        utf8Append(tmp, 0x10000 + ((high - 0xD800) << 10) + (cp - 0xDC00));
                        high = 0;
                    } else {
                        utf8Append(tmp, cp);
                    }
                    j += static_cast<size_t>(digits);
                }
                // expect the \X0\ terminator
                if (ok && j + 3 < n && in[j] == '\\' && in[j + 1] == 'X' &&
                    in[j + 2] == '0' && in[j + 3] == '\\') {
                    out += tmp;
                    i = j + 4;
                    continue;
                }
            }
        }
        out.push_back(c); // unknown directive: keep the raw character
        ++i;
    }
    return out;
}

std::string strArg(const Args& a, size_t i) {
    const PValue& v = at(a, i);
    return (v.k == PValue::KStr) ? v.str : std::string();
}

// Decode + trim a raw Part 21 name; placeholder names ("", "NONE") map to "".
std::string cleanName(const std::string& raw) {
    std::string s = decodeP21(raw);
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return std::string();
    size_t e = s.find_last_not_of(" \t\r\n");
    s = s.substr(b, e - b + 1);
    if (s.size() == 4 && (s[0] == 'N' || s[0] == 'n') && (s[1] == 'O' || s[1] == 'o') &&
        (s[2] == 'N' || s[2] == 'n') && (s[3] == 'E' || s[3] == 'e'))
        return std::string();
    return s;
}

// ---------------------------------------------------------------------------
// Entity builder: turns record trees into the geometric model
// ---------------------------------------------------------------------------

class Builder {
public:
    explicit Builder(const EntityDB& db_) : db(db_) {}

    const PEntity* get(int id) const {
        auto it = db.find(id);
        return (it == db.end()) ? nullptr : &it->second;
    }

    static const Args* recArgs(const PEntity& e, const char* name) {
        for (const PRec& r : e.recs)
            if (r.name == name) return &r.args;
        return nullptr;
    }

    static bool hasType(const PEntity& e, const char* name) {
        return recArgs(e, name) != nullptr;
    }

    // ---- basic geometry -------------------------------------------------

    bool getPoint(int id, Vec3d& out) const {
        const PEntity* e = get(id);
        if (!e) return false;
        const Args* a = recArgs(*e, "CARTESIAN_POINT");
        if (!a) return false;
        const PValue* lst = nullptr;
        for (const PValue& v : *a)
            if (isList(v)) { lst = &v; break; }
        if (!lst || lst->list.empty()) return false;
        out.x = vnum(at(lst->list, 0));
        out.y = vnum(at(lst->list, 1));
        out.z = vnum(at(lst->list, 2));
        return true;
    }

    bool getDirection(int id, Vec3d& out) const {
        const PEntity* e = get(id);
        if (!e) return false;
        const Args* a = recArgs(*e, "DIRECTION");
        if (!a) return false;
        const PValue* lst = nullptr;
        for (const PValue& v : *a)
            if (isList(v)) { lst = &v; break; }
        if (!lst || lst->list.empty()) return false;
        out.x = vnum(at(lst->list, 0));
        out.y = vnum(at(lst->list, 1));
        out.z = vnum(at(lst->list, 2));
        return true;
    }

    bool getVector(int id, Vec3d& out) const {
        const PEntity* e = get(id);
        if (!e) return false;
        const Args* a = recArgs(*e, "VECTOR");
        if (!a) return false;
        Vec3d d{0, 0, 1};
        double mag = 1.0;
        bool haveDir = false;
        for (const PValue& v : *a) {
            if (v.k == PValue::KRef && !haveDir)
                haveDir = getDirection(v.ref, d);
            else if (v.k == PValue::KNum)
                mag = v.num;
        }
        if (!haveDir) return false;
        out = step::normalized(d) * mag;
        return true;
    }

    static Frame frameFromZX(const Vec3d& o, Vec3d z, Vec3d xr, bool haveZ, bool haveX) {
        Frame f;
        f.o = o;
        f.az = haveZ ? step::normalized(z) : Vec3d{0, 0, 1};
        Vec3d x;
        if (haveX) {
            x = xr - f.az * step::dot(xr, f.az);
        }
        if (!haveX || step::norm(x) < 1e-9) {
            // pick any vector not parallel to az
            Vec3d cand = (std::fabs(f.az.x) < 0.9) ? Vec3d{1, 0, 0} : Vec3d{0, 1, 0};
            x = cand - f.az * step::dot(cand, f.az);
        }
        f.ax = step::normalized(x);
        f.ay = step::cross(f.az, f.ax);
        return f;
    }

    bool getAxis2(int id, Frame& out) const {
        const PEntity* e = get(id);
        if (!e) return false;
        const Args* a = recArgs(*e, "AXIS2_PLACEMENT_3D");
        if (!a) a = recArgs(*e, "AXIS2_PLACEMENT_2D");
        if (!a) return false;
        Vec3d o{0, 0, 0}, z{0, 0, 1}, x{1, 0, 0};
        bool haveZ = false, haveX = false;
        int refIdx = 0;
        for (const PValue& v : *a) {
            if (v.k != PValue::KRef) continue;
            if (refIdx == 0) getPoint(v.ref, o);
            else if (refIdx == 1) haveZ = getDirection(v.ref, z);
            else if (refIdx == 2) haveX = getDirection(v.ref, x);
            ++refIdx;
        }
        // Note: for AXIS2_PLACEMENT_3D the arg order is (name, location,
        // axis, ref_direction); '$' entries simply don't count as refs, which
        // can shift meaning. Handle the common '$ axis' case: if only two
        // refs and second arg was '$', the second ref is ref_direction.
        if (a->size() >= 4 && at(*a, 2).k == PValue::KUnset && refIdx == 2) {
            haveX = haveZ;
            x = z;
            haveZ = false;
            z = {0, 0, 1};
        }
        out = frameFromZX(o, z, x, haveZ, haveX);
        return true;
    }

    bool getAxis1(int id, Frame& out) const {
        const PEntity* e = get(id);
        if (!e) return false;
        const Args* a = recArgs(*e, "AXIS1_PLACEMENT");
        if (!a) return getAxis2(id, out); // tolerate AXIS2 where AXIS1 expected
        Vec3d o{0, 0, 0}, z{0, 0, 1};
        bool haveZ = false;
        int refIdx = 0;
        for (const PValue& v : *a) {
            if (v.k != PValue::KRef) continue;
            if (refIdx == 0) getPoint(v.ref, o);
            else if (refIdx == 1) haveZ = getDirection(v.ref, z);
            ++refIdx;
        }
        out = frameFromZX(o, z, {1, 0, 0}, haveZ, false);
        return true;
    }

    // ---- knot vector helpers -------------------------------------------

    static std::vector<double> expandKnots(const PValue& mults, const PValue& knots) {
        std::vector<double> out;
        if (!isList(mults) || !isList(knots)) return out;
        size_t cnt = std::min(mults.list.size(), knots.list.size());
        for (size_t i = 0; i < cnt; ++i) {
            int m = vint(mults.list[i], 1);
            double kv = vnum(knots.list[i]);
            for (int j = 0; j < m; ++j) out.push_back(kv);
        }
        return out;
    }

    // Generated knot vectors for b-splines given without explicit knots.
    static std::vector<double> makeKnots(int nctrl, int deg, const char* style) {
        std::vector<double> k;
        if (nctrl < deg + 1) return k;
        if (std::strcmp(style, "UNIFORM") == 0) {
            for (int i = 0; i < nctrl + deg + 1; ++i)
                k.push_back(static_cast<double>(i - deg));
            return k;
        }
        if (std::strcmp(style, "BEZIER") == 0 && deg > 0 && (nctrl - 1) % deg == 0) {
            int spans = (nctrl - 1) / deg;
            for (int i = 0; i <= deg; ++i) k.push_back(0.0);
            for (int sSpan = 1; sSpan < spans; ++sSpan)
                for (int i = 0; i < deg; ++i) k.push_back(static_cast<double>(sSpan));
            for (int i = 0; i <= deg; ++i) k.push_back(static_cast<double>(spans));
            return k;
        }
        // quasi-uniform / default: clamped, single interior knots
        int inner = nctrl - deg - 1;
        for (int i = 0; i <= deg; ++i) k.push_back(0.0);
        for (int i = 1; i <= inner; ++i)
            k.push_back(static_cast<double>(i) / (inner + 1));
        for (int i = 0; i <= deg; ++i) k.push_back(1.0);
        return k;
    }

    // ---- b-spline curves ------------------------------------------------

    bool fillBSCurve(const PEntity& e, BSCurve& bs) const {
        static const char* kCurveNames[] = {
            "B_SPLINE_CURVE_WITH_KNOTS", "BEZIER_CURVE", "UNIFORM_CURVE",
            "QUASI_UNIFORM_CURVE", "B_SPLINE_CURVE"};
        const Args* base = recArgs(e, "B_SPLINE_CURVE");
        const Args* named = nullptr;   // record that carries the full flat arg list
        if (!base || e.recs.size() == 1) {
            for (const char* nm : kCurveNames) {
                if (const Args* a = recArgs(e, nm)) { named = a; break; }
            }
            base = named;
        }
        if (!base) return false;
        size_t off = (!base->empty() && (*base)[0].k == PValue::KStr) ? 1 : 0;
        bs.degree = vint(at(*base, off + 0), -1);
        const PValue& ctrlList = at(*base, off + 1);
        if (bs.degree < 1 || bs.degree > 15 || !isList(ctrlList)) return false;
        bs.ctrl.clear();
        for (const PValue& v : ctrlList.list) {
            Vec3d pnt;
            if (v.k != PValue::KRef || !getPoint(v.ref, pnt)) return false;
            bs.ctrl.push_back(pnt);
        }
        if (bs.ctrl.size() < static_cast<size_t>(bs.degree + 1)) return false;
        bs.closed = vbool(at(*base, off + 3), false);

        // knots
        bs.knots.clear();
        if (const Args* wk = recArgs(e, "B_SPLINE_CURVE_WITH_KNOTS")) {
            if (wk == base) {
                bs.knots = expandKnots(at(*wk, off + 5), at(*wk, off + 6));
            } else {
                size_t woff = (!wk->empty() && (*wk)[0].k == PValue::KStr) ? 1 : 0;
                bs.knots = expandKnots(at(*wk, woff + 0), at(*wk, woff + 1));
            }
        }
        if (bs.knots.size() != bs.ctrl.size() + bs.degree + 1) {
            const char* style = "QUASI";
            if (hasType(e, "BEZIER_CURVE")) style = "BEZIER";
            else if (hasType(e, "UNIFORM_CURVE")) style = "UNIFORM";
            bs.knots = makeKnots(static_cast<int>(bs.ctrl.size()), bs.degree, style);
            if (bs.knots.size() != bs.ctrl.size() + bs.degree + 1) return false;
        }

        // weights
        bs.weights.clear();
        if (const Args* rw = recArgs(e, "RATIONAL_B_SPLINE_CURVE")) {
            const PValue* wl = nullptr;
            for (const PValue& v : *rw)
                if (isList(v)) { wl = &v; break; }
            if (wl && wl->list.size() == bs.ctrl.size()) {
                for (const PValue& v : wl->list) bs.weights.push_back(vnum(v, 1.0));
            }
        }
        return true;
    }

    // ---- b-spline surfaces ---------------------------------------------

    bool fillBSSurf(const PEntity& e, BSSurf& bs) const {
        static const char* kSurfNames[] = {
            "B_SPLINE_SURFACE_WITH_KNOTS", "BEZIER_SURFACE", "UNIFORM_SURFACE",
            "QUASI_UNIFORM_SURFACE", "B_SPLINE_SURFACE"};
        const Args* base = recArgs(e, "B_SPLINE_SURFACE");
        const Args* named = nullptr;
        if (!base || e.recs.size() == 1) {
            for (const char* nm : kSurfNames) {
                if (const Args* a = recArgs(e, nm)) { named = a; break; }
            }
            base = named;
        }
        if (!base) return false;
        size_t off = (!base->empty() && (*base)[0].k == PValue::KStr) ? 1 : 0;
        bs.udeg = vint(at(*base, off + 0), -1);
        bs.vdeg = vint(at(*base, off + 1), -1);
        const PValue& net = at(*base, off + 2);
        if (bs.udeg < 1 || bs.udeg > 15 || bs.vdeg < 1 || bs.vdeg > 15 || !isList(net))
            return false;
        bs.nu = static_cast<int>(net.list.size());
        bs.nv = 0;
        bs.ctrl.clear();
        for (const PValue& row : net.list) {
            if (!isList(row)) return false;
            if (bs.nv == 0) bs.nv = static_cast<int>(row.list.size());
            if (static_cast<int>(row.list.size()) != bs.nv) return false;
            for (const PValue& v : row.list) {
                Vec3d pnt;
                if (v.k != PValue::KRef || !getPoint(v.ref, pnt)) return false;
                bs.ctrl.push_back(pnt);
            }
        }
        if (bs.nu < bs.udeg + 1 || bs.nv < bs.vdeg + 1) return false;
        bs.uclosed = vbool(at(*base, off + 4), false);
        bs.vclosed = vbool(at(*base, off + 5), false);

        // knots
        bs.uknots.clear();
        bs.vknots.clear();
        if (const Args* wk = recArgs(e, "B_SPLINE_SURFACE_WITH_KNOTS")) {
            if (wk == base) {
                bs.uknots = expandKnots(at(*wk, off + 7), at(*wk, off + 9));
                bs.vknots = expandKnots(at(*wk, off + 8), at(*wk, off + 10));
            } else {
                size_t woff = (!wk->empty() && (*wk)[0].k == PValue::KStr) ? 1 : 0;
                bs.uknots = expandKnots(at(*wk, woff + 0), at(*wk, woff + 2));
                bs.vknots = expandKnots(at(*wk, woff + 1), at(*wk, woff + 3));
            }
        }
        const char* style = "QUASI";
        if (hasType(e, "BEZIER_SURFACE")) style = "BEZIER";
        else if (hasType(e, "UNIFORM_SURFACE")) style = "UNIFORM";
        if (bs.uknots.size() != static_cast<size_t>(bs.nu + bs.udeg + 1))
            bs.uknots = makeKnots(bs.nu, bs.udeg, style);
        if (bs.vknots.size() != static_cast<size_t>(bs.nv + bs.vdeg + 1))
            bs.vknots = makeKnots(bs.nv, bs.vdeg, style);
        if (bs.uknots.size() != static_cast<size_t>(bs.nu + bs.udeg + 1) ||
            bs.vknots.size() != static_cast<size_t>(bs.nv + bs.vdeg + 1))
            return false;

        // weights
        bs.weights.clear();
        if (const Args* rw = recArgs(e, "RATIONAL_B_SPLINE_SURFACE")) {
            const PValue* wl = nullptr;
            for (const PValue& v : *rw)
                if (isList(v)) { wl = &v; break; }
            if (wl && static_cast<int>(wl->list.size()) == bs.nu) {
                std::vector<double> w;
                bool ok = true;
                for (const PValue& row : wl->list) {
                    if (!isList(row) || static_cast<int>(row.list.size()) != bs.nv) {
                        ok = false;
                        break;
                    }
                    for (const PValue& v : row.list) w.push_back(vnum(v, 1.0));
                }
                if (ok) bs.weights = std::move(w);
            }
        }
        return true;
    }

    // ---- curves ---------------------------------------------------------

    bool buildCurve(int id, Curve& out, int depth) const {
        if (depth > 8) return false;
        const PEntity* e = get(id);
        if (!e) return false;

        if (const Args* a = recArgs(*e, "LINE")) {
            Vec3d p0, d;
            if (a->size() >= 3 && getPoint(vref(at(*a, 1)), p0) &&
                getVector(vref(at(*a, 2)), d)) {
                out.kind = CurveKind::Line;
                out.p0 = p0;
                out.dir = d;
                return true;
            }
            return false;
        }
        if (const Args* a = recArgs(*e, "CIRCLE")) {
            Frame f;
            if (a->size() >= 3 && getAxis2(vref(at(*a, 1)), f)) {
                out.kind = CurveKind::Circle;
                out.frame = f;
                out.r1 = vnum(at(*a, 2));
                return out.r1 > 0;
            }
            return false;
        }
        if (const Args* a = recArgs(*e, "ELLIPSE")) {
            Frame f;
            if (a->size() >= 4 && getAxis2(vref(at(*a, 1)), f)) {
                out.kind = CurveKind::Ellipse;
                out.frame = f;
                out.r1 = vnum(at(*a, 2));
                out.r2 = vnum(at(*a, 3));
                return out.r1 > 0 && out.r2 > 0;
            }
            return false;
        }
        if (const Args* a = recArgs(*e, "POLYLINE")) {
            const PValue* lst = nullptr;
            for (const PValue& v : *a)
                if (isList(v)) { lst = &v; break; }
            if (!lst) return false;
            out.poly.clear();
            for (const PValue& v : lst->list) {
                Vec3d pnt;
                if (v.k != PValue::KRef || !getPoint(v.ref, pnt)) return false;
                out.poly.push_back(pnt);
            }
            if (out.poly.size() < 2) return false;
            out.kind = CurveKind::Polyline;
            return true;
        }
        if (const Args* a = recArgs(*e, "TRIMMED_CURVE")) {
            // Use the basis curve; trimming comes from edge vertices anyway.
            if (a->size() >= 2) return buildCurve(vref(at(*a, 1)), out, depth + 1);
            return false;
        }
        if (hasType(*e, "B_SPLINE_CURVE") || hasType(*e, "B_SPLINE_CURVE_WITH_KNOTS") ||
            hasType(*e, "BEZIER_CURVE") || hasType(*e, "UNIFORM_CURVE") ||
            hasType(*e, "QUASI_UNIFORM_CURVE") || hasType(*e, "RATIONAL_B_SPLINE_CURVE")) {
            if (fillBSCurve(*e, out.bs)) {
                out.kind = CurveKind::BSpline;
                return true;
            }
            return false;
        }
        return false;
    }

    // ---- surfaces -------------------------------------------------------

    bool buildSurface(int id, Surface& out) const {
        const PEntity* e = get(id);
        if (!e) return false;

        if (const Args* a = recArgs(*e, "PLANE")) {
            if (a->size() >= 2 && getAxis2(vref(at(*a, 1)), out.frame)) {
                out.kind = SurfKind::Plane;
                return true;
            }
            return false;
        }
        if (const Args* a = recArgs(*e, "CYLINDRICAL_SURFACE")) {
            if (a->size() >= 3 && getAxis2(vref(at(*a, 1)), out.frame)) {
                out.kind = SurfKind::Cylinder;
                out.r1 = vnum(at(*a, 2));
                return out.r1 > 0;
            }
            return false;
        }
        if (const Args* a = recArgs(*e, "CONICAL_SURFACE")) {
            if (a->size() >= 4 && getAxis2(vref(at(*a, 1)), out.frame)) {
                out.kind = SurfKind::Cone;
                out.r1 = vnum(at(*a, 2));
                out.angle = vnum(at(*a, 3));
                return out.r1 >= 0 && std::fabs(out.angle) < 1.56;
            }
            return false;
        }
        if (const Args* a = recArgs(*e, "SPHERICAL_SURFACE")) {
            if (a->size() >= 3 && getAxis2(vref(at(*a, 1)), out.frame)) {
                out.kind = SurfKind::Sphere;
                out.r1 = vnum(at(*a, 2));
                return out.r1 > 0;
            }
            return false;
        }
        if (const Args* a = recArgs(*e, "TOROIDAL_SURFACE")) {
            if (a->size() >= 4 && getAxis2(vref(at(*a, 1)), out.frame)) {
                out.kind = SurfKind::Torus;
                out.r1 = vnum(at(*a, 2));
                out.r2 = vnum(at(*a, 3));
                return out.r1 > 0 && out.r2 > 0;
            }
            return false;
        }
        if (const Args* a = recArgs(*e, "SURFACE_OF_LINEAR_EXTRUSION")) {
            if (a->size() >= 3 && buildCurve(vref(at(*a, 1)), out.base, 0) &&
                getVector(vref(at(*a, 2)), out.sweepDir) &&
                step::norm(out.sweepDir) > 1e-12) {
                out.kind = SurfKind::Extrusion;
                return true;
            }
            return false;
        }
        if (const Args* a = recArgs(*e, "SURFACE_OF_REVOLUTION")) {
            if (a->size() >= 3 && buildCurve(vref(at(*a, 1)), out.base, 0) &&
                getAxis1(vref(at(*a, 2)), out.frame)) {
                out.kind = SurfKind::Revolution;
                return true;
            }
            return false;
        }
        if (hasType(*e, "B_SPLINE_SURFACE") || hasType(*e, "B_SPLINE_SURFACE_WITH_KNOTS") ||
            hasType(*e, "BEZIER_SURFACE") || hasType(*e, "UNIFORM_SURFACE") ||
            hasType(*e, "QUASI_UNIFORM_SURFACE") || hasType(*e, "RATIONAL_B_SPLINE_SURFACE")) {
            if (fillBSSurf(*e, out.bs)) {
                out.kind = SurfKind::BSpline;
                return true;
            }
            return false;
        }
        return false;
    }

    // ---- topology -------------------------------------------------------

    bool getVertexPoint(int id, Vec3d& out) const {
        const PEntity* e = get(id);
        if (!e) return false;
        if (const Args* a = recArgs(*e, "VERTEX_POINT")) {
            for (const PValue& v : *a)
                if (v.k == PValue::KRef && getPoint(v.ref, out)) return true;
            return false;
        }
        return getPoint(id, out); // tolerate a direct CARTESIAN_POINT
    }

    bool buildEdge(int oeId, step::Edge& out) const {
        const PEntity* e = get(oeId);
        if (!e) return false;
        bool orient = true;
        if (const Args* a = recArgs(*e, "ORIENTED_EDGE")) {
            // (name, edge_start[*], edge_end[*], edge_element, orientation)
            if (a->size() < 5) return false;
            orient = vbool(at(*a, 4), true);
            e = get(vref(at(*a, 3)));
            if (!e) return false;
        }
        const Args* ec = recArgs(*e, "EDGE_CURVE");
        if (!ec || ec->size() < 5) return false;
        if (!getVertexPoint(vref(at(*ec, 1)), out.pstart)) return false;
        if (!getVertexPoint(vref(at(*ec, 2)), out.pend)) return false;
        out.curveSense = vbool(at(*ec, 4), true);
        out.orientation = orient;
        int cid = vref(at(*ec, 3));
        if (cid == 0 || !buildCurve(cid, out.curve, 0))
            out.curve = Curve{}; // straight fallback
        return true;
    }

    // Build a loop from an EDGE_LOOP / POLY_LOOP entity; returns false if the
    // loop is unusable.
    bool buildLoop(int loopId, bool boundOrient, step::Loop& out) const {
        const PEntity* e = get(loopId);
        if (!e) return false;
        if (const Args* a = recArgs(*e, "EDGE_LOOP")) {
            const PValue* lst = nullptr;
            for (const PValue& v : *a)
                if (isList(v)) { lst = &v; break; }
            if (!lst || lst->list.empty()) return false;
            for (const PValue& v : lst->list) {
                step::Edge ed;
                if (v.k != PValue::KRef || !buildEdge(v.ref, ed)) return false;
                out.edges.push_back(std::move(ed));
            }
        } else if (const Args* pl = recArgs(*e, "POLY_LOOP")) {
            const PValue* lst = nullptr;
            for (const PValue& v : *pl)
                if (isList(v)) { lst = &v; break; }
            if (!lst || lst->list.size() < 3) return false;
            std::vector<Vec3d> pts;
            for (const PValue& v : lst->list) {
                Vec3d pnt;
                if (v.k != PValue::KRef || !getPoint(v.ref, pnt)) return false;
                pts.push_back(pnt);
            }
            for (size_t i = 0; i < pts.size(); ++i) {
                step::Edge ed;
                ed.pstart = pts[i];
                ed.pend = pts[(i + 1) % pts.size()];
                out.edges.push_back(std::move(ed));
            }
        } else {
            return false; // VERTEX_LOOP or unknown -> no usable boundary
        }
        if (!boundOrient) {
            std::reverse(out.edges.begin(), out.edges.end());
            for (step::Edge& ed : out.edges) ed.orientation = !ed.orientation;
        }
        return true;
    }

    bool buildFace(int id, step::Face& out) const {
        const PEntity* e = get(id);
        if (!e) return false;
        const Args* fa = recArgs(*e, "ADVANCED_FACE");
        if (!fa) fa = recArgs(*e, "FACE_SURFACE");
        if (!fa || fa->size() < 4) return false;
        // (name, bounds, face_geometry, same_sense)
        if (!buildSurface(vref(at(*fa, 2)), out.surf)) return false;
        out.sameSense = vbool(at(*fa, 3), true);
        const PValue& bounds = at(*fa, 1);
        if (!isList(bounds)) return false;
        for (const PValue& bv : bounds.list) {
            if (bv.k != PValue::KRef) continue;
            const PEntity* be = get(bv.ref);
            if (!be) continue;
            const Args* ba = recArgs(*be, "FACE_OUTER_BOUND");
            bool isOuter = (ba != nullptr);
            if (!ba) ba = recArgs(*be, "FACE_BOUND");
            if (!ba || ba->size() < 3) continue;
            step::Loop lp;
            lp.outer = isOuter;
            bool orient = vbool(at(*ba, 2), true);
            if (!buildLoop(vref(at(*ba, 1)), orient, lp)) {
                if (isOuter) return false; // broken outer boundary -> skip face
                continue;                  // broken hole -> ignore hole
            }
            out.loops.push_back(std::move(lp));
        }
        // A face with no loops is kept only for closed surfaces; the
        // tessellator decides whether it can handle it.
        return true;
    }

private:
    const EntityDB& db;
};

// ---------------------------------------------------------------------------
// Root discovery
// ---------------------------------------------------------------------------

// One shell of the flat (non-assembly) path: its faces plus a display name
// taken from the owning solid item, the shell itself, or empty.
struct ShellGroup {
    std::string name;
    std::vector<int> faceIds;
};

// Collect the faces referenced by shells, one group per shell (a face is
// assigned to the first shell that references it, so groups are disjoint).
// If no shell exists, fall back to a single group with every face entity in
// the file. Walking every shell covers MANIFOLD_SOLID_BREP, BREP_WITH_VOIDS,
// SHELL_BASED_SURFACE_MODEL, ADVANCED_BREP_SHAPE_REPRESENTATION... without
// explicit navigation, exactly like the previous flat collector.
std::vector<ShellGroup> collectShellGroups(const EntityDB& db, const Builder& b) {
    std::vector<ShellGroup> out;
    std::unordered_set<int> seen;
    std::vector<int> ids;
    ids.reserve(db.size());
    for (const auto& kv : db) ids.push_back(kv.first);
    std::sort(ids.begin(), ids.end()); // deterministic order

    // Pass 1: names of the solid items that own each shell.
    std::unordered_map<int, std::string> ownerName;
    static const char* kOwners[] = {"MANIFOLD_SOLID_BREP", "BREP_WITH_VOIDS",
                                    "FACETED_BREP", "SHELL_BASED_SURFACE_MODEL"};
    for (int id : ids) {
        const PEntity& e = db.at(id);
        for (const char* nm : kOwners) {
            const Args* a = Builder::recArgs(e, nm);
            if (!a) continue;
            std::string oname = cleanName(strArg(*a, 0));
            if (oname.empty()) continue;
            for (const PValue& v : *a) {
                if (v.k == PValue::KRef) {
                    ownerName.emplace(v.ref, oname);
                } else if (isList(v)) {
                    for (const PValue& s : v.list)
                        if (s.k == PValue::KRef) ownerName.emplace(s.ref, oname);
                }
            }
        }
    }

    auto addShellFaces = [&](const PEntity& e, std::vector<int>& faces) {
        const Args* a = Builder::recArgs(e, "CLOSED_SHELL");
        if (!a) a = Builder::recArgs(e, "OPEN_SHELL");
        if (!a) return;
        for (const PValue& v : *a) {
            if (!isList(v)) continue;
            for (const PValue& f : v.list)
                if (f.k == PValue::KRef && f.ref > 0 && seen.insert(f.ref).second)
                    faces.push_back(f.ref);
        }
    };

    // Pass 2: one group per shell entity (or oriented-shell wrapper).
    for (int id : ids) {
        const PEntity& e = db.at(id);
        const Args* sa = Builder::recArgs(e, "CLOSED_SHELL");
        if (!sa) sa = Builder::recArgs(e, "OPEN_SHELL");
        const Args* oa = Builder::recArgs(e, "ORIENTED_CLOSED_SHELL");
        if (!oa) oa = Builder::recArgs(e, "ORIENTED_OPEN_SHELL");
        if (!sa && !oa) continue;
        ShellGroup g;
        auto own = ownerName.find(id);
        if (own != ownerName.end()) g.name = own->second;
        if (sa) {
            addShellFaces(e, g.faceIds);
            if (g.name.empty()) g.name = cleanName(strArg(*sa, 0));
        }
        if (oa) {
            for (const PValue& v : *oa) {
                if (v.k != PValue::KRef) continue;
                const PEntity* se = b.get(v.ref);
                if (!se) continue;
                addShellFaces(*se, g.faceIds);
                if (g.name.empty()) {
                    auto wrapped = ownerName.find(v.ref);
                    if (wrapped != ownerName.end()) g.name = wrapped->second;
                }
                if (g.name.empty()) {
                    const Args* wa = Builder::recArgs(*se, "CLOSED_SHELL");
                    if (!wa) wa = Builder::recArgs(*se, "OPEN_SHELL");
                    if (wa) g.name = cleanName(strArg(*wa, 0));
                }
            }
            if (g.name.empty()) g.name = cleanName(strArg(*oa, 0));
        }
        if (!g.faceIds.empty()) out.push_back(std::move(g));
    }

    if (out.empty()) {
        ShellGroup g;
        for (int id : ids) {
            const PEntity& e = db.at(id);
            if ((Builder::hasType(e, "ADVANCED_FACE") || Builder::hasType(e, "FACE_SURFACE")) &&
                seen.insert(id).second)
                g.faceIds.push_back(id);
        }
        if (!g.faceIds.empty()) out.push_back(std::move(g));
    }
    return out;
}

// ---------------------------------------------------------------------------
// Assembly structure (AP203 / AP214 / AP242 product structure)
//
// Supported instancing mechanisms:
//  1. NEXT_ASSEMBLY_USAGE_OCCURRENCE (relating = parent product definition,
//     related = child) + CONTEXT_DEPENDENT_SHAPE_REPRESENTATION pointing to a
//     (SHAPE_)REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION (usually a
//     complex multi-type instance) whose operator is an
//     ITEM_DEFINED_TRANSFORMATION between two AXIS2_PLACEMENT_3D.
//  2. MAPPED_ITEM + REPRESENTATION_MAP inside the items of a
//     SHAPE_REPRESENTATION.
//
// Transformation direction (verified with the layout produced by the common
// CAD exporters, e.g. the classic as1-oc-214 sample written by Open CASCADE
// based systems):
//     #rr = ( REPRESENTATION_RELATIONSHIP('','',#childRep,#parentRep)
//             REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#idt)
//             SHAPE_REPRESENTATION_RELATIONSHIP() );
//     #idt = ITEM_DEFINED_TRANSFORMATION('','',#childSideAxis,#placement);
// i.e. rep_1 is the representation of the CHILD (component), rep_2 the one of
// the PARENT (assembly); transform_item_1 pairs with rep_1 (almost always an
// identity axis at the child origin) and transform_item_2 pairs with rep_2
// (the placement of the instance in parent coordinates). The child -> parent
// transform is therefore  M(item_2) * M(item_1)^-1.  Some exporters swap
// rep_1/rep_2 (and with them item_1/item_2); this is detected by checking
// which representation belongs to the NAUO's related (child) product, and the
// transform is inverted accordingly:  M(item_1) * M(item_2)^-1.
//
// For MAPPED_ITEM the mapped representation is positioned by carrying the
// frame of REPRESENTATION_MAP.mapping_origin onto MAPPED_ITEM.mapping_target:
//     T = M(mapping_target) * M(mapping_origin)^-1.
// ---------------------------------------------------------------------------

using step::Mat43;

std::vector<int> refsOf(const Args& a) {
    std::vector<int> r;
    for (const PValue& v : a)
        if (v.k == PValue::KRef) r.push_back(v.ref);
    return r;
}

class AsmResolver {
public:
    AsmResolver(const EntityDB& db_, const Builder& b_) : db(db_), b(b_) { scan(); }

    // True when the file carries any assembly structure at all.
    bool active() const { return !nauos.empty() || haveMapped; }

    int warningCount() const { return warnings; }

    // Build model.parts (unique geometry + instance transforms) from the
    // assembly structure. Returns false when the structure yields no usable
    // geometry; the caller then falls back to the flat single-part path.
    bool buildModel(step::Model& model) {
        resolveOccurrences();

        // ---- roots: product definitions never 'related' in a NAUO ---------
        std::unordered_set<int> childPDs;
        for (const Nauo& n : nauos) childPDs.insert(n.child);
        std::vector<int> rootCand;
        for (const auto& kv : pdToReps) rootCand.push_back(kv.first);
        for (const Nauo& n : nauos) rootCand.push_back(n.parent);
        std::sort(rootCand.begin(), rootCand.end());
        rootCand.erase(std::unique(rootCand.begin(), rootCand.end()), rootCand.end());

        std::vector<int> pdStack, repStack;
        for (int pd : rootCand) {
            if (childPDs.count(pd)) continue;
            // A product whose every representation is instanced elsewhere
            // (mapped source / transformed child) is not a real root: using
            // it would duplicate the placed geometry.
            auto it = pdToReps.find(pd);
            if (it != pdToReps.end()) {
                bool hasOwn = false;
                for (int rep : it->second) {
                    int g = groupOf(rep);
                    if (!mappedSourceGroups.count(g) && !childRepGroups.count(g)) {
                        hasOwn = true;
                        break;
                    }
                }
                bool hasChildren = false;
                for (const Nauo& n : nauos)
                    if (n.parent == pd) { hasChildren = true; break; }
                if (!hasOwn && !hasChildren) continue;
            }
            std::string rootName = productNameOfPD(pd);
            if (rootName.empty()) rootName = "Assembly";
            placePD(pd, Mat43{}, 0, pdStack, repStack, -1, rootName);
        }

        // ---- no product tree: place root representations directly ---------
        // (files that only use MAPPED_ITEM, without SDR/NAUO product links)
        if (placedList.empty()) {
            std::unordered_set<int> done;
            for (int rep : repEntities) {
                int g = groupOf(rep);
                if (!done.insert(g).second) continue;
                if (mappedSourceGroups.count(g) || childRepGroups.count(g)) continue;
                const RepGroup& G = groupData(g);
                if (G.faceIds.empty() && G.mappedItems.empty()) continue;
                std::vector<int> stack;
                placeRep(rep, Mat43{}, 0, stack, -1);
            }
        }

        if (placedList.empty()) return false;

        // ---- assemble parts (one per unique group, in placement order) ----
        std::unordered_map<int, size_t> partIndex;
        size_t builtFaces = 0;
        std::vector<int> plPart(placedList.size(), -1);
        std::vector<int> plInst(placedList.size(), -1);
        for (size_t pi = 0; pi < placedList.size(); ++pi) {
            const Placed& pl = placedList[pi];
            auto it = partIndex.find(pl.group);
            size_t idx;
            if (it == partIndex.end()) {
                idx = model.parts.size();
                partIndex[pl.group] = idx;
                model.parts.emplace_back();
                const RepGroup& G = groupData(pl.group);
                model.totalFaces += static_cast<int>(G.faceIds.size());
                for (int fid : G.faceIds) {
                    step::Face f;
                    bool ok = false;
                    try {
                        ok = b.buildFace(fid, f);
                    } catch (...) {
                        ok = false;
                    }
                    if (ok) model.parts[idx].faces.push_back(std::move(f));
                    else ++model.skippedFaces;
                }
                builtFaces += model.parts[idx].faces.size();
            } else {
                idx = it->second;
            }
            plPart[pi] = static_cast<int>(idx);
            plInst[pi] = static_cast<int>(model.parts[idx].instances.size());
            model.parts[idx].instances.push_back(pl.xf);
        }
        if (builtFaces == 0) return false;
        buildNodes(model, plPart, plInst);
        return true;
    }

private:
    struct Nauo {
        int id = 0, parent = 0, child = 0;
        std::string name;              // occurrence display name (may be empty)
    };
    struct Cdsr {
        int rr = 0, pds = 0;
    };
    struct RepMapInfo {
        int origin = 0, mappedRep = 0;
    };
    struct MappedItemInfo {
        int mapId = 0, target = 0;
    };
    struct RepGroup {
        std::vector<int> faceIds;      // unique faces, discovery order
        std::vector<int> mappedItems;  // MAPPED_ITEM entity ids
        std::string itemName;          // name of the first face-bearing item
    };
    struct OccXf {
        Mat43 xf;
        bool warned = false;
    };
    // One node of the occurrence tree built during placement.
    struct TreeNode {
        std::string name;
        int parent = -1;
        bool hasChildren = false;      // has child tree nodes (sub-occurrences)
    };
    // One placed copy of a representation group and the tree node it belongs
    // to (-1 = top level, for files without any product tree).
    struct Placed {
        int group = 0;
        Mat43 xf;
        int owner = -1;
    };

    const EntityDB& db;
    const Builder& b;

    std::vector<Nauo> nauos;
    std::vector<Cdsr> cdsrs;
    std::unordered_map<int, std::vector<int>> pdToReps;      // PD -> rep ids
    std::unordered_map<int, RepMapInfo> repMaps;             // REPRESENTATION_MAP
    std::unordered_map<int, MappedItemInfo> mappedItems;     // MAPPED_ITEM
    std::unordered_map<int, std::vector<int>> srrAdj;        // plain SRR links
    std::vector<int> repEntities;                            // representation-typed ids
    bool haveMapped = false;
    int warnings = 0;

    // SRR connected components
    std::unordered_map<int, int> comp;                       // rep -> representative
    std::unordered_map<int, std::vector<int>> compMembers;   // representative -> members
    std::unordered_map<int, RepGroup> groupCache;

    std::unordered_set<int> mappedSourceGroups;  // groups used as map sources
    std::unordered_set<int> childRepGroups;      // groups on the child side of a RRWT
    std::unordered_map<int, OccXf> occCache;     // NAUO id -> resolved transform
    std::vector<Placed> placedList;              // placement order = instance order
    std::vector<TreeNode> tnodes;                // occurrence tree (placement order)
    std::unordered_map<int, int> groupToPD;      // rep group -> owning product def.

    static bool hasTypePrefix(const PEntity& e, const char* prefix) {
        size_t n = std::strlen(prefix);
        for (const PRec& r : e.recs)
            if (r.name.compare(0, n, prefix) == 0) return true;
        return false;
    }

    static bool looksLikeRep(const PEntity& e) {
        static const char* kNames[] = {
            "SHAPE_REPRESENTATION", "ADVANCED_BREP_SHAPE_REPRESENTATION",
            "MANIFOLD_SURFACE_SHAPE_REPRESENTATION", "FACETED_BREP_SHAPE_REPRESENTATION",
            "GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION",
            "GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION",
            "EDGE_BASED_WIREFRAME_SHAPE_REPRESENTATION", "REPRESENTATION"};
        for (const char* nm : kNames)
            if (Builder::hasType(e, nm)) return true;
        return false;
    }

    bool isAxisLike(int id) const {
        const PEntity* e = b.get(id);
        if (!e) return false;
        return Builder::hasType(*e, "AXIS2_PLACEMENT_3D") ||
               Builder::hasType(*e, "AXIS2_PLACEMENT_2D") ||
               Builder::hasType(*e, "CARTESIAN_TRANSFORMATION_OPERATOR_3D") ||
               Builder::hasType(*e, "CARTESIAN_TRANSFORMATION_OPERATOR");
    }

    bool isRepMap(int id) const {
        const PEntity* e = b.get(id);
        return e && Builder::hasType(*e, "REPRESENTATION_MAP");
    }

    int pdsDefinition(int pdsId) const {
        const PEntity* e = b.get(pdsId);
        if (!e) return 0;
        const Args* a = Builder::recArgs(*e, "PRODUCT_DEFINITION_SHAPE");
        if (!a) a = Builder::recArgs(*e, "PROPERTY_DEFINITION");
        if (!a) return 0;
        std::vector<int> r = refsOf(*a);
        return r.empty() ? 0 : r[0];
    }

    // ---- scan pass ------------------------------------------------------

    void scan() {
        std::vector<int> ids;
        ids.reserve(db.size());
        for (const auto& kv : db) ids.push_back(kv.first);
        std::sort(ids.begin(), ids.end()); // deterministic order
        for (int id : ids) {
            const PEntity& e = db.at(id);
            if (const Args* a = Builder::recArgs(e, "NEXT_ASSEMBLY_USAGE_OCCURRENCE")) {
                std::vector<int> r = refsOf(*a);
                if (r.size() >= 2) {
                    Nauo nn;
                    nn.id = id;
                    nn.parent = r[0];
                    nn.child = r[1];
                    // (id, name, description, relating, related, ...)
                    nn.name = cleanName(strArg(*a, 1));
                    nauos.push_back(std::move(nn));
                }
            }
            if (const Args* a = Builder::recArgs(e, "SHAPE_DEFINITION_REPRESENTATION")) {
                std::vector<int> r = refsOf(*a);
                if (r.size() >= 2) {
                    int def = pdsDefinition(r[0]);
                    const PEntity* de = (def > 0) ? b.get(def) : nullptr;
                    if (de && hasTypePrefix(*de, "PRODUCT_DEFINITION") &&
                        !Builder::hasType(*de, "NEXT_ASSEMBLY_USAGE_OCCURRENCE")) {
                        std::vector<int>& reps = pdToReps[def];
                        if (std::find(reps.begin(), reps.end(), r[1]) == reps.end())
                            reps.push_back(r[1]);
                    }
                }
            }
            if (const Args* a = Builder::recArgs(e, "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")) {
                std::vector<int> r = refsOf(*a);
                if (r.size() >= 2) cdsrs.push_back({r[0], r[1]});
            }
            if (const Args* a = Builder::recArgs(e, "MAPPED_ITEM")) {
                haveMapped = true;
                std::vector<int> r = refsOf(*a);
                MappedItemInfo mi;
                if (r.size() >= 1) mi.mapId = r[0];
                if (r.size() >= 2) mi.target = r[1];
                if (mi.mapId && !isRepMap(mi.mapId) && mi.target && isRepMap(mi.target))
                    std::swap(mi.mapId, mi.target);
                mappedItems[id] = mi;
            }
            if (const Args* a = Builder::recArgs(e, "REPRESENTATION_MAP")) {
                std::vector<int> r = refsOf(*a);
                if (r.size() >= 2) {
                    RepMapInfo rm{r[0], r[1]};
                    if (!isAxisLike(rm.origin) && isAxisLike(rm.mappedRep))
                        std::swap(rm.origin, rm.mappedRep);
                    repMaps[id] = rm;
                }
            }
            // plain representation relationship (no transformation) merges
            // two representations of the same shape
            bool hasRR = Builder::hasType(e, "REPRESENTATION_RELATIONSHIP") ||
                         Builder::hasType(e, "SHAPE_REPRESENTATION_RELATIONSHIP");
            bool hasWT =
                Builder::hasType(e, "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") ||
                Builder::hasType(e, "SHAPE_REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION");
            if (hasRR && !hasWT &&
                !Builder::hasType(e, "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")) {
                const Args* a = Builder::recArgs(e, "REPRESENTATION_RELATIONSHIP");
                if (!a) a = Builder::recArgs(e, "SHAPE_REPRESENTATION_RELATIONSHIP");
                std::vector<int> r = refsOf(*a);
                if (r.size() >= 2 && r[0] != r[1]) {
                    srrAdj[r[0]].push_back(r[1]);
                    srrAdj[r[1]].push_back(r[0]);
                }
            }
            if (looksLikeRep(e)) repEntities.push_back(id);
        }
        buildComponents();
        for (const auto& kv : repMaps) mappedSourceGroups.insert(groupOf(kv.second.mappedRep));
        // rep group -> owning product definition (lowest PD id wins, so the
        // mapping is deterministic); used to name geometry leaves
        std::vector<int> pdKeys;
        pdKeys.reserve(pdToReps.size());
        for (const auto& kv : pdToReps) pdKeys.push_back(kv.first);
        std::sort(pdKeys.begin(), pdKeys.end());
        for (int pd : pdKeys)
            for (int rep : pdToReps.at(pd)) groupToPD.emplace(groupOf(rep), pd);
    }

    void buildComponents() {
        std::vector<int> nodes;
        for (const auto& kv : srrAdj) nodes.push_back(kv.first);
        std::sort(nodes.begin(), nodes.end());
        for (int start : nodes) {
            if (comp.count(start)) continue;
            std::vector<int> stack{start}, memberList;
            std::unordered_set<int> seen{start};
            while (!stack.empty()) {
                int cur = stack.back();
                stack.pop_back();
                memberList.push_back(cur);
                auto it = srrAdj.find(cur);
                if (it == srrAdj.end()) continue;
                for (int nb : it->second)
                    if (seen.insert(nb).second) stack.push_back(nb);
            }
            std::sort(memberList.begin(), memberList.end());
            int repMin = memberList.front();
            for (int m : memberList) comp[m] = repMin;
            compMembers[repMin] = std::move(memberList);
        }
    }

    int groupOf(int rep) const {
        auto it = comp.find(rep);
        return (it == comp.end()) ? rep : it->second;
    }

    // ---- geometry of a representation group ------------------------------

    void collectShellFaces(const PEntity& e, std::vector<int>& out,
                           std::unordered_set<int>& seen) const {
        const Args* a = Builder::recArgs(e, "CLOSED_SHELL");
        if (!a) a = Builder::recArgs(e, "OPEN_SHELL");
        if (!a) return;
        for (const PValue& v : *a) {
            if (!isList(v)) continue;
            for (const PValue& f : v.list)
                if (f.k == PValue::KRef && seen.insert(f.ref).second) out.push_back(f.ref);
        }
    }

    void resolveShellRef(int id, std::vector<int>& out, std::unordered_set<int>& seen) const {
        const PEntity* e = b.get(id);
        if (!e) return;
        collectShellFaces(*e, out, seen);
        const Args* oa = Builder::recArgs(*e, "ORIENTED_CLOSED_SHELL");
        if (!oa) oa = Builder::recArgs(*e, "ORIENTED_OPEN_SHELL");
        if (oa) {
            for (const PValue& v : *oa) {
                if (v.k != PValue::KRef) continue;
                if (const PEntity* se = b.get(v.ref)) collectShellFaces(*se, out, seen);
            }
        }
    }

    void collectItemFaces(int id, std::vector<int>& out, std::unordered_set<int>& seen) const {
        const PEntity* e = b.get(id);
        if (!e) return;
        if (Builder::hasType(*e, "ADVANCED_FACE") || Builder::hasType(*e, "FACE_SURFACE")) {
            if (seen.insert(id).second) out.push_back(id);
            return;
        }
        if (Builder::hasType(*e, "CLOSED_SHELL") || Builder::hasType(*e, "OPEN_SHELL") ||
            Builder::hasType(*e, "ORIENTED_CLOSED_SHELL") ||
            Builder::hasType(*e, "ORIENTED_OPEN_SHELL")) {
            resolveShellRef(id, out, seen);
            return;
        }
        // solids / shell models: every reference (either direct or inside a
        // list) may be a shell
        static const char* kOwners[] = {"MANIFOLD_SOLID_BREP", "BREP_WITH_VOIDS",
                                        "FACETED_BREP", "SHELL_BASED_SURFACE_MODEL"};
        for (const char* nm : kOwners) {
            const Args* a = Builder::recArgs(*e, nm);
            if (!a) continue;
            for (const PValue& v : *a) {
                if (v.k == PValue::KRef) {
                    resolveShellRef(v.ref, out, seen);
                } else if (isList(v)) {
                    for (const PValue& s : v.list)
                        if (s.k == PValue::KRef) resolveShellRef(s.ref, out, seen);
                }
            }
        }
    }

    const RepGroup& groupData(int rep) {
        int g = groupOf(rep);
        auto it = groupCache.find(g);
        if (it != groupCache.end()) return it->second;
        RepGroup grp;
        std::unordered_set<int> seen;
        std::vector<int> single{g};
        auto mit = compMembers.find(g);
        const std::vector<int>& memberList = (mit != compMembers.end()) ? mit->second : single;
        for (int member : memberList) {
            const PEntity* e = b.get(member);
            if (!e) continue;
            const PValue* items = nullptr;
            for (const PRec& r : e->recs) {
                for (const PValue& v : r.args)
                    if (isList(v)) { items = &v; break; }
                if (items) break;
            }
            if (!items) continue;
            for (const PValue& v : items->list) {
                if (v.k != PValue::KRef) continue;
                const PEntity* ie = b.get(v.ref);
                if (!ie) continue;
                if (Builder::hasType(*ie, "MAPPED_ITEM")) {
                    grp.mappedItems.push_back(v.ref);
                    continue;
                }
                size_t beforeFaces = grp.faceIds.size();
                collectItemFaces(v.ref, grp.faceIds, seen);
                // remember the name of the first item that contributed faces
                if (grp.itemName.empty() && grp.faceIds.size() > beforeFaces &&
                    !ie->recs.empty())
                    grp.itemName = cleanName(strArg(ie->recs[0].args, 0));
            }
        }
        return groupCache.emplace(g, std::move(grp)).first->second;
    }

    // ---- transforms ------------------------------------------------------

    // AXIS2_PLACEMENT_3D or CARTESIAN_TRANSFORMATION_OPERATOR(_3D) -> matrix.
    bool placementMat(int id, Mat43& out) const {
        step::Frame f;
        if (b.getAxis2(id, f)) {
            out = step::xfFromFrame(f);
            return true;
        }
        const PEntity* e = b.get(id);
        if (!e) return false;
        const Args* cto3 = Builder::recArgs(*e, "CARTESIAN_TRANSFORMATION_OPERATOR_3D");
        const Args* cto = Builder::recArgs(*e, "CARTESIAN_TRANSFORMATION_OPERATOR");
        if (!cto3 && !cto) return false;
        // attribute order: (name, axis1[x], axis2[y], local_origin, scale)
        // + axis3[z] from the 3D subtype
        Vec3d dirs[3];
        bool haveDir[3] = {false, false, false};
        int ndir = 0;
        Vec3d o{0, 0, 0};
        double scale = 1.0;
        auto scanRec = [&](const Args* a) {
            if (!a) return;
            for (const PValue& v : *a) {
                if (v.k == PValue::KRef) {
                    Vec3d tmp;
                    if (b.getDirection(v.ref, tmp)) {
                        if (ndir < 3) {
                            dirs[ndir] = tmp;
                            haveDir[ndir] = true;
                            ++ndir;
                        }
                    } else {
                        Vec3d p;
                        if (b.getPoint(v.ref, p)) o = p;
                    }
                } else if (v.k == PValue::KNum) {
                    scale = v.num;
                }
            }
        };
        if (cto != cto3) scanRec(cto);
        scanRec(cto3);
        step::Frame fr = Builder::frameFromZX(o, dirs[2], dirs[0], haveDir[2], haveDir[0]);
        out = step::xfFromFrame(fr);
        if (scale > 0.0 && scale != 1.0) {
            out.cx = out.cx * scale;
            out.cy = out.cy * scale;
            out.cz = out.cz * scale;
        }
        return true;
    }

    bool repBelongsTo(int rep, int pd) {
        auto it = pdToReps.find(pd);
        if (it == pdToReps.end()) return false;
        int g = groupOf(rep);
        for (int r : it->second)
            if (groupOf(r) == g) return true;
        return false;
    }

    // (SHAPE_)REPRESENTATION_RELATIONSHIP(_WITH_TRANSFORMATION) -> reps + op.
    static void parseRelWithTransform(const PEntity& e, int& rep1, int& rep2, int& opId) {
        rep1 = rep2 = opId = 0;
        for (const PRec& r : e.recs) {
            std::vector<int> refs = refsOf(r.args);
            if (r.name == "REPRESENTATION_RELATIONSHIP" ||
                r.name == "SHAPE_REPRESENTATION_RELATIONSHIP") {
                if (refs.size() >= 2 && rep1 == 0) {
                    rep1 = refs[0];
                    rep2 = refs[1];
                }
            } else if (r.name == "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION" ||
                       r.name == "SHAPE_REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") {
                if (refs.size() >= 3) {
                    // flattened simple instance carries the full arg list
                    if (rep1 == 0) {
                        rep1 = refs[0];
                        rep2 = refs[1];
                    }
                    opId = refs[2];
                } else if (!refs.empty() && opId == 0) {
                    opId = refs[0];
                }
            }
        }
    }

    // Child -> parent placement of one NAUO (identity + warning on failure).
    OccXf occurrenceTransform(const Nauo& n) {
        auto cached = occCache.find(n.id);
        if (cached != occCache.end()) return cached->second;
        OccXf res;
        res.warned = true; // cleared on full success
        do {
            int rrId = 0;
            for (const Cdsr& c : cdsrs)
                if (pdsDefinition(c.pds) == n.id) { rrId = c.rr; break; }
            if (!rrId) break;
            const PEntity* e = b.get(rrId);
            if (!e) break;
            int rep1 = 0, rep2 = 0, opId = 0;
            parseRelWithTransform(*e, rep1, rep2, opId);
            if (!opId) break;
            // Which side is the child? Default: rep_1 (common convention,
            // see the block comment above).
            bool r1c = repBelongsTo(rep1, n.child);
            bool r2c = repBelongsTo(rep2, n.child);
            bool r1p = repBelongsTo(rep1, n.parent);
            bool r2p = repBelongsTo(rep2, n.parent);
            bool swapped = (r2c && !r1c) || (r1p && !r2p);
            childRepGroups.insert(groupOf(swapped ? rep2 : rep1));
            const PEntity* op = b.get(opId);
            if (!op) break;
            if (const Args* idt = Builder::recArgs(*op, "ITEM_DEFINED_TRANSFORMATION")) {
                std::vector<int> refs = refsOf(*idt);
                Mat43 m1, m2;
                if (refs.size() < 2 || !placementMat(refs[0], m1) ||
                    !placementMat(refs[1], m2))
                    break;
                bool okInv = true;
                res.xf = swapped ? step::xfCompose(m1, step::xfInverse(m2, okInv))
                                 : step::xfCompose(m2, step::xfInverse(m1, okInv));
                if (!okInv) {
                    res.xf = Mat43{};
                    break;
                }
                res.warned = false;
                break;
            }
            // operator given directly as a cartesian transformation operator
            Mat43 m;
            if (placementMat(opId, m)) {
                if (swapped) {
                    bool okInv = true;
                    Mat43 inv = step::xfInverse(m, okInv);
                    if (!okInv) break;
                    res.xf = inv;
                } else {
                    res.xf = m;
                }
                res.warned = false;
            }
        } while (false);
        if (res.warned) res.xf = Mat43{};
        occCache[n.id] = res;
        return res;
    }

    bool mappedTransform(int miId, Mat43& T, int& mappedRep) {
        T = Mat43{};
        mappedRep = 0;
        auto mi = mappedItems.find(miId);
        if (mi == mappedItems.end()) return false;
        auto rm = repMaps.find(mi->second.mapId);
        if (rm == repMaps.end()) return false;
        mappedRep = rm->second.mappedRep;
        Mat43 mo, mt;
        bool okO = placementMat(rm->second.origin, mo);
        bool okT = placementMat(mi->second.target, mt);
        bool okInv = true;
        T = step::xfCompose(okT ? mt : Mat43{}, step::xfInverse(okO ? mo : Mat43{}, okInv));
        return okO && okT && okInv;
    }

    // ---- names -----------------------------------------------------------

    // PRODUCT name (or id) of a product definition, via
    // PRODUCT_DEFINITION -> PRODUCT_DEFINITION_FORMATION -> PRODUCT.
    std::string productNameOfPD(int pd) const {
        const PEntity* e = b.get(pd);
        if (!e) return std::string();
        const Args* a = Builder::recArgs(*e, "PRODUCT_DEFINITION");
        if (!a) {
            for (const PRec& r : e->recs) {
                if (r.name.compare(0, 18, "PRODUCT_DEFINITION") == 0 &&
                    r.name.find("SHAPE") == std::string::npos &&
                    r.name.find("FORMATION") == std::string::npos &&
                    r.name.find("RELATIONSHIP") == std::string::npos) {
                    a = &r.args;
                    break;
                }
            }
        }
        if (!a) return std::string();
        for (int fid : refsOf(*a)) { // formation is normally the first ref
            const PEntity* fe = b.get(fid);
            if (!fe) continue;
            const Args* fa = nullptr;
            for (const PRec& r : fe->recs)
                if (r.name.compare(0, 28, "PRODUCT_DEFINITION_FORMATION") == 0) {
                    fa = &r.args;
                    break;
                }
            if (!fa) continue;
            for (int pid : refsOf(*fa)) {
                const PEntity* pe = b.get(pid);
                if (!pe) continue;
                const Args* pa = Builder::recArgs(*pe, "PRODUCT");
                if (!pa) continue;
                // PRODUCT(id, name, description, frame_of_reference)
                std::string nm = cleanName(strArg(*pa, 1));
                if (nm.empty()) nm = cleanName(strArg(*pa, 0));
                return nm;
            }
        }
        return std::string();
    }

    // Display name for a geometry leaf: product owning the group, then the
    // name of its solid item, then a positional fallback.
    std::string leafName(int group, bool nested, int seq) {
        auto it = groupToPD.find(group);
        if (it != groupToPD.end()) {
            std::string nm = productNameOfPD(it->second);
            if (!nm.empty()) return nm;
        }
        const RepGroup& G = groupData(group);
        if (!G.itemName.empty()) return G.itemName;
        return (nested ? "Body " : "Part ") + std::to_string(seq);
    }

    // ---- placement recursion --------------------------------------------

    void placeRep(int rep, const Mat43& xf, int depth, std::vector<int>& stack,
                  int owner) {
        int g = groupOf(rep);
        if (depth > 64 ||
            std::find(stack.begin(), stack.end(), g) != stack.end()) {
            ++warnings; // cycle / runaway depth in the mapping graph
            return;
        }
        const RepGroup& G = groupData(g);
        if (!G.faceIds.empty()) placedList.push_back({g, xf, owner});
        if (G.mappedItems.empty()) return;
        stack.push_back(g);
        for (int mi : G.mappedItems) {
            Mat43 T;
            int mrep = 0;
            if (!mappedTransform(mi, T, mrep)) ++warnings;
            if (mrep) placeRep(mrep, step::xfCompose(xf, T), depth + 1, stack, owner);
        }
        stack.pop_back();
    }

    void placePD(int pd, const Mat43& xf, int depth, std::vector<int>& pdStack,
                 std::vector<int>& repStack, int parentNode, const std::string& name) {
        if (depth > 64 ||
            std::find(pdStack.begin(), pdStack.end(), pd) != pdStack.end()) {
            ++warnings; // cycle in the product graph
            return;
        }
        int nodeIdx = static_cast<int>(tnodes.size());
        tnodes.push_back({name, parentNode, false});
        if (parentNode >= 0) tnodes[static_cast<size_t>(parentNode)].hasChildren = true;
        pdStack.push_back(pd);
        auto it = pdToReps.find(pd);
        if (it != pdToReps.end()) {
            std::unordered_set<int> done;
            for (int rep : it->second) {
                int g = groupOf(rep);
                if (!done.insert(g).second) continue;
                if (mappedSourceGroups.count(g)) continue; // placed via MAPPED_ITEM
                placeRep(rep, xf, depth, repStack, nodeIdx);
            }
        }
        int childSeq = 0;
        for (const Nauo& n : nauos) {
            if (n.parent != pd) continue;
            ++childSeq;
            OccXf o = occurrenceTransform(n);
            if (o.warned) ++warnings;
            std::string cname = n.name;
            if (cname.empty()) cname = productNameOfPD(n.child);
            if (cname.empty()) cname = "Occurrence #" + std::to_string(childSeq);
            placePD(n.child, step::xfCompose(xf, o.xf), depth + 1, pdStack, repStack,
                    nodeIdx, cname);
        }
        pdStack.pop_back();
    }

    // ---- component tree -> Model::nodes ---------------------------------

    // Convert the occurrence tree plus the placement list into Model::nodes.
    // A placement is attached directly to its occurrence node when it is the
    // only geometry of a childless node; otherwise (multi-body products,
    // products that both carry geometry and contain sub-occurrences, files
    // without a product tree) a distinct leaf node is appended, so leaves
    // always map 1:1 to placed instances. Appending keeps every parent index
    // smaller than its children's indices.
    void buildNodes(step::Model& model, const std::vector<int>& plPart,
                    const std::vector<int>& plInst) {
        model.nodes.clear();
        model.nodes.reserve(tnodes.size() + placedList.size());
        for (const TreeNode& t : tnodes) {
            step::ModelNode nd;
            nd.name = t.name;
            nd.parent = t.parent;
            model.nodes.push_back(std::move(nd));
        }
        std::vector<int> ownerCount(tnodes.size(), 0);
        for (const Placed& pl : placedList)
            if (pl.owner >= 0) ++ownerCount[static_cast<size_t>(pl.owner)];
        std::vector<int> ownerSeq(tnodes.size(), 0);
        int topSeq = 0;
        for (size_t pi = 0; pi < placedList.size(); ++pi) {
            const Placed& pl = placedList[pi];
            if (pl.owner >= 0 && ownerCount[static_cast<size_t>(pl.owner)] == 1 &&
                !tnodes[static_cast<size_t>(pl.owner)].hasChildren) {
                model.nodes[static_cast<size_t>(pl.owner)].part = plPart[pi];
                model.nodes[static_cast<size_t>(pl.owner)].instance = plInst[pi];
                continue;
            }
            step::ModelNode leaf;
            leaf.parent = pl.owner;
            leaf.part = plPart[pi];
            leaf.instance = plInst[pi];
            int seq = (pl.owner >= 0) ? ++ownerSeq[static_cast<size_t>(pl.owner)]
                                      : ++topSeq;
            leaf.name = leafName(pl.group, pl.owner >= 0, seq);
            model.nodes.push_back(std::move(leaf));
        }
    }

    void resolveOccurrences() {
        // resolve every occurrence upfront so childRepGroups is complete
        // before root detection (and so warnings are counted once)
        for (const Nauo& n : nauos) occurrenceTransform(n);
    }
};

} // namespace

// ---------------------------------------------------------------------------
// Public module functions
// ---------------------------------------------------------------------------

namespace step {

bool parseStep(const std::string& path, Model& model, std::string& err) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        err = "STEP: cannot open file: " + path;
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    std::string text = ss.str();
    if (text.empty()) {
        err = "STEP: file is empty: " + path;
        return false;
    }
    text = stripComments(text);

    // Isolate the DATA section; be lenient if the framing is missing.
    std::string data;
    size_t dpos = text.find("DATA;");
    if (dpos == std::string::npos) dpos = text.find("DATA ");
    if (dpos != std::string::npos) {
        size_t start = text.find(';', dpos);
        start = (start == std::string::npos) ? dpos + 4 : start + 1;
        size_t end = text.find("ENDSEC;", start);
        if (end == std::string::npos) end = text.size();
        data = text.substr(start, end - start);
    } else {
        data = text;
    }

    EntityDB db;
    P21Parser parser(data);
    parser.parseInstances(db);
    if (db.empty()) {
        err = "STEP: no Part 21 instances found (not a STEP file?)";
        return false;
    }

    Builder b(db);

    // Assembly structure first: if the file has NAUO/MAPPED_ITEM instancing,
    // place only the instanced geometry (a shell reachable both directly and
    // through the assembly must not be duplicated). The flat path below runs
    // when there is no assembly structure or when it yields no geometry.
    AsmResolver resolver(db, b);
    bool assembled = false;
    if (resolver.active()) {
        assembled = resolver.buildModel(model);
        if (!assembled) {
            // broken assembly structure: reset and fall back
            model.parts.clear();
            model.totalFaces = 0;
            model.skippedFaces = 0;
            ++model.assemblyWarnings;
        }
    }
    model.assemblyWarnings += resolver.warningCount();

    if (!assembled) {
        model.nodes.clear();
        std::vector<ShellGroup> groups = collectShellGroups(db, b);
        size_t totalIds = 0;
        for (const ShellGroup& g : groups) totalIds += g.faceIds.size();
        model.totalFaces = static_cast<int>(totalIds);
        if (groups.empty()) {
            err = "STEP: no B-rep faces found in file";
            return false;
        }
        // With several independent shells each becomes a part with its own
        // top-level node, so simple multi-body files stay navigable; a single
        // shell keeps the classic anonymous single-object layout.
        const bool multi = groups.size() > 1;
        for (size_t gi = 0; gi < groups.size(); ++gi) {
            Part part;
            for (int id : groups[gi].faceIds) {
                Face f;
                bool ok = false;
                try {
                    ok = b.buildFace(id, f);
                } catch (...) {
                    ok = false;
                }
                if (ok) part.faces.push_back(std::move(f));
                else ++model.skippedFaces;
            }
            if (part.faces.empty()) continue;
            part.instances.push_back(Mat43{}); // identity: geometry stays local
            int pidx = static_cast<int>(model.parts.size());
            model.parts.push_back(std::move(part));
            if (multi) {
                ModelNode nd;
                nd.name = groups[gi].name.empty()
                              ? "Part " + std::to_string(gi + 1)
                              : groups[gi].name;
                nd.parent = -1;
                nd.part = pidx;
                nd.instance = 0;
                model.nodes.push_back(std::move(nd));
            }
        }
    }

    size_t builtFaces = 0;
    for (const Part& p : model.parts) builtFaces += p.faces.size();
    if (builtFaces == 0) {
        err = "STEP: all " + std::to_string(model.totalFaces) +
              " faces use unsupported geometry";
        return false;
    }
    return true;
}

} // namespace step

// ---------------------------------------------------------------------------
// Viewer entry point
// ---------------------------------------------------------------------------

bool loadSTEP(const std::string& path, Mesh& out, std::string& err) {
    step::Model model;
    if (!step::parseStep(path, model, err)) return false;

    int ok = 0, approx = 0, failed = 0;
    if (!step::tessellate(model, out, err, &ok, &approx, &failed)) return false;

    if (out.positions.empty()) {
        err = "STEP: tessellation produced no triangles (" +
              std::to_string(model.skippedFaces + failed) + " of " +
              std::to_string(model.totalFaces) + " faces unsupported/failed)";
        return false;
    }
    out.computeBounds();
    return true;
}
