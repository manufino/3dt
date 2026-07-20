/*
 * renderer.cpp
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

// Renderer implementation: OpenGL 3.3 core (shaders, MSAA, shadow mapping)
// with a legacy OpenGL 1.1 fixed-function fallback.
//
// Design notes:
// - Context creation is delegated to the platform layer (WGL on Windows,
//   GLX on Linux): platform::createGLContext() tries MSAA 8x -> 4x -> none
//   and a 3.3 core profile, with a legacy-context fallback.
// - Gamma correction is done manually in the lit fragment shader
//   (pow(1/2.2)); lighting math runs in linear space. Background/lines are
//   authored directly in display space.
// - Flat shading mode recomputes the face normal in the fragment shader with
//   dFdx/dFdy (no extra vertex data, works with any mesh).
// - The ground "shadow catcher" plane is visible only while shadows are
//   enabled (and not in wireframe): it is fully transparent where lit and
//   darkens only where the shadow falls, so it never hides the grid.

#include "renderer.h"
#include "gl_loader.h"
#include "mesh.h"
#include "platform.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#ifndef GL_CLIP_DISTANCE0
#define GL_CLIP_DISTANCE0 0x3000
#endif

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg2Rad = kPi / 180.f;
constexpr float kFovYDeg = 45.f;

// Key light direction (towards the light, world space, Z-up). Must match
// kKeyDir in the lit/ground fragment shaders.
constexpr float kKeyLightDir[3] = {0.45f, 0.30f, 0.80f};

// Model tree panel metrics (pixels). Shared by layout, drawing and
// hit-testing so the three always agree.
constexpr float kPanelMargin = 12.f;   // distance from the window edges
constexpr float kPanelPad    = 10.f;   // inner padding of overlay panels
constexpr float kTreeArrowW  = 14.f;   // expand/collapse arrow column
constexpr float kTreeCheckW  = 16.f;   // checkbox column (box + gap)

// --- small vector/matrix helpers (column-major, OpenGL convention) ----------

inline void normalize3(float v[3]) {
    float len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (len > 1e-20f) {
        v[0] /= len; v[1] /= len; v[2] /= len;
    }
}

inline void cross3(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

inline float dot3(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

struct Mat4 {
    float m[16]; // column-major
};

Mat4 matMul(const Mat4& a, const Mat4& b) { // a * b
    Mat4 r;
    for (int c = 0; c < 4; ++c) {
        for (int row = 0; row < 4; ++row) {
            float s = 0.f;
            for (int k = 0; k < 4; ++k) s += a.m[k * 4 + row] * b.m[c * 4 + k];
            r.m[c * 4 + row] = s;
        }
    }
    return r;
}

Mat4 matPerspective(float fovYDeg, float aspect, float znear, float zfar) {
    Mat4 r{};
    float f = 1.f / std::tan(fovYDeg * 0.5f * kDeg2Rad);
    r.m[0] = f / aspect;
    r.m[5] = f;
    r.m[10] = (zfar + znear) / (znear - zfar);
    r.m[11] = -1.f;
    r.m[14] = (2.f * zfar * znear) / (znear - zfar);
    return r;
}

Mat4 matOrtho(float l, float rgt, float b, float t, float n, float f) {
    Mat4 r{};
    r.m[0] = 2.f / (rgt - l);
    r.m[5] = 2.f / (t - b);
    r.m[10] = -2.f / (f - n);
    r.m[12] = -(rgt + l) / (rgt - l);
    r.m[13] = -(t + b) / (t - b);
    r.m[14] = -(f + n) / (f - n);
    r.m[15] = 1.f;
    return r;
}

Mat4 matLookAt(const float eye[3], const float target[3], const float upHint[3]) {
    float fwd[3] = {target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]};
    normalize3(fwd);
    float right[3];
    cross3(fwd, upHint, right);
    normalize3(right);
    float up[3];
    cross3(right, fwd, up);
    Mat4 r{};
    r.m[0] = right[0]; r.m[4] = right[1]; r.m[8]  = right[2];
    r.m[1] = up[0];    r.m[5] = up[1];    r.m[9]  = up[2];
    r.m[2] = -fwd[0];  r.m[6] = -fwd[1];  r.m[10] = -fwd[2];
    r.m[12] = -dot3(right, eye);
    r.m[13] = -dot3(up, eye);
    r.m[14] = dot3(fwd, eye);
    r.m[15] = 1.f;
    return r;
}

// Unit vector from target towards the eye, plus camera right/up vectors,
// for a Z-up orbital camera.
void cameraBasis(float yawDeg, float pitchDeg,
                 float toEye[3], float right[3], float up[3]) {
    float yr = yawDeg * kDeg2Rad;
    float pr = pitchDeg * kDeg2Rad;
    float cp = std::cos(pr), sp = std::sin(pr);
    toEye[0] = cp * std::cos(yr);
    toEye[1] = cp * std::sin(yr);
    toEye[2] = sp;
    float fwd[3] = {-toEye[0], -toEye[1], -toEye[2]};
    const float worldUp[3] = {0.f, 0.f, 1.f};
    cross3(fwd, worldUp, right);
    normalize3(right);
    cross3(right, fwd, up);
    normalize3(up);
}

// Grid layout shared by both paths: step snapped to the power of 10 nearest
// to diagonal/10, centered under the model, n lines each side.
struct GridLayout {
    float step, half, cx, cy;
    int n;
};

GridLayout gridLayout(float diag, float centerX, float centerY) {
    GridLayout g;
    g.step = std::pow(10.f, std::round(std::log10(diag / 10.f)));
    if (!(g.step > 0.f) || !std::isfinite(g.step)) g.step = 1.f;
    g.n = 10;
    g.half = g.step * static_cast<float>(g.n);
    g.cx = g.step * std::round(centerX / g.step);
    g.cy = g.step * std::round(centerY / g.step);
    return g;
}

// --- overlay text helpers ----------------------------------------------------

// 1234567 -> "1.234.567"
std::string fmtGrouped(unsigned long long v) {
    std::string s = std::to_string(v);
    for (int i = static_cast<int>(s.size()) - 3; i > 0; i -= 3)
        s.insert(static_cast<size_t>(i), ".");
    return s;
}

std::string fmtFloat(float v) {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%.4g", static_cast<double>(v));
    return buf;
}

// --- shader sources (GLSL 330 core) ------------------------------------------

const char* kLitVS = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 uViewProj;
uniform mat4 uLightVP;
uniform vec4 uClip[3];   // world-space section planes (X/Y/Z)
out vec3 vPos;
out vec3 vNormal;
out vec4 vShadow;
out float gl_ClipDistance[3];
void main() {
    vPos = aPos;
    vNormal = aNormal;
    vShadow = uLightVP * vec4(aPos, 1.0);
    for (int i = 0; i < 3; ++i)
        gl_ClipDistance[i] = dot(vec4(aPos, 1.0), uClip[i]);
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)GLSL";

const char* kLitFS = R"GLSL(
#version 330 core
in vec3 vPos;
in vec3 vNormal;
in vec4 vShadow;
out vec4 fragColor;
uniform vec3 uCamPos;
uniform bool uFlat;
uniform bool uShadows;
uniform sampler2DShadow uShadowMap;
uniform float uShadowTexel;

const vec3 kKeyDir    = normalize(vec3(0.45, 0.30, 0.80)); // towards key light
const vec3 kFillDir   = normalize(vec3(-0.50, -0.35, 0.25));
const vec3 kKeyColor  = vec3(0.95, 0.95, 0.95);
const vec3 kFillColor = vec3(0.26, 0.28, 0.33);
const vec3 kAmbient   = vec3(0.15, 0.16, 0.18);
const vec3 kAlbedo    = vec3(0.55, 0.62, 0.72);            // grey-blue
const vec3 kSpec      = vec3(0.30, 0.30, 0.30);
const float kShininess = 48.0;

// 3x3 PCF over a compare-mode depth texture (each tap is itself bilinearly
// filtered by hardware). Constant depth bias only: the slope-scaled part is
// applied with glPolygonOffset in the depth pass. Samples outside the map
// count as lit.
float shadowFactor() {
    vec3 p = vShadow.xyz / vShadow.w;
    p = p * 0.5 + 0.5;
    if (p.x < 0.0 || p.x > 1.0 || p.y < 0.0 || p.y > 1.0 || p.z > 1.0)
        return 1.0;
    p.z -= 0.0015;
    float s = 0.0;
    for (int y = -1; y <= 1; ++y)
        for (int x = -1; x <= 1; ++x)
            s += texture(uShadowMap,
                         vec3(p.xy + vec2(float(x), float(y)) * uShadowTexel, p.z));
    return s / 9.0;
}

void main() {
    vec3 V = normalize(uCamPos - vPos);
    vec3 n;
    if (uFlat) {
        // Face normal from screen-space derivatives, oriented to the viewer.
        n = normalize(cross(dFdx(vPos), dFdy(vPos)));
        if (dot(n, V) < 0.0) n = -n;
    } else {
        n = normalize(vNormal);
        if (!gl_FrontFacing) n = -n;   // two-sided shading
        if (dot(n, V) < 0.0) n = -n;   // guard for mixed-winding meshes
    }

    float sh = uShadows ? shadowFactor() : 1.0;

    vec3 c = kAmbient * kAlbedo;
    // Key light: Blinn-Phong, shadowed.
    float ndl = max(dot(n, kKeyDir), 0.0);
    if (ndl > 0.0) {
        vec3 h = normalize(kKeyDir + V);
        float sp = pow(max(dot(n, h), 0.0), kShininess);
        c += sh * kKeyColor * (kAlbedo * ndl + kSpec * sp);
    }
    // Fill light: diffuse only, unshadowed.
    c += kFillColor * kAlbedo * max(dot(n, kFillDir), 0.0);

    c = pow(c, vec3(1.0 / 2.2));       // linear -> display gamma
    fragColor = vec4(c, 1.0);
}
)GLSL";

const char* kDepthVS = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uLightVP;
uniform vec4 uClip[3];
out float gl_ClipDistance[3];
void main() {
    for (int i = 0; i < 3; ++i)
        gl_ClipDistance[i] = dot(vec4(aPos, 1.0), uClip[i]);
    gl_Position = uLightVP * vec4(aPos, 1.0);
}
)GLSL";

const char* kDepthFS = R"GLSL(
#version 330 core
void main() { }
)GLSL";

const char* kLineVS = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 uViewProj;
out vec3 vColor;
void main() {
    vColor = aColor;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)GLSL";

const char* kLineFS = R"GLSL(
#version 330 core
in vec3 vColor;
out vec4 fragColor;
void main() { fragColor = vec4(vColor, 1.0); }
)GLSL";

// Also used (with clipping disabled) for the section-plane highlight quad.
const char* kWireVS = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uViewProj;
uniform vec4 uClip[3];
out float gl_ClipDistance[3];
void main() {
    for (int i = 0; i < 3; ++i)
        gl_ClipDistance[i] = dot(vec4(aPos, 1.0), uClip[i]);
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)GLSL";

const char* kWireFS = R"GLSL(
#version 330 core
uniform vec3 uColor;
uniform float uAlpha;
out vec4 fragColor;
void main() { fragColor = vec4(uColor, uAlpha); }
)GLSL";

// Fullscreen triangle generated from gl_VertexID (no vertex buffer).
const char* kBgVS = R"GLSL(
#version 330 core
out vec2 vUV;
void main() {
    vec2 p = vec2((gl_VertexID == 1) ? 3.0 : -1.0,
                  (gl_VertexID == 2) ? 3.0 : -1.0);
    vUV = p * 0.5 + 0.5;
    gl_Position = vec4(p, 0.0, 1.0);
}
)GLSL";

const char* kBgFS = R"GLSL(
#version 330 core
in vec2 vUV;
out vec4 fragColor;
void main() {
    vec3 bottom = vec3(0.060, 0.066, 0.080);
    vec3 top    = vec3(0.165, 0.185, 0.230);
    vec3 c = mix(bottom, top, smoothstep(0.0, 1.0, vUV.y));
    float d = distance(vUV, vec2(0.5, 0.55));
    c *= 1.0 - 0.22 * smoothstep(0.45, 0.95, d);   // subtle vignette
    fragColor = vec4(c, 1.0);
}
)GLSL";

// Ground shadow catcher: transparent where lit, dark where shadowed, with a
// radial fade so it blends into the background gradient.
const char* kGroundVS = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aUV;   // -1..1 across the quad
uniform mat4 uViewProj;
uniform mat4 uLightVP;
out vec4 vShadow;
out vec2 vUV;
void main() {
    vShadow = uLightVP * vec4(aPos, 1.0);
    vUV = aUV;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)GLSL";

const char* kGroundFS = R"GLSL(
#version 330 core
in vec4 vShadow;
in vec2 vUV;
out vec4 fragColor;
uniform sampler2DShadow uShadowMap;
uniform float uShadowTexel;

float shadowFactor() {
    vec3 p = vShadow.xyz / vShadow.w;
    p = p * 0.5 + 0.5;
    if (p.x < 0.0 || p.x > 1.0 || p.y < 0.0 || p.y > 1.0 || p.z > 1.0)
        return 1.0;
    p.z -= 0.0015;
    float s = 0.0;
    for (int y = -1; y <= 1; ++y)
        for (int x = -1; x <= 1; ++x)
            s += texture(uShadowMap,
                         vec3(p.xy + vec2(float(x), float(y)) * uShadowTexel, p.z));
    return s / 9.0;
}

void main() {
    float sh = shadowFactor();
    float fade = 1.0 - smoothstep(0.35, 1.0, length(vUV));
    float a = (1.0 - sh) * 0.42 * fade;
    fragColor = vec4(vec3(0.020, 0.022, 0.028), a);
}
)GLSL";

// --- shader compilation ------------------------------------------------------

GLuint compileStage(GLenum type, const char* src, const char* name) {
    GLuint sh = glCreateShader(type);
    glShaderSource(sh, 1, &src, nullptr);
    glCompileShader(sh);
    GLint ok = GL_FALSE;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048] = "";
        GLsizei len = 0;
        glGetShaderInfoLog(sh, sizeof(log) - 1, &len, log);
        platform::debugLog("3dt: shader compile failed: ");
        platform::debugLog(name);
        platform::debugLog("\n");
        platform::debugLog(log);
        glDeleteShader(sh);
        return 0;
    }
    return sh;
}

GLuint linkProgram(const char* vsSrc, const char* fsSrc, const char* name) {
    GLuint vs = compileStage(GL_VERTEX_SHADER, vsSrc, name);
    if (!vs) return 0;
    GLuint fs = compileStage(GL_FRAGMENT_SHADER, fsSrc, name);
    if (!fs) {
        glDeleteShader(vs);
        return 0;
    }
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint ok = GL_FALSE;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[2048] = "";
        GLsizei len = 0;
        glGetProgramInfoLog(prog, sizeof(log) - 1, &len, log);
        platform::debugLog("3dt: program link failed: ");
        platform::debugLog(name);
        platform::debugLog("\n");
        platform::debugLog(log);
        glDeleteProgram(prog);
        return 0;
    }
    return prog;
}

} // namespace

// --- context creation --------------------------------------------------------

bool Renderer::init() {
    platform::GLContextInfo info;
    if (!platform::createGLContext(info)) return false;
    ctxValid_ = true;
    modern_ = info.modern;
    msaaSamples_ = info.msaaSamples;

    if (modern_ && !glLoaderInit(&platform::getGLProcAddress)) {
        platform::debugLog("3dt: GL 3.3 function loading failed\n");
        modern_ = false;
        if (!platform::recreateLegacyGLContext()) {
            ctxValid_ = false;
            return false;
        }
    }

    if (modern_ && !initModernResources()) {
        // Shaders/FBO failed on a supposedly capable context: fall back.
        // (The platform reuses the already-set pixel format if needed.)
        platform::debugLog("3dt: modern resources failed, using legacy\n");
        destroyModernResources();
        modern_ = false;
        if (!platform::recreateLegacyGLContext()) {
            ctxValid_ = false;
            return false;
        }
    }

    if (!modern_) {
        msaaSamples_ = 0;
        initLegacyState();
    }

    // VSync (harmless if unsupported).
    platform::setSwapInterval(1);

    return true;
}

// --- modern resources --------------------------------------------------------

bool Renderer::createPrograms() {
    progLit_    = linkProgram(kLitVS, kLitFS, "lit");
    progDepth_  = linkProgram(kDepthVS, kDepthFS, "depth");
    progLine_   = linkProgram(kLineVS, kLineFS, "line");
    progWire_   = linkProgram(kWireVS, kWireFS, "wire");
    progBg_     = linkProgram(kBgVS, kBgFS, "background");
    progGround_ = linkProgram(kGroundVS, kGroundFS, "ground");
    if (!progLit_ || !progDepth_ || !progLine_ || !progWire_ || !progBg_ ||
        !progGround_)
        return false;

    uLitViewProj_ = glGetUniformLocation(progLit_, "uViewProj");
    uLitLightVP_  = glGetUniformLocation(progLit_, "uLightVP");
    uLitCamPos_   = glGetUniformLocation(progLit_, "uCamPos");
    uLitFlat_     = glGetUniformLocation(progLit_, "uFlat");
    uLitShadows_  = glGetUniformLocation(progLit_, "uShadows");
    uLitTexel_    = glGetUniformLocation(progLit_, "uShadowTexel");
    uLitClip_     = glGetUniformLocation(progLit_, "uClip[0]");
    uDepthLightVP_ = glGetUniformLocation(progDepth_, "uLightVP");
    uDepthClip_    = glGetUniformLocation(progDepth_, "uClip[0]");
    uLineViewProj_ = glGetUniformLocation(progLine_, "uViewProj");
    uWireViewProj_ = glGetUniformLocation(progWire_, "uViewProj");
    uWireColor_    = glGetUniformLocation(progWire_, "uColor");
    uWireAlpha_    = glGetUniformLocation(progWire_, "uAlpha");
    uWireClip_     = glGetUniformLocation(progWire_, "uClip[0]");
    uGroundViewProj_ = glGetUniformLocation(progGround_, "uViewProj");
    uGroundLightVP_  = glGetUniformLocation(progGround_, "uLightVP");
    uGroundTexel_    = glGetUniformLocation(progGround_, "uShadowTexel");

    // The shadow map always lives on texture unit 0.
    glUseProgram(progLit_);
    glUniform1i(glGetUniformLocation(progLit_, "uShadowMap"), 0);
    glUseProgram(progGround_);
    glUniform1i(glGetUniformLocation(progGround_, "uShadowMap"), 0);
    glUseProgram(0);
    return true;
}

bool Renderer::createShadowTarget() {
    glGenTextures(1, &shadowTex_);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, shadowTex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24,
                 shadowSize_, shadowSize_, 0,
                 GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    const float border[4] = {1.f, 1.f, 1.f, 1.f}; // outside the map = lit
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE,
                    GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);

    glGenFramebuffers(1, &shadowFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_2D, shadowTex_, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    bool ok = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (!ok) platform::debugLog("3dt: shadow FBO incomplete\n");
    return ok;
}

bool Renderer::initModernResources() {
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);        // STEP meshes may have mixed winding
    if (msaaSamples_ > 0) glEnable(GL_MULTISAMPLE);

    if (!createPrograms()) return false;
    if (!createShadowTarget()) return false;

    // Mesh VAO: interleaved-block VBO (positions block, then normals block).
    glGenVertexArrays(1, &meshVao_);
    glGenBuffers(1, &meshVbo_);

    // Grid/axes VAO: interleaved position(3) + color(3).
    glGenVertexArrays(1, &lineVao_);
    glGenBuffers(1, &lineVbo_);
    glBindVertexArray(lineVao_);
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<const void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<const void*>(3 * sizeof(float)));

    // Ground VAO: position(3) + quad uv(2, -1..1).
    glGenVertexArrays(1, &groundVao_);
    glGenBuffers(1, &groundVbo_);
    glBindVertexArray(groundVao_);
    glBindBuffer(GL_ARRAY_BUFFER, groundVbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                          reinterpret_cast<const void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                          reinterpret_cast<const void*>(3 * sizeof(float)));

    // Background: attributeless fullscreen triangle still needs a VAO bound.
    glGenVertexArrays(1, &bgVao_);

    // Section-plane highlight quad: 4 vertices, position only, dynamic.
    glGenVertexArrays(1, &secVao_);
    glGenBuffers(1, &secVbo_);
    glBindVertexArray(secVao_);
    glBindBuffer(GL_ARRAY_BUFFER, secVbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          reinterpret_cast<const void*>(0));

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Text overlay (non-fatal if it fails: the 3D view still works).
    overlayOk_ = overlay_.init();
    if (!overlayOk_) platform::debugLog("3dt: overlay init failed\n");

    rebuildSceneBuffers();
    return true;
}

void Renderer::destroyModernResources() {
    if (meshVbo_)   { glDeleteBuffers(1, &meshVbo_); meshVbo_ = 0; }
    if (meshVao_)   { glDeleteVertexArrays(1, &meshVao_); meshVao_ = 0; }
    if (lineVbo_)   { glDeleteBuffers(1, &lineVbo_); lineVbo_ = 0; }
    if (lineVao_)   { glDeleteVertexArrays(1, &lineVao_); lineVao_ = 0; }
    if (groundVbo_) { glDeleteBuffers(1, &groundVbo_); groundVbo_ = 0; }
    if (groundVao_) { glDeleteVertexArrays(1, &groundVao_); groundVao_ = 0; }
    if (bgVao_)     { glDeleteVertexArrays(1, &bgVao_); bgVao_ = 0; }
    if (secVbo_)    { glDeleteBuffers(1, &secVbo_); secVbo_ = 0; }
    if (secVao_)    { glDeleteVertexArrays(1, &secVao_); secVao_ = 0; }
    overlay_.destroy();
    overlayOk_ = false;
    if (shadowFbo_) { glDeleteFramebuffers(1, &shadowFbo_); shadowFbo_ = 0; }
    if (shadowTex_) { glDeleteTextures(1, &shadowTex_); shadowTex_ = 0; }
    GLuint progs[6] = {progLit_, progDepth_, progLine_, progWire_, progBg_,
                       progGround_};
    for (GLuint p : progs) {
        if (p) glDeleteProgram(p);
    }
    progLit_ = progDepth_ = progLine_ = progWire_ = progBg_ = progGround_ = 0;
    meshVerts_ = lineVerts_ = 0;
}

// Rebuild grid/axes and ground VBOs for the current scene bounds.
void Renderer::rebuildSceneBuffers() {
    if (!modern_) return;

    GridLayout g = gridLayout(diag_, centerX_, centerY_);
    float z = groundZ_;

    std::vector<float> v;
    v.reserve(static_cast<size_t>((2 * g.n + 1) * 4 + 6) * 6);
    const float gridCol[3] = {0.28f, 0.30f, 0.33f};
    auto push = [&v](float x, float y, float zz, const float c[3]) {
        v.push_back(x); v.push_back(y); v.push_back(zz);
        v.push_back(c[0]); v.push_back(c[1]); v.push_back(c[2]);
    };
    for (int i = -g.n; i <= g.n; ++i) {
        float o = static_cast<float>(i) * g.step;
        push(g.cx - g.half, g.cy + o, z, gridCol);
        push(g.cx + g.half, g.cy + o, z, gridCol);
        push(g.cx + o, g.cy - g.half, z, gridCol);
        push(g.cx + o, g.cy + g.half, z, gridCol);
    }
    // Axes at the grid center: X red, Y green, Z blue.
    float alen = g.half * 0.5f;
    const float cr[3] = {0.85f, 0.25f, 0.25f};
    const float cg[3] = {0.25f, 0.75f, 0.30f};
    const float cb[3] = {0.30f, 0.45f, 0.90f};
    push(g.cx, g.cy, z, cr); push(g.cx + alen, g.cy, z, cr);
    push(g.cx, g.cy, z, cg); push(g.cx, g.cy + alen, z, cg);
    push(g.cx, g.cy, z, cb); push(g.cx, g.cy, z + alen, cb);

    lineVerts_ = static_cast<int>(v.size() / 6);
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(v.size() * sizeof(float)),
                 v.data(), GL_STATIC_DRAW);

    // Ground quad, slightly below the grid to avoid line z-fighting.
    float e = diag_ * 1.4f;
    float gz = z - diag_ * 0.0015f;
    float x0 = centerX_ - e, x1 = centerX_ + e;
    float y0 = centerY_ - e, y1 = centerY_ + e;
    const float q[] = {
        x0, y0, gz, -1.f, -1.f,
        x1, y0, gz,  1.f, -1.f,
        x1, y1, gz,  1.f,  1.f,
        x0, y0, gz, -1.f, -1.f,
        x1, y1, gz,  1.f,  1.f,
        x0, y1, gz, -1.f,  1.f,
    };
    glBindBuffer(GL_ARRAY_BUFFER, groundVbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(q), q, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// Upload mesh vertex data once per setMesh: positions block followed by a
// normals block in one VBO (avoids a large temporary interleaving copy).
void Renderer::uploadMesh() {
    if (!modern_) return;

    meshVerts_ = 0;
    meshHasNormals_ = false;
    if (!mesh_ || mesh_->positions.empty()) {
        glBindBuffer(GL_ARRAY_BUFFER, meshVbo_);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        return;
    }

    meshVerts_ = static_cast<int>(mesh_->positions.size());
    meshHasNormals_ = mesh_->normals.size() == mesh_->positions.size();
    GLsizeiptr posBytes =
        static_cast<GLsizeiptr>(mesh_->positions.size() * sizeof(Vec3f));
    GLsizeiptr nrmBytes = meshHasNormals_ ? posBytes : 0;

    glBindVertexArray(meshVao_);
    glBindBuffer(GL_ARRAY_BUFFER, meshVbo_);
    glBufferData(GL_ARRAY_BUFFER, posBytes + nrmBytes, nullptr, GL_STATIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, posBytes, mesh_->positions.data());
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3f),
                          reinterpret_cast<const void*>(0));
    if (meshHasNormals_) {
        glBufferSubData(GL_ARRAY_BUFFER, posBytes, nrmBytes,
                        mesh_->normals.data());
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3f),
                              reinterpret_cast<const void*>(
                                  static_cast<uintptr_t>(posBytes)));
    } else {
        glDisableVertexAttribArray(1);
        glVertexAttrib3f(1, 0.f, 0.f, 1.f);
    }
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// --- lifecycle / camera ------------------------------------------------------

void Renderer::shutdown() {
    if (ctxValid_) {
        platform::makeContextCurrent();
        if (modern_) destroyModernResources();
        platform::destroyGLContext();
        ctxValid_ = false;
    }
    mesh_ = nullptr;
    modern_ = false;
}

void Renderer::resize(int w, int h) {
    width_ = (w > 0) ? w : 1;
    height_ = (h > 0) ? h : 1;
}

void Renderer::setMesh(Mesh* mesh) {
    mesh_ = mesh;
    // Reset the tree UI state for the new model (visibility flags default to
    // true in MeshNode itself).
    treeExpanded_.assign(mesh_ ? mesh_->nodes.size() : 0, 1);
    treeScroll_ = 0;
    lastToggleNode_ = -1;
    lastToggleVis_.clear();
    rebuildDrawRanges();
    if (mesh_ && !mesh_->positions.empty()) {
        const Vec3f& mn = mesh_->bboxMin;
        const Vec3f& mx = mesh_->bboxMax;
        bbMin_[0] = mn.x; bbMin_[1] = mn.y; bbMin_[2] = mn.z;
        bbMax_[0] = mx.x; bbMax_[1] = mx.y; bbMax_[2] = mx.z;
        centerX_ = (mn.x + mx.x) * 0.5f;
        centerY_ = (mn.y + mx.y) * 0.5f;
        centerZ_ = (mn.z + mx.z) * 0.5f;
        float dx = mx.x - mn.x, dy = mx.y - mn.y, dz = mx.z - mn.z;
        diag_ = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (diag_ < 1e-6f) diag_ = 1.f;
        groundZ_ = mn.z - diag_ * 0.002f; // tiny offset avoids z-fighting
    } else {
        for (int i = 0; i < 3; ++i) { bbMin_[i] = 0.f; bbMax_[i] = 0.f; }
        centerX_ = centerY_ = centerZ_ = 0.f;
        diag_ = 10.f;
        groundZ_ = 0.f;
    }
    if (ctxValid_ && modern_) {
        platform::makeContextCurrent();
        uploadMesh();
        rebuildSceneBuffers();
    }
}

// Place the camera so the given world-space box fills the view.
void Renderer::fitToBox(const float mn[3], const float mx[3]) {
    targetX_ = (mn[0] + mx[0]) * 0.5f;
    targetY_ = (mn[1] + mx[1]) * 0.5f;
    targetZ_ = (mn[2] + mx[2]) * 0.5f;
    float dx = mx[0] - mn[0], dy = mx[1] - mn[1], dz = mx[2] - mn[2];
    float radius = 0.5f * std::sqrt(dx * dx + dy * dy + dz * dz);
    if (radius < 1e-6f) radius = 1.f;
    float aspect = static_cast<float>(width_) / static_cast<float>(height_);
    float vHalf = kFovYDeg * 0.5f * kDeg2Rad;
    float hHalf = std::atan(std::tan(vHalf) * aspect);
    float half = (vHalf < hHalf) ? vHalf : hHalf;
    dist_ = radius / std::sin(half) * 1.05f;
}

void Renderer::fitToBounds() {
    // Fit the visible part when some tree nodes are hidden, else the whole
    // model bbox.
    float mn[3], mx[3];
    if (visibleBounds(mn, mx)) fitToBox(mn, mx);
    else fitToBox(bbMin_, bbMax_);
}

void Renderer::orbit(float dx, float dy) {
    yawDeg_ -= dx * 0.4f;
    pitchDeg_ += dy * 0.4f;
    if (pitchDeg_ > 89.f) pitchDeg_ = 89.f;
    if (pitchDeg_ < -89.f) pitchDeg_ = -89.f;
}

void Renderer::pan(float dx, float dy) {
    float toEye[3], right[3], up[3];
    cameraBasis(yawDeg_, pitchDeg_, toEye, right, up);
    // World units per pixel at the target distance.
    float wpp = 2.f * dist_ * std::tan(kFovYDeg * 0.5f * kDeg2Rad)
                / static_cast<float>(height_);
    targetX_ += (-dx * right[0] + dy * up[0]) * wpp;
    targetY_ += (-dx * right[1] + dy * up[1]) * wpp;
    targetZ_ += (-dx * right[2] + dy * up[2]) * wpp;
}

void Renderer::zoom(float wheelSteps) {
    dist_ *= std::pow(0.88f, wheelSteps);
    float lo = diag_ * 0.005f;
    float hi = diag_ * 100.f;
    if (dist_ < lo) dist_ = lo;
    if (dist_ > hi) dist_ = hi;
}

bool Renderer::toggleWireframe() { wireframe_ = !wireframe_; return wireframe_; }
bool Renderer::toggleSmooth()    { smooth_ = !smooth_;       return smooth_; }
bool Renderer::toggleGrid()      { grid_ = !grid_;           return grid_; }

bool Renderer::toggleShadows() {
    if (!modern_) return false;
    shadows_ = !shadows_;
    return shadows_;
}

bool Renderer::toggleHelp() { showHelp_ = !showHelp_; return showHelp_; }
bool Renderer::toggleInfo() { showInfo_ = !showInfo_; return showInfo_; }

void Renderer::setModelName(const std::string& nameUtf8) {
    modelName_.clear();
    modelName_.reserve(nameUtf8.size());
    // One '?' per non-ASCII codepoint (skip UTF-8 continuation bytes).
    for (size_t i = 0; i < nameUtf8.size();) {
        unsigned char c = static_cast<unsigned char>(nameUtf8[i]);
        if (c < 0x80) {
            modelName_ += (c >= 32 && c < 127) ? static_cast<char>(c) : '?';
            ++i;
        } else {
            modelName_ += '?';
            size_t n = (c >= 0xF0) ? 4 : (c >= 0xE0) ? 3 : 2;
            i += n;
        }
    }
}

// --- model tree & per-node visibility ----------------------------------------

bool Renderer::toggleTree() {
    if (!mesh_ || mesh_->nodes.empty()) return false;   // nothing to show
    showTree_ = !showTree_;
    return showTree_;
}

// The panel is drawn (and clickable) only on the modern path with a working
// overlay; the visibility filter itself works on both paths.
bool Renderer::treePanelVisible() const {
    return modern_ && overlayOk_ && showTree_ && mesh_ &&
           !mesh_->nodes.empty();
}

// Children lists in index order. A node whose parent index is invalid (or
// does not precede it) is treated as a root, so a malformed tree cannot
// cause cycles.
void Renderer::buildChildren(std::vector<std::vector<int>>& kids) const {
    const std::vector<MeshNode>& nodes = mesh_->nodes;
    kids.assign(nodes.size(), {});
    for (size_t i = 0; i < nodes.size(); ++i) {
        int p = nodes[i].parent;
        if (p >= 0 && static_cast<size_t>(p) < i)
            kids[static_cast<size_t>(p)].push_back(static_cast<int>(i));
    }
}

// Depth-first list of the rows currently on display (children of collapsed
// nodes are skipped).
void Renderer::buildTreeRows(std::vector<TreeRow>& rows) const {
    rows.clear();
    if (!mesh_) return;
    const std::vector<MeshNode>& nodes = mesh_->nodes;
    std::vector<std::vector<int>> kids;
    buildChildren(kids);

    std::vector<std::pair<int, int>> stack;   // node, depth
    for (size_t i = nodes.size(); i-- > 0;) {
        int p = nodes[i].parent;
        if (p < 0 || static_cast<size_t>(p) >= i)
            stack.emplace_back(static_cast<int>(i), 0);
    }
    while (!stack.empty()) {
        int n = stack.back().first;
        int d = stack.back().second;
        stack.pop_back();
        TreeRow row;
        row.node = n;
        row.depth = d;
        row.hasChildren = !kids[static_cast<size_t>(n)].empty();
        rows.push_back(row);
        bool expanded = static_cast<size_t>(n) < treeExpanded_.size() &&
                        treeExpanded_[static_cast<size_t>(n)] != 0;
        if (row.hasChildren && expanded) {
            const std::vector<int>& c = kids[static_cast<size_t>(n)];
            for (size_t j = c.size(); j-- > 0;)
                stack.emplace_back(c[j], d + 1);
        }
    }
}

// Aggregated visibility over each node's geometry (its own triangles plus
// all descendants'): 0 = hidden, 1 = visible, 2 = mixed. Nodes without any
// geometry below them report their own flag. Single bottom-up pass (children
// always follow their parent in the vector).
void Renderer::computeVisStates(std::vector<unsigned char>& st) const {
    const std::vector<MeshNode>& nodes = mesh_->nodes;
    size_t n = nodes.size();
    std::vector<int> vis(n, 0), tot(n, 0);
    for (size_t i = n; i-- > 0;) {
        if (nodes[i].triCount > 0) {
            ++tot[i];
            if (nodes[i].visible) ++vis[i];
        }
        int p = nodes[i].parent;
        if (p >= 0 && static_cast<size_t>(p) < i) {
            vis[static_cast<size_t>(p)] += vis[i];
            tot[static_cast<size_t>(p)] += tot[i];
        }
    }
    st.resize(n);
    for (size_t i = 0; i < n; ++i) {
        int s;
        if (tot[i] == 0) s = nodes[i].visible ? 1 : 0;
        else s = (vis[i] == 0) ? 0 : (vis[i] == tot[i]) ? 1 : 2;
        st[i] = static_cast<unsigned char>(s);
    }
}

// Toggle a node: a fully visible subtree gets hidden, anything else (hidden
// or mixed) becomes fully visible. The previous flags are recorded so that
// the first click of a double-click can be undone before fitting the view.
void Renderer::toggleNodeVisible(int node) {
    if (!mesh_ || node < 0 ||
        static_cast<size_t>(node) >= mesh_->nodes.size())
        return;
    std::vector<unsigned char> st;
    computeVisStates(st);
    bool newVis = st[static_cast<size_t>(node)] != 1;

    std::vector<std::vector<int>> kids;
    buildChildren(kids);
    lastToggleNode_ = node;
    lastToggleVis_.clear();
    std::vector<int> stack{node};
    while (!stack.empty()) {
        int n = stack.back();
        stack.pop_back();
        MeshNode& mn = mesh_->nodes[static_cast<size_t>(n)];
        lastToggleVis_.emplace_back(n, static_cast<char>(mn.visible ? 1 : 0));
        mn.visible = newVis;
        for (int c : kids[static_cast<size_t>(n)]) stack.push_back(c);
    }
    rebuildDrawRanges();
}

// Collect the triangle ranges of the visible leaves, merge the contiguous
// ones and store them as vertex ranges. Typically this yields very few
// glDrawArrays calls (one when everything is visible).
void Renderer::rebuildDrawRanges() {
    drawRanges_.clear();
    if (!mesh_ || mesh_->positions.empty()) return;
    size_t totalTris = mesh_->triangleCount();
    if (mesh_->nodes.empty()) {
        drawRanges_.emplace_back(0, static_cast<int>(totalTris * 3));
        return;
    }
    std::vector<std::pair<size_t, size_t>> spans;   // [start, end) triangles
    for (const MeshNode& n : mesh_->nodes) {
        if (n.triCount == 0 || !n.visible) continue;
        size_t s = n.triStart;
        size_t e = n.triStart + n.triCount;
        if (s >= totalTris) continue;
        if (e > totalTris) e = totalTris;
        spans.emplace_back(s, e);
    }
    std::sort(spans.begin(), spans.end());
    size_t curS = 0, curE = 0;
    bool open = false;
    for (const std::pair<size_t, size_t>& sp : spans) {
        if (open && sp.first <= curE) {
            if (sp.second > curE) curE = sp.second;
        } else {
            if (open)
                drawRanges_.emplace_back(static_cast<int>(curS * 3),
                                         static_cast<int>((curE - curS) * 3));
            curS = sp.first;
            curE = sp.second;
            open = true;
        }
    }
    if (open)
        drawRanges_.emplace_back(static_cast<int>(curS * 3),
                                 static_cast<int>((curE - curS) * 3));
}

// Issue the mesh draw calls for the visible ranges (used by the color,
// wireframe, shadow-depth and legacy passes alike).
void Renderer::drawMeshRanges() const {
    for (const std::pair<int, int>& r : drawRanges_)
        glDrawArrays(GL_TRIANGLES, r.first, r.second);
}

// Bbox of all triangles below a node (regardless of visibility).
bool Renderer::nodeBounds(int node, float mn[3], float mx[3]) const {
    if (!mesh_ || node < 0 ||
        static_cast<size_t>(node) >= mesh_->nodes.size())
        return false;
    std::vector<std::vector<int>> kids;
    buildChildren(kids);
    bool any = false;
    size_t totalVerts = mesh_->positions.size();
    std::vector<int> stack{node};
    while (!stack.empty()) {
        int n = stack.back();
        stack.pop_back();
        const MeshNode& nd = mesh_->nodes[static_cast<size_t>(n)];
        if (nd.triCount > 0) {
            size_t v0 = nd.triStart * 3;
            size_t v1 = v0 + nd.triCount * 3;
            if (v1 > totalVerts) v1 = totalVerts;
            for (size_t v = v0; v < v1; ++v) {
                const Vec3f& p = mesh_->positions[v];
                if (!any) {
                    mn[0] = mx[0] = p.x;
                    mn[1] = mx[1] = p.y;
                    mn[2] = mx[2] = p.z;
                    any = true;
                } else {
                    if (p.x < mn[0]) mn[0] = p.x;
                    if (p.y < mn[1]) mn[1] = p.y;
                    if (p.z < mn[2]) mn[2] = p.z;
                    if (p.x > mx[0]) mx[0] = p.x;
                    if (p.y > mx[1]) mx[1] = p.y;
                    if (p.z > mx[2]) mx[2] = p.z;
                }
            }
        }
        for (int c : kids[static_cast<size_t>(n)]) stack.push_back(c);
    }
    return any;
}

// Bbox of the currently visible geometry. Returns false when nothing is
// hidden (the cached full bbox applies) or nothing is visible.
bool Renderer::visibleBounds(float mn[3], float mx[3]) const {
    if (!mesh_ || mesh_->nodes.empty()) return false;
    bool anyHidden = false;
    for (const MeshNode& n : mesh_->nodes) {
        if (n.triCount > 0 && !n.visible) { anyHidden = true; break; }
    }
    if (!anyHidden) return false;
    bool any = false;
    size_t totalVerts = mesh_->positions.size();
    for (const std::pair<int, int>& r : drawRanges_) {
        size_t v0 = static_cast<size_t>(r.first);
        size_t v1 = v0 + static_cast<size_t>(r.second);
        if (v1 > totalVerts) v1 = totalVerts;
        for (size_t v = v0; v < v1; ++v) {
            const Vec3f& p = mesh_->positions[v];
            if (!any) {
                mn[0] = mx[0] = p.x;
                mn[1] = mx[1] = p.y;
                mn[2] = mx[2] = p.z;
                any = true;
            } else {
                if (p.x < mn[0]) mn[0] = p.x;
                if (p.y < mn[1]) mn[1] = p.y;
                if (p.z < mn[2]) mn[2] = p.z;
                if (p.x > mx[0]) mx[0] = p.x;
                if (p.y > mx[1]) mx[1] = p.y;
                if (p.z > mx[2]) mx[2] = p.z;
            }
        }
    }
    return any;
}

// Panel geometry for the current window size and scroll position.
void Renderer::computeTreeLayout(TreeLayout& out) const {
    out = TreeLayout();
    if (!treePanelVisible()) return;
    buildTreeRows(out.rows);
    if (out.rows.empty()) return;

    const float ch = overlay_.charWidth();
    const float lh = overlay_.lineHeight();
    out.rowH = lh + 4.f;
    out.indent = ch * 2.f;
    const float headerH = lh + 8.f;   // title line + separator

    // Width: longest row wins, clamped to ~35% of the window (names are
    // then truncated at draw time). +8 keeps room for the scroll bar.
    float maxRow = overlay_.textWidth("Model tree  000/000");
    for (const TreeRow& r : out.rows) {
        float wRow = static_cast<float>(r.depth) * out.indent + kTreeArrowW +
                     kTreeCheckW +
                     overlay_.textWidth(mesh_->nodes[
                         static_cast<size_t>(r.node)].name);
        if (wRow > maxRow) maxRow = wRow;
    }
    float wMax = static_cast<float>(width_) * 0.35f;
    if (wMax < 200.f) wMax = 200.f;
    out.w = maxRow + 2.f * kPanelPad + 8.f;
    if (out.w > wMax) out.w = wMax;

    // Height: at most ~60% of the window; the rest scrolls.
    int total = static_cast<int>(out.rows.size());
    float maxH = static_cast<float>(height_) * 0.60f;
    int fitRows =
        static_cast<int>((maxH - 2.f * kPanelPad - headerH) / out.rowH);
    if (fitRows < 1) fitRows = 1;
    out.shown = (total < fitRows) ? total : fitRows;
    out.first = treeScroll_;
    if (out.first > total - out.shown) out.first = total - out.shown;
    if (out.first < 0) out.first = 0;

    out.x = kPanelMargin;
    out.y = kPanelMargin;
    out.h = 2.f * kPanelPad + headerH +
            static_cast<float>(out.shown) * out.rowH;
    out.rowsY = out.y + kPanelPad + headerH;
    out.valid = true;
}

// Draw the panel into the current overlay batch (between begin() and end()).
void Renderer::drawTreePanel() {
    TreeLayout L;
    computeTreeLayout(L);
    if (!L.valid) return;

    const float ch = overlay_.charWidth();
    const float lh = overlay_.lineHeight();
    std::vector<unsigned char> st;
    computeVisStates(st);

    overlay_.rect(L.x, L.y, L.w, L.h, 0.03f, 0.04f, 0.06f, 0.62f);

    // Header: title plus visible/total object count and a separator rule.
    int totalObj = 0, visObj = 0;
    for (const MeshNode& n : mesh_->nodes) {
        if (n.triCount > 0) {
            ++totalObj;
            if (n.visible) ++visObj;
        }
    }
    char hdr[64];
    std::snprintf(hdr, sizeof(hdr), "Model tree  %d/%d", visObj, totalObj);
    overlay_.text(L.x + kPanelPad, L.y + kPanelPad, hdr,
                  0.92f, 0.94f, 0.97f, 1.f);
    overlay_.rect(L.x + kPanelPad, L.y + kPanelPad + lh + 3.f,
                  L.w - 2.f * kPanelPad, 1.f, 0.55f, 0.60f, 0.68f, 0.35f);

    int total = static_cast<int>(L.rows.size());
    for (int i = L.first; i < L.first + L.shown && i < total; ++i) {
        const TreeRow& r = L.rows[static_cast<size_t>(i)];
        const MeshNode& nd = mesh_->nodes[static_cast<size_t>(r.node)];
        float ry = L.rowsY + static_cast<float>(i - L.first) * L.rowH;
        float rx = L.x + kPanelPad + static_cast<float>(r.depth) * L.indent;
        float mid = ry + L.rowH * 0.5f;

        // Expand/collapse triangle (down = expanded, right = collapsed).
        if (r.hasChildren) {
            bool exp = treeExpanded_[static_cast<size_t>(r.node)] != 0;
            float ax = rx + 2.f;
            if (exp)
                overlay_.tri(ax, mid - 2.f, ax + 8.f, mid - 2.f,
                             ax + 4.f, mid + 4.f,
                             0.70f, 0.75f, 0.82f, 0.95f);
            else
                overlay_.tri(ax + 1.f, mid - 4.f, ax + 7.f, mid,
                             ax + 1.f, mid + 4.f,
                             0.70f, 0.75f, 0.82f, 0.95f);
        }

        // Checkbox: [x] visible, [ ] hidden, [-] mixed subtree.
        unsigned char s = st[static_cast<size_t>(r.node)];
        const float cb = 11.f;
        float cx = rx + kTreeArrowW;
        float cy = mid - cb * 0.5f;
        overlay_.rect(cx, cy, cb, cb, 0.55f, 0.60f, 0.68f, 0.90f);
        overlay_.rect(cx + 1.f, cy + 1.f, cb - 2.f, cb - 2.f,
                      0.05f, 0.06f, 0.09f, 1.f);
        if (s == 1)
            overlay_.rect(cx + 3.f, cy + 3.f, cb - 6.f, cb - 6.f,
                          0.55f, 0.78f, 0.95f, 1.f);
        else if (s == 2)
            overlay_.rect(cx + 3.f, mid - 1.f, cb - 6.f, 2.f,
                          0.55f, 0.78f, 0.95f, 1.f);

        // Name (dimmed when hidden), truncated with "..." if too long.
        float tx = cx + kTreeCheckW;
        float avail = L.x + L.w - kPanelPad - 6.f - tx;
        int maxChars = static_cast<int>(avail / ch);
        if (maxChars < 0) maxChars = 0;
        std::string name = nd.name.empty() ? std::string("(unnamed)")
                                           : nd.name;
        if (static_cast<int>(name.size()) > maxChars) {
            if (maxChars > 3)
                name = name.substr(0, static_cast<size_t>(maxChars - 3)) +
                       "...";
            else
                name = name.substr(0, static_cast<size_t>(maxChars));
        }
        float ty = mid - lh * 0.5f;
        if (s == 0)
            overlay_.text(tx, ty, name, 0.48f, 0.51f, 0.56f, 1.f);
        else
            overlay_.text(tx, ty, name, 0.82f, 0.86f, 0.92f, 1.f);
    }

    // Simple scroll indicator when the tree exceeds the panel.
    if (total > L.shown) {
        float trackX = L.x + L.w - 5.f;
        float trackY = L.rowsY;
        float trackH = static_cast<float>(L.shown) * L.rowH;
        overlay_.rect(trackX, trackY, 3.f, trackH, 1.f, 1.f, 1.f, 0.08f);
        float thumbH = trackH * static_cast<float>(L.shown) /
                       static_cast<float>(total);
        if (thumbH < 12.f) thumbH = 12.f;
        float thumbY = trackY + (trackH - thumbH) *
                       static_cast<float>(L.first) /
                       static_cast<float>(total - L.shown);
        overlay_.rect(trackX, thumbY, 3.f, thumbH, 1.f, 1.f, 1.f, 0.30f);
    }
}

bool Renderer::treeHitTest(int x, int y) const {
    TreeLayout L;
    computeTreeLayout(L);
    if (!L.valid) return false;
    float fx = static_cast<float>(x), fy = static_cast<float>(y);
    return fx >= L.x && fx < L.x + L.w && fy >= L.y && fy < L.y + L.h;
}

bool Renderer::treeClick(int x, int y) {
    TreeLayout L;
    computeTreeLayout(L);
    if (!L.valid) return false;
    float fx = static_cast<float>(x), fy = static_cast<float>(y);
    if (fx < L.x || fx >= L.x + L.w || fy < L.y || fy >= L.y + L.h)
        return false;

    // Which row was hit?
    int row = -1;
    if (fy >= L.rowsY) {
        int i = L.first + static_cast<int>((fy - L.rowsY) / L.rowH);
        if (i >= L.first && i < L.first + L.shown &&
            i < static_cast<int>(L.rows.size()))
            row = i;
    }
    if (row < 0) return true;   // header/padding: consumed, no action

    const TreeRow& r = L.rows[static_cast<size_t>(row)];
    float rx = L.x + kPanelPad + static_cast<float>(r.depth) * L.indent;
    if (r.hasChildren && fx >= rx - 2.f && fx < rx + kTreeArrowW - 2.f) {
        // Arrow: expand/collapse the subtree.
        char& e = treeExpanded_[static_cast<size_t>(r.node)];
        e = e ? 0 : 1;
        lastToggleNode_ = -1;
    } else {
        // Checkbox or row body: toggle visibility (recursively for groups).
        toggleNodeVisible(r.node);
    }
    return true;
}

bool Renderer::treeWheel(int x, int y, float steps) {
    if (!treeHitTest(x, y)) return false;
    TreeLayout L;
    computeTreeLayout(L);
    int total = static_cast<int>(L.rows.size());
    int maxFirst = total - L.shown;
    if (maxFirst < 0) maxFirst = 0;
    int d = static_cast<int>(std::lround(static_cast<double>(steps) * 3.0));
    if (d == 0) d = (steps > 0.f) ? 1 : -1;
    int next = L.first - d;      // wheel forward scrolls towards the top
    if (next > maxFirst) next = maxFirst;
    if (next < 0) next = 0;
    treeScroll_ = next;
    return true;
}

// Double click on a row: undo the toggle performed by the first click of
// the pair, then fit the view on that part's bounding box.
bool Renderer::treeDoubleClick(int x, int y) {
    TreeLayout L;
    computeTreeLayout(L);
    if (!L.valid) return false;
    float fx = static_cast<float>(x), fy = static_cast<float>(y);
    if (fx < L.x || fx >= L.x + L.w || fy < L.y || fy >= L.y + L.h)
        return false;

    int row = -1;
    if (fy >= L.rowsY) {
        int i = L.first + static_cast<int>((fy - L.rowsY) / L.rowH);
        if (i >= L.first && i < L.first + L.shown &&
            i < static_cast<int>(L.rows.size()))
            row = i;
    }
    if (row < 0) return true;

    const TreeRow& r = L.rows[static_cast<size_t>(row)];
    float rx = L.x + kPanelPad + static_cast<float>(r.depth) * L.indent;
    if (r.hasChildren && fx >= rx - 2.f && fx < rx + kTreeArrowW - 2.f) {
        // Rapid clicks on the arrow keep expanding/collapsing.
        char& e = treeExpanded_[static_cast<size_t>(r.node)];
        e = e ? 0 : 1;
        return true;
    }

    if (lastToggleNode_ == r.node) {
        for (const std::pair<int, char>& pv : lastToggleVis_) {
            if (pv.first >= 0 &&
                static_cast<size_t>(pv.first) < mesh_->nodes.size())
                mesh_->nodes[static_cast<size_t>(pv.first)].visible =
                    pv.second != 0;
        }
        lastToggleNode_ = -1;
        lastToggleVis_.clear();
        rebuildDrawRanges();
    }
    float mn[3], mx[3];
    if (nodeBounds(r.node, mn, mx)) fitToBox(mn, mx);
    return true;
}

// --- section planes ----------------------------------------------------------

bool Renderer::toggleSection(int axis) {
    if (axis < 0 || axis > 2) return false;
    sections_[axis].enabled = !sections_[axis].enabled;
    return sections_[axis].enabled;
}

void Renderer::invertSection(int axis) {
    if (axis < 0 || axis > 2) return;
    sections_[axis].flip = !sections_[axis].flip;
}

void Renderer::moveSection(int axis, float steps) {
    if (axis < 0 || axis > 2) return;
    // 2% of the bbox extent per wheel notch, clamped to the bbox.
    float t = sections_[axis].t + steps * 0.02f;
    if (t < 0.f) t = 0.f;
    if (t > 1.f) t = 1.f;
    sections_[axis].t = t;
}

bool Renderer::sectionEnabled(int axis) const {
    return axis >= 0 && axis <= 2 && sections_[axis].enabled;
}

void Renderer::setSectionHighlight(int axis) {
    sectionHighlight_ = (axis >= 0 && axis <= 2) ? axis : -1;
}

// World-space plane equations (keep the half-space where dot >= 0). Disabled
// planes get a harmless always-positive equation; they are only evaluated if
// the matching GL_CLIP_DISTANCEi is enabled anyway.
void Renderer::computeClipPlanes(float planes[12]) const {
    for (int i = 0; i < 3; ++i) {
        float* p = planes + i * 4;
        p[0] = p[1] = p[2] = 0.f;
        p[3] = 1.f;
        if (!sections_[i].enabled) continue;
        float pos = bbMin_[i] + sections_[i].t * (bbMax_[i] - bbMin_[i]);
        float s = sections_[i].flip ? 1.f : -1.f;   // kept side
        p[i] = s;
        p[3] = -s * pos;
    }
}

void Renderer::enableClipPlanes() const {
    for (int i = 0; i < 3; ++i)
        if (sections_[i].enabled) glEnable(GL_CLIP_DISTANCE0 + i);
}

void Renderer::disableClipPlanes() const {
    for (int i = 0; i < 3; ++i)
        if (sections_[i].enabled) glDisable(GL_CLIP_DISTANCE0 + i);
}

// --- modern rendering --------------------------------------------------------

// Orthographic light frustum fitted around the scene bounding box (with room
// for the shadow spilling onto the ground plane).
void Renderer::computeLightVP(float* out) const {
    float dir[3] = {kKeyLightDir[0], kKeyLightDir[1], kKeyLightDir[2]};
    normalize3(dir);
    const float center[3] = {centerX_, centerY_, centerZ_};
    const float eye[3] = {center[0] + dir[0] * diag_ * 1.6f,
                          center[1] + dir[1] * diag_ * 1.6f,
                          center[2] + dir[2] * diag_ * 1.6f};
    const float up[3] = {0.f, 0.f, 1.f};
    Mat4 view = matLookAt(eye, center, up);
    float r = diag_;
    Mat4 proj = matOrtho(-r, r, -r, r, diag_ * 0.2f, diag_ * 3.2f);
    Mat4 vp = matMul(proj, view);
    std::memcpy(out, vp.m, sizeof(vp.m));
}

void Renderer::renderShadowPass(const float* lightVP,
                                const float* clipPlanes) {
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
    glViewport(0, 0, shadowSize_, shadowSize_);
    glClear(GL_DEPTH_BUFFER_BIT);
    glUseProgram(progDepth_);
    glUniformMatrix4fv(uDepthLightVP_, 1, GL_FALSE, lightVP);
    // Clipped-away geometry must not cast a shadow either.
    glUniform4fv(uDepthClip_, 3, clipPlanes);
    enableClipPlanes();
    // Slope-scaled depth bias against shadow acne.
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(2.f, 4.f);
    glBindVertexArray(meshVao_);
    drawMeshRanges();   // hidden parts cast no shadow
    glDisable(GL_POLYGON_OFFSET_FILL);
    disableClipPlanes();
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Renderer::renderModern() {
    // Camera matrices (same adaptive near/far logic as the legacy path).
    float znear = dist_ * 0.001f;
    float lowest = diag_ * 1e-5f;
    if (znear < lowest) znear = lowest;
    if (znear < 1e-6f) znear = 1e-6f;
    float zfar = dist_ * 10.f + diag_ * 2.f;
    if (zfar < znear * 10.f) zfar = znear * 10.f;
    float aspect = static_cast<float>(width_) / static_cast<float>(height_);
    Mat4 proj = matPerspective(kFovYDeg, aspect, znear, zfar);

    float toEye[3], right[3], up[3];
    cameraBasis(yawDeg_, pitchDeg_, toEye, right, up);
    const float eye[3] = {targetX_ + toEye[0] * dist_,
                          targetY_ + toEye[1] * dist_,
                          targetZ_ + toEye[2] * dist_};
    const float target[3] = {targetX_, targetY_, targetZ_};
    Mat4 view = matLookAt(eye, target, up);
    Mat4 viewProj = matMul(proj, view);

    // Section planes (applied to the mesh and its shadow only).
    float clip[12];
    computeClipPlanes(clip);

    // Depth-only pass (only when shadows are on and there is a mesh).
    float lightVP[16];
    bool doShadow = shadows_ && meshVerts_ > 0;
    if (doShadow) {
        computeLightVP(lightVP);
        renderShadowPass(lightVP, clip);
    }

    glViewport(0, 0, width_, height_);
    glClear(GL_DEPTH_BUFFER_BIT);   // color fully covered by the background

    // Background gradient (fullscreen triangle, no depth).
    glDisable(GL_DEPTH_TEST);
    glUseProgram(progBg_);
    glBindVertexArray(bgVao_);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glEnable(GL_DEPTH_TEST);

    // Grid + axes.
    if (grid_ && lineVerts_ > 0) {
        glUseProgram(progLine_);
        glUniformMatrix4fv(uLineViewProj_, 1, GL_FALSE, viewProj.m);
        glBindVertexArray(lineVao_);
        glDrawArrays(GL_LINES, 0, lineVerts_);
    }

    // Mesh: one draw call per visible range, with section clipping enabled.
    if (meshVerts_ > 0) {
        glBindVertexArray(meshVao_);
        enableClipPlanes();
        if (wireframe_) {
            glUseProgram(progWire_);
            glUniformMatrix4fv(uWireViewProj_, 1, GL_FALSE, viewProj.m);
            glUniform3f(uWireColor_, 0.75f, 0.80f, 0.88f);
            glUniform1f(uWireAlpha_, 1.f);
            glUniform4fv(uWireClip_, 3, clip);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            drawMeshRanges();
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        } else {
            glUseProgram(progLit_);
            glUniformMatrix4fv(uLitViewProj_, 1, GL_FALSE, viewProj.m);
            if (doShadow)
                glUniformMatrix4fv(uLitLightVP_, 1, GL_FALSE, lightVP);
            glUniform3f(uLitCamPos_, eye[0], eye[1], eye[2]);
            // Flat mode also kicks in when the mesh has no usable normals.
            glUniform1i(uLitFlat_, (!smooth_ || !meshHasNormals_) ? 1 : 0);
            glUniform1i(uLitShadows_, doShadow ? 1 : 0);
            glUniform1f(uLitTexel_, 1.f / static_cast<float>(shadowSize_));
            glUniform4fv(uLitClip_, 3, clip);
            drawMeshRanges();
        }
        disableClipPlanes();
    }

    // Ground shadow catcher (transparent where lit), after opaque geometry.
    if (doShadow && !wireframe_) {
        glUseProgram(progGround_);
        glUniformMatrix4fv(uGroundViewProj_, 1, GL_FALSE, viewProj.m);
        glUniformMatrix4fv(uGroundLightVP_, 1, GL_FALSE, lightVP);
        glUniform1f(uGroundTexel_, 1.f / static_cast<float>(shadowSize_));
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        glBindVertexArray(groundVao_);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }

    // Section-plane outline while its key is held, then the 2D overlay.
    drawSectionHighlight(viewProj.m);
    renderOverlay();

    glBindVertexArray(0);
    glUseProgram(0);
    platform::swapBuffers();
}

// Semi-transparent accent quad + outline showing the plane being adjusted.
void Renderer::drawSectionHighlight(const float* viewProj) {
    int a = sectionHighlight_;
    if (a < 0 || !sections_[a].enabled || meshVerts_ == 0) return;

    float pos = bbMin_[a] + sections_[a].t * (bbMax_[a] - bbMin_[a]);
    int u = (a + 1) % 3, v = (a + 2) % 3;
    // Slightly larger than the bbox cross-section so it reads as a plane.
    float mu = (bbMax_[u] - bbMin_[u]) * 0.04f + diag_ * 0.01f;
    float mv = (bbMax_[v] - bbMin_[v]) * 0.04f + diag_ * 0.01f;
    float lu = bbMin_[u] - mu, hu = bbMax_[u] + mu;
    float lv = bbMin_[v] - mv, hv = bbMax_[v] + mv;

    float q[12];
    const float cu[4] = {lu, hu, hu, lu};
    const float cv[4] = {lv, lv, hv, hv};
    for (int i = 0; i < 4; ++i) {
        q[i * 3 + a] = pos;
        q[i * 3 + u] = cu[i];
        q[i * 3 + v] = cv[i];
    }
    glBindBuffer(GL_ARRAY_BUFFER, secVbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(q), q, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glUseProgram(progWire_);
    glUniformMatrix4fv(uWireViewProj_, 1, GL_FALSE, viewProj);
    glUniform3f(uWireColor_, 1.0f, 0.62f, 0.25f);   // accent orange
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glBindVertexArray(secVao_);
    glUniform1f(uWireAlpha_, 0.12f);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glUniform1f(uWireAlpha_, 0.85f);
    glDrawArrays(GL_LINE_LOOP, 0, 4);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

// Short status like "X 46%(-)  Z 80%(+)" for the enabled sections; the sign
// shows the kept side of the cut.
std::string Renderer::sectionSummary() const {
    std::string s;
    const char axisName[3] = {'X', 'Y', 'Z'};
    for (int i = 0; i < 3; ++i) {
        if (!sections_[i].enabled) continue;
        if (!s.empty()) s += "  ";
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%c %d%%(%c)", axisName[i],
                      static_cast<int>(sections_[i].t * 100.f + 0.5f),
                      sections_[i].flip ? '+' : '-');
        s += buf;
    }
    return s;
}

void Renderer::renderOverlay() {
    if (!overlayOk_ || (!showHelp_ && !showInfo_ && !treePanelVisible()))
        return;

    const float pad = 10.f;
    const float lh = overlay_.lineHeight() + 2.f;
    const float margin = 12.f;
    overlay_.begin(width_, height_);

    if (showHelp_) {
        std::vector<std::string> lines = {
            "Mouse: L orbit   R/M pan   Wheel zoom   Dbl-click fit",
            "W wireframe   S smooth/flat   G grid   O shadows   F fit",
            "X/Y/Z section   hold + Wheel move   Shift+X/Y/Z flip side",
            "T tree   I model info   H help   F11 fullscreen   Ctrl+O open   Esc quit",
        };
        std::string sec = sectionSummary();
        if (!sec.empty()) lines.push_back("Sections: " + sec);

        float w = 0.f;
        for (const std::string& l : lines) {
            float tw = overlay_.textWidth(l);
            if (tw > w) w = tw;
        }
        float h = static_cast<float>(lines.size()) * lh + 2.f * pad;
        float x = margin;
        float y = static_cast<float>(height_) - margin - h;
        overlay_.rect(x, y, w + 2.f * pad, h, 0.03f, 0.04f, 0.06f, 0.62f);
        for (size_t i = 0; i < lines.size(); ++i) {
            overlay_.text(x + pad, y + pad + static_cast<float>(i) * lh,
                          lines[i], 0.82f, 0.86f, 0.92f, 1.f);
        }
    }

    if (showInfo_) {
        std::vector<std::string> lines;
        lines.push_back("File: " +
                        (modelName_.empty() ? std::string("(none)")
                                            : modelName_));
        if (mesh_ && !mesh_->positions.empty()) {
            lines.push_back("Triangles: " + fmtGrouped(mesh_->triangleCount()));
            lines.push_back("Vertices:  " +
                            fmtGrouped(mesh_->positions.size()));
            if (!mesh_->nodes.empty()) {
                int totalObj = 0, visObj = 0;
                for (const MeshNode& n : mesh_->nodes) {
                    if (n.triCount > 0) {
                        ++totalObj;
                        if (n.visible) ++visObj;
                    }
                }
                lines.push_back("Objects: " + std::to_string(visObj) + "/" +
                                std::to_string(totalObj) + " visible");
            }
            lines.push_back("Size: " + fmtFloat(bbMax_[0] - bbMin_[0]) +
                            " x " + fmtFloat(bbMax_[1] - bbMin_[1]) +
                            " x " + fmtFloat(bbMax_[2] - bbMin_[2]));
            lines.push_back("Diagonal: " + fmtFloat(diag_));
        }
        lines.push_back(std::string("Shadows: ") + (shadows_ ? "on" : "off") +
                        "   MSAA: " +
                        (msaaSamples_ > 0
                             ? std::to_string(msaaSamples_) + "x"
                             : std::string("off")));
        std::string sec = sectionSummary();
        lines.push_back("Sections: " + (sec.empty() ? "off" : sec));

        float w = 0.f;
        for (const std::string& l : lines) {
            float tw = overlay_.textWidth(l);
            if (tw > w) w = tw;
        }
        float pw = w + 2.f * pad;
        float h = static_cast<float>(lines.size()) * lh + 2.f * pad;
        float x = static_cast<float>(width_) - margin - pw;
        float y = margin;
        overlay_.rect(x, y, pw, h, 0.03f, 0.04f, 0.06f, 0.62f);
        for (size_t i = 0; i < lines.size(); ++i) {
            overlay_.text(x + pad, y + pad + static_cast<float>(i) * lh,
                          lines[i], 0.82f, 0.86f, 0.92f, 1.f);
        }
    }

    if (treePanelVisible()) drawTreePanel();

    overlay_.end();
}

// --- legacy fixed-function path ----------------------------------------------

void Renderer::initLegacyState() {
    glClearColor(0.13f, 0.15f, 0.18f, 1.f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glDisable(GL_CULL_FACE);

    glEnable(GL_LIGHTING);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    const float sceneAmbient[4] = {0.18f, 0.18f, 0.20f, 1.f};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, sceneAmbient);

    const float keyDiffuse[4]  = {0.85f, 0.85f, 0.85f, 1.f};
    const float keySpecular[4] = {0.30f, 0.30f, 0.30f, 1.f};
    const float fillDiffuse[4] = {0.28f, 0.30f, 0.33f, 1.f};
    const float black[4] = {0.f, 0.f, 0.f, 1.f};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, keyDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, keySpecular);
    glLightfv(GL_LIGHT0, GL_AMBIENT, black);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, fillDiffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, black);
    glLightfv(GL_LIGHT1, GL_AMBIENT, black);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);

    const float matDiffuse[4]  = {0.55f, 0.62f, 0.72f, 1.f};
    const float matAmbient[4]  = {0.55f, 0.62f, 0.72f, 1.f};
    const float matSpecular[4] = {0.25f, 0.25f, 0.25f, 1.f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, matDiffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, matAmbient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpecular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 32.f);
}

void Renderer::applyProjection() const {
    double znear = dist_ * 0.001;
    double lowest = diag_ * 1e-5;
    if (znear < lowest) znear = lowest;
    if (znear < 1e-6) znear = 1e-6;
    double zfar = dist_ * 10.0 + diag_ * 2.0;
    if (zfar < znear * 10.0) zfar = znear * 10.0;

    double aspect = static_cast<double>(width_) / static_cast<double>(height_);
    double top = znear * std::tan(kFovYDeg * 0.5 * kDeg2Rad);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-top * aspect, top * aspect, -top, top, znear, zfar);
}

void Renderer::applyLights() const {
    // Positions set with identity modelview => fixed in eye space.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    const float keyDir[4]  = {0.3f, 0.5f, 1.0f, 0.f};
    const float fillDir[4] = {-0.4f, -0.3f, -1.0f, 0.f};
    glLightfv(GL_LIGHT0, GL_POSITION, keyDir);
    glLightfv(GL_LIGHT1, GL_POSITION, fillDir);
}

void Renderer::applyView() const {
    float toEye[3], right[3], up[3];
    cameraBasis(yawDeg_, pitchDeg_, toEye, right, up);
    float eye[3] = {targetX_ + toEye[0] * dist_,
                    targetY_ + toEye[1] * dist_,
                    targetZ_ + toEye[2] * dist_};
    const float target[3] = {targetX_, targetY_, targetZ_};
    Mat4 m = matLookAt(eye, target, up);
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(m.m);
}

void Renderer::drawMeshLegacy() const {
    if (!mesh_ || mesh_->positions.empty()) return;

    glShadeModel(smooth_ ? GL_SMOOTH : GL_FLAT);

    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, mesh_->positions.data());
    bool hasNormals = mesh_->normals.size() == mesh_->positions.size();
    if (hasNormals) {
        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, 0, mesh_->normals.data());
    }

    // Only the visible ranges are drawn (whole mesh = one range).
    if (wireframe_) {
        glDisable(GL_LIGHTING);
        glColor3f(0.75f, 0.80f, 0.88f);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        drawMeshRanges();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_LIGHTING);
    } else {
        drawMeshRanges();
    }

    if (hasNormals) glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
}

void Renderer::drawGridAndAxesLegacy() const {
    GridLayout g = gridLayout(diag_, centerX_, centerY_);
    float z = groundZ_;

    glDisable(GL_LIGHTING);

    glColor3f(0.28f, 0.30f, 0.33f);
    glBegin(GL_LINES);
    for (int i = -g.n; i <= g.n; ++i) {
        float o = static_cast<float>(i) * g.step;
        glVertex3f(g.cx - g.half, g.cy + o, z);
        glVertex3f(g.cx + g.half, g.cy + o, z);
        glVertex3f(g.cx + o, g.cy - g.half, z);
        glVertex3f(g.cx + o, g.cy + g.half, z);
    }
    glEnd();

    float alen = g.half * 0.5f;
    glBegin(GL_LINES);
    glColor3f(0.85f, 0.25f, 0.25f);
    glVertex3f(g.cx, g.cy, z); glVertex3f(g.cx + alen, g.cy, z);
    glColor3f(0.25f, 0.75f, 0.30f);
    glVertex3f(g.cx, g.cy, z); glVertex3f(g.cx, g.cy + alen, z);
    glColor3f(0.30f, 0.45f, 0.90f);
    glVertex3f(g.cx, g.cy, z); glVertex3f(g.cx, g.cy, z + alen);
    glEnd();

    glEnable(GL_LIGHTING);
}

void Renderer::renderLegacy() {
    glViewport(0, 0, width_, height_);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    applyProjection();
    applyLights();
    applyView();

    if (grid_) drawGridAndAxesLegacy();

    // Section planes via fixed-function clip planes. The modelview matrix
    // currently holds the view transform, so the equations (world space) are
    // interpreted correctly by glClipPlane. Grid/axes stay unclipped.
    // NOTE: the text overlay is not available on this fallback path.
    float clip[12];
    computeClipPlanes(clip);
    for (int i = 0; i < 3; ++i) {
        if (!sections_[i].enabled) continue;
        GLdouble eq[4] = {clip[i * 4 + 0], clip[i * 4 + 1],
                          clip[i * 4 + 2], clip[i * 4 + 3]};
        glClipPlane(GL_CLIP_PLANE0 + i, eq);
        glEnable(GL_CLIP_PLANE0 + i);
    }
    drawMeshLegacy();
    for (int i = 0; i < 3; ++i)
        if (sections_[i].enabled) glDisable(GL_CLIP_PLANE0 + i);

    platform::swapBuffers();
}

// --- entry point -------------------------------------------------------------

void Renderer::render() {
    if (!ctxValid_) return;
    platform::makeContextCurrent();
    if (modern_) renderModern();
    else renderLegacy();
}
