/*
 * overlay.cpp
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

// Overlay implementation: platform-rasterized glyph atlas + batched
// textured quads.

#include "overlay.h"
#include "gl_loader.h"
#include "platform.h"

#include <cstring>
#include <vector>

#ifndef GL_R8
#define GL_R8 0x8229
#endif
#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

namespace {

constexpr int kFirstChar = 32;
constexpr int kLastChar  = 126;
constexpr int kCols = 16;
constexpr int kRows = 6;          // 95 glyphs + 1 solid cell = 96 = 16 x 6
constexpr int kSolidCell = 95;    // atlas cell filled with opaque white

const char* kOverlayVS = R"GLSL(
#version 330 core
layout(location = 0) in vec2 aPos;    // pixels, origin top-left
layout(location = 1) in vec2 aUV;
layout(location = 2) in vec4 aColor;
uniform vec2 uViewport;
out vec2 vUV;
out vec4 vColor;
void main() {
    vUV = aUV;
    vColor = aColor;
    vec2 ndc = vec2(aPos.x / uViewport.x * 2.0 - 1.0,
                    1.0 - aPos.y / uViewport.y * 2.0);
    gl_Position = vec4(ndc, 0.0, 1.0);
}
)GLSL";

const char* kOverlayFS = R"GLSL(
#version 330 core
in vec2 vUV;
in vec4 vColor;
out vec4 fragColor;
uniform sampler2D uAtlas;
void main() {
    fragColor = vec4(vColor.rgb, vColor.a * texture(uAtlas, vUV).r);
}
)GLSL";

GLuint compileOverlayStage(GLenum type, const char* src) {
    GLuint sh = glCreateShader(type);
    glShaderSource(sh, 1, &src, nullptr);
    glCompileShader(sh);
    GLint ok = GL_FALSE;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024] = "";
        GLsizei len = 0;
        glGetShaderInfoLog(sh, sizeof(log) - 1, &len, log);
        platform::debugLog("3dt: overlay shader compile failed:\n");
        platform::debugLog(log);
        glDeleteShader(sh);
        return 0;
    }
    return sh;
}

GLuint linkOverlayProgram() {
    GLuint vs = compileOverlayStage(GL_VERTEX_SHADER, kOverlayVS);
    if (!vs) return 0;
    GLuint fs = compileOverlayStage(GL_FRAGMENT_SHADER, kOverlayFS);
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
        char log[1024] = "";
        GLsizei len = 0;
        glGetProgramInfoLog(prog, sizeof(log) - 1, &len, log);
        platform::debugLog("3dt: overlay program link failed:\n");
        platform::debugLog(log);
        glDeleteProgram(prog);
        return 0;
    }
    return prog;
}

} // namespace

bool Overlay::init() {
    // --- rasterize the glyph atlas via the platform layer -------------------
    platform::FontAtlas atlas;
    if (!platform::rasterizeFontAtlas(kFirstChar, kLastChar, kCols, kRows,
                                      atlas))
        return false;
    if (atlas.cellW <= 0 || atlas.cellH <= 0 || atlas.advance <= 0 ||
        atlas.coverage.size() !=
            static_cast<size_t>(atlas.width) * static_cast<size_t>(atlas.height))
        return false;

    advance_ = atlas.advance;            // monospace advance
    cellW_ = atlas.cellW;
    cellH_ = atlas.cellH;
    atlasW_ = atlas.width;
    atlasH_ = atlas.height;

    // Stamp the reserved solid-white cell (constant coverage for rects).
    std::vector<unsigned char>& gray = atlas.coverage;
    {
        int col = kSolidCell % kCols, row = kSolidCell / kCols;
        for (int y = 0; y < cellH_; ++y) {
            std::memset(&gray[static_cast<size_t>(row * cellH_ + y) * atlasW_ +
                              static_cast<size_t>(col) * cellW_],
                        255, static_cast<size_t>(cellW_));
        }
    }

    // --- GL resources (atlas lives on texture unit 1) -----------------------
    glActiveTexture(GL_TEXTURE1);
    glGenTextures(1, &tex_);
    glBindTexture(GL_TEXTURE_2D, tex_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, atlasW_, atlasH_, 0,
                 GL_RED, GL_UNSIGNED_BYTE, gray.data());
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glActiveTexture(GL_TEXTURE0);

    prog_ = linkOverlayProgram();
    if (!prog_) {
        destroy();
        return false;
    }
    uViewport_ = glGetUniformLocation(prog_, "uViewport");
    glUseProgram(prog_);
    glUniform1i(glGetUniformLocation(prog_, "uAtlas"), 1);
    glUseProgram(0);

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    const GLsizei stride = 8 * sizeof(float);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<const void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<const void*>(2 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<const void*>(4 * sizeof(float)));
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    return true;
}

void Overlay::destroy() {
    if (vbo_)  { glDeleteBuffers(1, &vbo_); vbo_ = 0; }
    if (vao_)  { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
    if (prog_) { glDeleteProgram(prog_); prog_ = 0; }
    if (tex_)  { glDeleteTextures(1, &tex_); tex_ = 0; }
    verts_.clear();
}

void Overlay::begin(int viewportW, int viewportH) {
    vpW_ = (viewportW > 0) ? viewportW : 1;
    vpH_ = (viewportH > 0) ? viewportH : 1;
    verts_.clear();
}

void Overlay::pushQuad(float x0, float y0, float x1, float y1,
                       float u0, float v0, float u1, float v1,
                       float r, float g, float b, float a) {
    const float q[6][4] = {
        {x0, y0, u0, v0}, {x1, y0, u1, v0}, {x1, y1, u1, v1},
        {x0, y0, u0, v0}, {x1, y1, u1, v1}, {x0, y1, u0, v1},
    };
    for (const float* p : q) {
        verts_.push_back(p[0]); verts_.push_back(p[1]);
        verts_.push_back(p[2]); verts_.push_back(p[3]);
        verts_.push_back(r); verts_.push_back(g);
        verts_.push_back(b); verts_.push_back(a);
    }
}

// UV of the center of the solid-white cell (constant coverage 1).
void Overlay::solidUV(float& u, float& v) const {
    u = (static_cast<float>(kSolidCell % kCols) + 0.5f) *
        static_cast<float>(cellW_) / static_cast<float>(atlasW_);
    v = (static_cast<float>(kSolidCell / kCols) + 0.5f) *
        static_cast<float>(cellH_) / static_cast<float>(atlasH_);
}

void Overlay::rect(float x, float y, float w, float h,
                   float r, float g, float b, float a) {
    float u, v;
    solidUV(u, v);
    pushQuad(x, y, x + w, y + h, u, v, u, v, r, g, b, a);
}

void Overlay::tri(float x0, float y0, float x1, float y1, float x2, float y2,
                  float r, float g, float b, float a) {
    float u, v;
    solidUV(u, v);
    const float p[3][2] = {{x0, y0}, {x1, y1}, {x2, y2}};
    for (int i = 0; i < 3; ++i) {
        verts_.push_back(p[i][0]); verts_.push_back(p[i][1]);
        verts_.push_back(u); verts_.push_back(v);
        verts_.push_back(r); verts_.push_back(g);
        verts_.push_back(b); verts_.push_back(a);
    }
}

void Overlay::drawRun(float x, float y, const std::string& s,
                      float r, float g, float b, float a) {
    float cx = x;
    for (char c : s) {
        unsigned char uc = static_cast<unsigned char>(c);
        if (uc < kFirstChar || uc > kLastChar) uc = '?';
        if (uc != ' ') {
            int i = uc - kFirstChar;
            int col = i % kCols, row = i / kCols;
            float u0 = static_cast<float>(col * cellW_) /
                       static_cast<float>(atlasW_);
            float v0 = static_cast<float>(row * cellH_) /
                       static_cast<float>(atlasH_);
            float u1 = static_cast<float>((col + 1) * cellW_) /
                       static_cast<float>(atlasW_);
            float v1 = static_cast<float>((row + 1) * cellH_) /
                       static_cast<float>(atlasH_);
            // Quad covers the whole padded cell, glyph was drawn at +1,+1.
            pushQuad(cx - 1.f, y - 1.f,
                     cx - 1.f + static_cast<float>(cellW_),
                     y - 1.f + static_cast<float>(cellH_),
                     u0, v0, u1, v1, r, g, b, a);
        }
        cx += static_cast<float>(advance_);
    }
}

void Overlay::text(float x, float y, const std::string& s,
                   float r, float g, float b, float a) {
    drawRun(x + 1.f, y + 1.f, s, 0.f, 0.f, 0.f, a * 0.55f);  // drop shadow
    drawRun(x, y, s, r, g, b, a);
}

void Overlay::end() {
    if (verts_.empty() || !prog_) return;

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(verts_.size() * sizeof(float)),
                 verts_.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, tex_);
    glActiveTexture(GL_TEXTURE0);

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(prog_);
    glUniform2f(uViewport_, static_cast<float>(vpW_),
                static_cast<float>(vpH_));
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(verts_.size() / 8));
    glBindVertexArray(0);
    glUseProgram(0);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}
