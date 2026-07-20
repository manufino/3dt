/*
 * overlay.h
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
// 2D text/panel overlay for the modern (GL 3.3 core) path.
//
// A glyph atlas covering printable ASCII (32..126) is rasterized once at
// init through platform::rasterizeFontAtlas() (GDI system font on Windows,
// embedded bitmap font on Linux) and uploaded as a single-channel GL_R8
// texture. Text and solid rectangles are batched as textured quads and drawn
// in one call with a pixel-space orthographic shader (origin top-left).
// Rectangles sample a reserved solid-white atlas cell, so one program/
// texture serves both.
//
// Not available on the legacy fixed-function fallback context.

#include <string>
#include <vector>

class Overlay {
public:
    // Requires a current GL 3.3 core context with gl_loader initialized.
    // Returns false on failure (overlay is then simply skipped).
    bool init();
    void destroy();

    float lineHeight() const { return static_cast<float>(cellH_); }
    float charWidth() const { return static_cast<float>(advance_); }
    float textWidth(const std::string& s) const {
        return static_cast<float>(s.size()) * static_cast<float>(advance_);
    }

    // Batched drawing; pixel coordinates, origin at the top-left corner.
    void begin(int viewportW, int viewportH);
    void rect(float x, float y, float w, float h,
              float r, float g, float b, float a);
    // Solid triangle (counter-clockwise or clockwise, culling is off).
    void tri(float x0, float y0, float x1, float y1, float x2, float y2,
             float r, float g, float b, float a);
    // Draws the string with a 1px dark drop shadow. Characters outside
    // printable ASCII are shown as '?'.
    void text(float x, float y, const std::string& s,
              float r, float g, float b, float a);
    void end();   // uploads the batch and draws it with blending enabled

private:
    void solidUV(float& u, float& v) const;
    void drawRun(float x, float y, const std::string& s,
                 float r, float g, float b, float a);
    void pushQuad(float x0, float y0, float x1, float y1,
                  float u0, float v0, float u1, float v1,
                  float r, float g, float b, float a);

    unsigned tex_ = 0, vao_ = 0, vbo_ = 0, prog_ = 0;
    int uViewport_ = -1;
    int cellW_ = 0, cellH_ = 0, advance_ = 0;
    int atlasW_ = 1, atlasH_ = 1;
    int vpW_ = 1, vpH_ = 1;
    std::vector<float> verts_;   // x,y,u,v,r,g,b,a per vertex
};
