/*
 * bitmap_font.h
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
// Embedded 5x7 dot-matrix font for the text overlay, shared by the platform
// backends that do not use a system font rasterizer (X11, macOS). Pure C++
// (data + one function, no system includes), so the translation unit
// compiles everywhere - on Windows it is simply an unused object file.

#include "platform.h"

namespace bitmapfont {

// Expand the embedded 5x7 glyphs (printable ASCII 32..126) into 8x16 cells
// with the exact layout contract of platform::rasterizeFontAtlas(): glyph i
// drawn in cell i at a 1px padding offset, 0/255 coverage, monospace
// advance. Returns false if the requested range is outside 32..126 or does
// not fit the cols x rows grid.
bool rasterize(int firstChar, int lastChar, int cols, int rows,
               platform::FontAtlas& out);

} // namespace bitmapfont
