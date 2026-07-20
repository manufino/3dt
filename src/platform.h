/*
 * platform.h
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
// Platform abstraction for the 3dt viewer.
//
// A deliberately small interface: one window, one GL context, an event loop
// that reports input through callbacks, and a handful of services that need
// OS support (file dialog, message box, system-font rasterization for the
// text overlay). Implemented by platform_win32.cpp (Win32/WGL/GDI) and
// platform_x11.cpp (Xlib/GLX). This header is pure C++ - no system includes -
// so renderer/overlay/app stay 100% portable.
//
// Threading: everything here is single-threaded; all callbacks fire on the
// thread that runs runEventLoop().

#include <functional>
#include <string>
#include <vector>

namespace platform {

// Abstract key codes for the keys the app cares about. Everything else is
// left to the platform default handling. ("Unknown" rather than "None":
// X11 headers #define None, which would clash on the Linux backend.)
enum class Key {
    Unknown,
    W, S, G, O, F, H, I, T, X, Y, Z,
    F11,
    Escape
};

// Modifier bit mask reported with key events.
enum : unsigned {
    ModCtrl  = 1u,
    ModShift = 2u
};

enum class MouseButton { Left, Middle, Right };

// A file reference as delivered by argv / drag&drop / the open dialog.
// - path:    byte path suitable for std::ifstream on this platform
//            (Windows: ANSI code page, with an 8.3 short-name fallback for
//            paths CP_ACP cannot represent; Linux: raw bytes, i.e. UTF-8).
// - display: the full path as UTF-8, for window titles / error messages.
struct FileRef {
    std::string path;
    std::string display;
};

// Glyph atlas produced by rasterizeFontAtlas(): a cols x rows grid of
// fixed-size cells, one 8-bit coverage value per pixel (0 = transparent,
// 255 = opaque), row-major, width = cellW * cols, height = cellH * rows.
// Glyph for character (firstChar + i) is drawn in cell i with a 1px padding
// offset; 'advance' is the (monospace) horizontal advance in pixels.
struct FontAtlas {
    int cellW = 0, cellH = 0;
    int advance = 0;
    int width = 0, height = 0;
    std::vector<unsigned char> coverage;
};

// Event callbacks from the platform into the app. Any of them may be left
// empty. Coordinates are client-area pixels, origin top-left.
struct Callbacks {
    std::function<void(int x, int y)> onMouseMove;
    std::function<void(MouseButton b, bool down, int x, int y)> onMouseButton;
    // steps: +1 per wheel notch away from the user (zoom in).
    std::function<void(float steps)> onMouseWheel;
    std::function<void(int x, int y)> onDoubleClick;
    // repeat is true for key auto-repeat events.
    std::function<void(Key k, bool down, unsigned mods, bool repeat)> onKey;
    std::function<void(int w, int h)> onResize;
    // Draw one frame now (rendering is on-demand, see requestRedraw()).
    std::function<void()> onDraw;
    std::function<void(const FileRef& file)> onDropFile;
    // The window is going away; release GL resources here.
    std::function<void()> onClose;
};

// Result of createGLContext().
struct GLContextInfo {
    bool modern = false;   // true: 3.3 core; false: legacy fixed-function
    int msaaSamples = 0;   // actual samples of the framebuffer (0 = none)
};

// --- window ------------------------------------------------------------------

// Create the (single) application window, initially hidden. The callbacks
// are copied and used for the whole life of the window.
bool createWindow(const char* titleUtf8, int width, int height,
                  const Callbacks& cb);
void showWindow();
void setWindowTitle(const char* titleUtf8);
void getClientSize(int& w, int& h);

// Borderless-fullscreen toggle on the monitor the window is currently on.
// The resulting resize is reported through onResize as usual.
void toggleFullscreen();

// Schedule a redraw: onDraw will be invoked (once) from the event loop.
void requestRedraw();

// Keep receiving mouse-move events while a drag leaves the window
// (no-op where the OS already grabs the pointer during a button drag).
void captureMouse(bool on);

// Async key state, for wheel-with-key-held combinations.
bool isKeyDown(Key k);

// --- OpenGL context ----------------------------------------------------------

// Create a GL context on the window: first a 3.3 core profile with MSAA
// (8x -> 4x -> none), falling back to a legacy context if that fails.
// Returns false only if no context at all could be created. On success the
// context is current.
bool createGLContext(GLContextInfo& out);

// Drop the current (modern) context and create a legacy one in its place;
// used when 3.3 function loading / resource setup fails after context
// creation. On success the new context is current.
bool recreateLegacyGLContext();

void destroyGLContext();
void makeContextCurrent();
void swapBuffers();

// Resolve a GL function (core or extension); null if unavailable.
void* getGLProcAddress(const char* name);

// Best-effort vsync control (no-op if unsupported).
void setSwapInterval(int interval);

// --- services ----------------------------------------------------------------

// Modal "open file" dialog. Returns false if the user cancelled OR if the
// platform has no dialog available (Linux without zenity/kdialog): the app
// stays usable via drag&drop and command-line arguments.
bool openFileDialog(FileRef& out);

// Modal message box (Linux: zenity/kdialog if present, else stderr).
void showMessageBox(const char* title, const char* textUtf8, bool isError);

// Rasterize the printable-ASCII range [firstChar, lastChar] (expected
// 32..126) of a monospace system font into a cols x rows cell grid.
// Cells beyond the last glyph are left empty. Returns false on failure.
bool rasterizeFontAtlas(int firstChar, int lastChar, int cols, int rows,
                        FontAtlas& out);

// Debug/diagnostic output (Windows: OutputDebugString; Linux: stderr).
// The message is emitted verbatim (append '\n' yourself).
void debugLog(const char* msg);

// --- event loop --------------------------------------------------------------

// Run until quit() / window close. Returns the process exit code.
int runEventLoop();

// Request the event loop to exit (destroys the window; onClose fires).
void quit();

} // namespace platform

// Portable application entry point, implemented in app.cpp and invoked by
// the platform-specific entry point (wWinMain / main) with the command-line
// file arguments already converted to FileRefs.
int appMain(const std::vector<platform::FileRef>& files);
