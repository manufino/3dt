/*
 * platform_x11.cpp
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

// Linux/X11 implementation of platform.h, written directly against Xlib and
// GLX (no GLFW/SDL/Qt):
// - GLX: glXChooseFBConfig with GLX_SAMPLES (MSAA 8x -> 4x -> none),
//   glXCreateContextAttribsARB for a 3.3 core context with a
//   glXCreateNewContext legacy fallback, glXGetProcAddressARB for the GL
//   loader, glXSwapIntervalEXT/MESA for vsync when available.
// - Input: KeyPress/KeyRelease (XLookupKeysym), ButtonPress (buttons 4/5 =
//   wheel), MotionNotify, ConfigureNotify, ClientMessage WM_DELETE_WINDOW.
//   Double click is synthesized from two quick Button1 presses.
// - Drag&drop: the XDND v5 protocol implemented by hand (XdndEnter/Position/
//   Status/Drop/Finished, text/uri-list selection, percent-encoded file://
//   URI decoding).
// - File dialog: zenity, then kdialog, via popen; if neither exists the
//   call reports failure and the app stays usable via drag&drop/argv.
// - Overlay font: an embedded public-domain 5x7 dot-matrix font (hand-drawn
//   below), expanded to 8x16 cells at runtime - no font libraries.
// - Fullscreen: EWMH _NET_WM_STATE_FULLSCREEN toggle via ClientMessage.
//
// Build deps (Debian/Ubuntu): libx11-dev + mesa dev headers (libgl1-mesa-dev
// or libglvnd-dev). Link with -lX11 -lGL.
//
// NOTE: developed on Windows; this backend still needs testing on a real
// Linux machine.

#ifdef __linux__

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>
#include <X11/XKBlib.h>
#include <X11/keysym.h>
#include <GL/glx.h>

#include <sys/wait.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "platform.h"

#ifndef GLX_CONTEXT_MAJOR_VERSION_ARB
#define GLX_CONTEXT_MAJOR_VERSION_ARB 0x2091
#endif
#ifndef GLX_CONTEXT_MINOR_VERSION_ARB
#define GLX_CONTEXT_MINOR_VERSION_ARB 0x2092
#endif
#ifndef GLX_CONTEXT_PROFILE_MASK_ARB
#define GLX_CONTEXT_PROFILE_MASK_ARB 0x9126
#endif
#ifndef GLX_CONTEXT_CORE_PROFILE_BIT_ARB
#define GLX_CONTEXT_CORE_PROFILE_BIT_ARB 0x00000001
#endif

namespace {

typedef GLXContext (*GlXCreateContextAttribsARBProc)(
    Display*, GLXFBConfig, GLXContext, Bool, const int*);
typedef void (*GlXSwapIntervalEXTProc)(Display*, GLXDrawable, int);
typedef int (*GlXSwapIntervalMESAProc)(unsigned int);

// Cast through void* to silence -Wcast-function-type on GLX proc lookups.
template <typename T>
T glxProc(const char* name) {
    return reinterpret_cast<T>(reinterpret_cast<void*>(
        glXGetProcAddressARB(reinterpret_cast<const GLubyte*>(name))));
}

Display* g_dpy = nullptr;
Window g_win = 0;
Colormap g_cmap = 0;
GLXFBConfig g_fbc = nullptr;
GLXContext g_ctx = nullptr;
bool g_ctxModern = false;
int g_fbSamples = 0;             // MSAA level of the chosen FBConfig

platform::Callbacks g_cb;
bool g_running = false;
bool g_dirty = false;            // a redraw was requested
int g_width = 1, g_height = 1;

// Async key state (updated from KeyPress/KeyRelease).
bool g_keyDown[32] = {};

// Double-click synthesis.
Time g_lastClickTime = 0;
int g_lastClickX = -10000, g_lastClickY = -10000;

// Atoms.
Atom g_wmProtocols = 0, g_wmDeleteWindow = 0;
Atom g_netWmName = 0, g_utf8String = 0;
Atom g_netWmState = 0, g_netWmStateFullscreen = 0;
Atom g_xdndAware = 0, g_xdndEnter = 0, g_xdndPosition = 0, g_xdndStatus = 0;
Atom g_xdndLeave = 0, g_xdndDrop = 0, g_xdndFinished = 0;
Atom g_xdndSelection = 0, g_xdndTypeList = 0, g_xdndActionCopy = 0;
Atom g_textUriList = 0, g_dndTargetProp = 0;

// XDND session state.
Window g_xdndSource = 0;
int g_xdndVersion = 0;
bool g_xdndHaveUriList = false;

// --- embedded font -----------------------------------------------------------

// Hand-drawn 5x7 dot-matrix glyphs for printable ASCII 32..126 (in the
// style of classic character-generator ROMs; drawn from scratch for this
// project, so the data carries no third-party license). Each glyph is 7
// rows of 5 bits, bit 0x10 = leftmost pixel. Expanded to 8x16 cells by
// rasterizeFontAtlas() (rows doubled, centered).
const unsigned char kFont5x7[95][7] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x04,0x04,0x04,0x04,0x04,0x00,0x04}, // '!'
    {0x0A,0x0A,0x0A,0x00,0x00,0x00,0x00}, // '"'
    {0x0A,0x0A,0x1F,0x0A,0x1F,0x0A,0x0A}, // '#'
    {0x04,0x0F,0x14,0x0E,0x05,0x1E,0x04}, // '$'
    {0x18,0x19,0x02,0x04,0x08,0x13,0x03}, // '%'
    {0x0C,0x12,0x14,0x08,0x15,0x12,0x0D}, // '&'
    {0x0C,0x04,0x08,0x00,0x00,0x00,0x00}, // '\''
    {0x02,0x04,0x08,0x08,0x08,0x04,0x02}, // '('
    {0x08,0x04,0x02,0x02,0x02,0x04,0x08}, // ')'
    {0x00,0x04,0x15,0x0E,0x15,0x04,0x00}, // '*'
    {0x00,0x04,0x04,0x1F,0x04,0x04,0x00}, // '+'
    {0x00,0x00,0x00,0x00,0x0C,0x04,0x08}, // ','
    {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}, // '-'
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}, // '.'
    {0x00,0x01,0x02,0x04,0x08,0x10,0x00}, // '/'
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}, // '0'
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, // '1'
    {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}, // '2'
    {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E}, // '3'
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}, // '4'
    {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}, // '5'
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}, // '6'
    {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}, // '7'
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}, // '8'
    {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}, // '9'
    {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00}, // ':'
    {0x00,0x0C,0x0C,0x00,0x0C,0x04,0x08}, // ';'
    {0x02,0x04,0x08,0x10,0x08,0x04,0x02}, // '<'
    {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00}, // '='
    {0x08,0x04,0x02,0x01,0x02,0x04,0x08}, // '>'
    {0x0E,0x11,0x01,0x02,0x04,0x00,0x04}, // '?'
    {0x0E,0x11,0x01,0x0D,0x15,0x15,0x0E}, // '@'
    {0x0E,0x11,0x11,0x11,0x1F,0x11,0x11}, // 'A'
    {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}, // 'B'
    {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}, // 'C'
    {0x1C,0x12,0x11,0x11,0x11,0x12,0x1C}, // 'D'
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}, // 'E'
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}, // 'F'
    {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F}, // 'G'
    {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}, // 'H'
    {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}, // 'I'
    {0x07,0x02,0x02,0x02,0x02,0x12,0x0C}, // 'J'
    {0x11,0x12,0x14,0x18,0x14,0x12,0x11}, // 'K'
    {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}, // 'L'
    {0x11,0x1B,0x15,0x15,0x11,0x11,0x11}, // 'M'
    {0x11,0x11,0x19,0x15,0x13,0x11,0x11}, // 'N'
    {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}, // 'O'
    {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}, // 'P'
    {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}, // 'Q'
    {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}, // 'R'
    {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}, // 'S'
    {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}, // 'T'
    {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}, // 'U'
    {0x11,0x11,0x11,0x11,0x11,0x0A,0x04}, // 'V'
    {0x11,0x11,0x11,0x15,0x15,0x15,0x0A}, // 'W'
    {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}, // 'X'
    {0x11,0x11,0x11,0x0A,0x04,0x04,0x04}, // 'Y'
    {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}, // 'Z'
    {0x0E,0x08,0x08,0x08,0x08,0x08,0x0E}, // '['
    {0x00,0x10,0x08,0x04,0x02,0x01,0x00}, // '\\'
    {0x0E,0x02,0x02,0x02,0x02,0x02,0x0E}, // ']'
    {0x04,0x0A,0x11,0x00,0x00,0x00,0x00}, // '^'
    {0x00,0x00,0x00,0x00,0x00,0x00,0x1F}, // '_'
    {0x08,0x04,0x02,0x00,0x00,0x00,0x00}, // '`'
    {0x00,0x00,0x0E,0x01,0x0F,0x11,0x0F}, // 'a'
    {0x10,0x10,0x1E,0x11,0x11,0x11,0x1E}, // 'b'
    {0x00,0x00,0x0E,0x10,0x10,0x11,0x0E}, // 'c'
    {0x01,0x01,0x0F,0x11,0x11,0x11,0x0F}, // 'd'
    {0x00,0x00,0x0E,0x11,0x1F,0x10,0x0E}, // 'e'
    {0x06,0x09,0x08,0x1C,0x08,0x08,0x08}, // 'f'
    {0x00,0x0F,0x11,0x11,0x0F,0x01,0x0E}, // 'g'
    {0x10,0x10,0x16,0x19,0x11,0x11,0x11}, // 'h'
    {0x04,0x00,0x0C,0x04,0x04,0x04,0x0E}, // 'i'
    {0x02,0x00,0x06,0x02,0x02,0x12,0x0C}, // 'j'
    {0x10,0x10,0x12,0x14,0x18,0x14,0x12}, // 'k'
    {0x0C,0x04,0x04,0x04,0x04,0x04,0x0E}, // 'l'
    {0x00,0x00,0x1A,0x15,0x15,0x11,0x11}, // 'm'
    {0x00,0x00,0x16,0x19,0x11,0x11,0x11}, // 'n'
    {0x00,0x00,0x0E,0x11,0x11,0x11,0x0E}, // 'o'
    {0x00,0x00,0x1E,0x11,0x1E,0x10,0x10}, // 'p'
    {0x00,0x00,0x0F,0x11,0x0F,0x01,0x01}, // 'q'
    {0x00,0x00,0x16,0x19,0x10,0x10,0x10}, // 'r'
    {0x00,0x00,0x0E,0x10,0x0E,0x01,0x1E}, // 's'
    {0x08,0x08,0x1C,0x08,0x08,0x09,0x06}, // 't'
    {0x00,0x00,0x11,0x11,0x11,0x13,0x0D}, // 'u'
    {0x00,0x00,0x11,0x11,0x11,0x0A,0x04}, // 'v'
    {0x00,0x00,0x11,0x11,0x15,0x15,0x0A}, // 'w'
    {0x00,0x00,0x11,0x0A,0x04,0x0A,0x11}, // 'x'
    {0x00,0x00,0x11,0x11,0x0F,0x01,0x0E}, // 'y'
    {0x00,0x00,0x1F,0x02,0x04,0x08,0x1F}, // 'z'
    {0x02,0x04,0x04,0x08,0x04,0x04,0x02}, // '{'
    {0x04,0x04,0x04,0x04,0x04,0x04,0x04}, // '|'
    {0x08,0x04,0x04,0x02,0x04,0x04,0x08}, // '}'
    {0x00,0x08,0x15,0x02,0x00,0x00,0x00}, // '~'
};

// --- key mapping -------------------------------------------------------------

platform::Key keysymToKey(KeySym ks) {
    using platform::Key;
    switch (ks) {
    case XK_w: case XK_W: return Key::W;
    case XK_s: case XK_S: return Key::S;
    case XK_g: case XK_G: return Key::G;
    case XK_o: case XK_O: return Key::O;
    case XK_f: case XK_F: return Key::F;
    case XK_h: case XK_H: return Key::H;
    case XK_i: case XK_I: return Key::I;
    case XK_t: case XK_T: return Key::T;
    case XK_x: case XK_X: return Key::X;
    case XK_y: case XK_Y: return Key::Y;
    case XK_z: case XK_Z: return Key::Z;
    case XK_F11:    return Key::F11;
    case XK_Escape: return Key::Escape;
    default:        return Key::Unknown;
    }
}

unsigned stateToMods(unsigned int state) {
    unsigned mods = 0;
    if (state & ControlMask) mods |= platform::ModCtrl;
    if (state & ShiftMask)   mods |= platform::ModShift;
    return mods;
}

bool& keyDownSlot(platform::Key k) {
    return g_keyDown[static_cast<int>(k) & 31];
}

// --- small helpers -----------------------------------------------------------

void sendClientMessage(Window target, Atom type, long l0, long l1, long l2,
                       long l3, long l4) {
    XEvent e;
    std::memset(&e, 0, sizeof(e));
    e.xclient.type = ClientMessage;
    e.xclient.display = g_dpy;
    e.xclient.window = target;
    e.xclient.message_type = type;
    e.xclient.format = 32;
    e.xclient.data.l[0] = l0;
    e.xclient.data.l[1] = l1;
    e.xclient.data.l[2] = l2;
    e.xclient.data.l[3] = l3;
    e.xclient.data.l[4] = l4;
    XSendEvent(g_dpy, target, False, NoEventMask, &e);
    XFlush(g_dpy);
}

// Decode "%41" style escapes in place.
std::string percentDecode(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '%' && i + 2 < s.size()) {
            auto hex = [](char c) -> int {
                if (c >= '0' && c <= '9') return c - '0';
                if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                if (c >= 'A' && c <= 'F') return c - 'A' + 10;
                return -1;
            };
            int hi = hex(s[i + 1]), lo = hex(s[i + 2]);
            if (hi >= 0 && lo >= 0) {
                out += static_cast<char>(hi * 16 + lo);
                i += 2;
                continue;
            }
        }
        out += s[i];
    }
    return out;
}

// First file:// URI of a text/uri-list payload -> local byte path.
std::string uriListToPath(const std::string& data) {
    size_t pos = 0;
    while (pos < data.size()) {
        size_t eol = data.find_first_of("\r\n", pos);
        if (eol == std::string::npos) eol = data.size();
        std::string line = data.substr(pos, eol - pos);
        pos = data.find_first_not_of("\r\n", eol);
        if (pos == std::string::npos) pos = data.size();

        if (line.empty() || line[0] == '#') continue;
        if (line.compare(0, 7, "file://") == 0) {
            std::string rest = line.substr(7);
            // Skip an optional hostname up to the next '/'.
            if (!rest.empty() && rest[0] != '/') {
                size_t slash = rest.find('/');
                if (slash == std::string::npos) continue;
                rest = rest.substr(slash);
            }
            return percentDecode(rest);
        }
        if (!line.empty() && line[0] == '/') return percentDecode(line);
    }
    return std::string();
}

// Run a command line, capture the first line of stdout (without '\n').
// Returns the command exit code (127 = not found, per /bin/sh), or -1 if it
// could not be run at all.
int runCommandFirstLine(const std::string& cmd, std::string& out) {
    out.clear();
    FILE* p = popen(cmd.c_str(), "r");
    if (!p) return -1;
    char buf[4096];
    if (fgets(buf, sizeof(buf), p)) out = buf;
    while (fgets(buf, sizeof(buf), p)) {}   // drain
    int status = pclose(p);
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r'))
        out.pop_back();
    if (status == -1 || !WIFEXITED(status)) return -1;
    return WEXITSTATUS(status);
}

// Escape a string for safe single-quoted use in /bin/sh.
std::string shellQuote(const std::string& s) {
    std::string out = "'";
    for (char c : s) {
        if (c == '\'') out += "'\\''";
        else out += c;
    }
    out += "'";
    return out;
}

// --- XDND (drag & drop) ------------------------------------------------------

void xdndReset() {
    g_xdndSource = 0;
    g_xdndVersion = 0;
    g_xdndHaveUriList = false;
}

void xdndHandleEnter(const XClientMessageEvent& m) {
    xdndReset();
    g_xdndSource = static_cast<Window>(m.data.l[0]);
    g_xdndVersion = static_cast<int>((static_cast<unsigned long>(m.data.l[1])
                                      >> 24) & 0xFF);
    if (g_xdndVersion > 5) g_xdndVersion = 5;

    if (m.data.l[1] & 1) {
        // More than three types: read the XdndTypeList property.
        Atom actualType = 0;
        int actualFormat = 0;
        unsigned long count = 0, remaining = 0;
        unsigned char* data = nullptr;
        if (XGetWindowProperty(g_dpy, g_xdndSource, g_xdndTypeList, 0, 65536,
                               False, XA_ATOM, &actualType, &actualFormat,
                               &count, &remaining, &data) == Success && data) {
            if (actualType == XA_ATOM && actualFormat == 32) {
                const Atom* atoms = reinterpret_cast<const Atom*>(data);
                for (unsigned long i = 0; i < count; ++i)
                    if (atoms[i] == g_textUriList) g_xdndHaveUriList = true;
            }
            XFree(data);
        }
    } else {
        for (int i = 2; i <= 4; ++i)
            if (static_cast<Atom>(m.data.l[i]) == g_textUriList)
                g_xdndHaveUriList = true;
    }
}

void xdndHandlePosition(const XClientMessageEvent& m) {
    Window src = static_cast<Window>(m.data.l[0]);
    // XdndStatus: accept (bit 0) iff a usable type was offered; empty
    // rectangle (send position updates continuously); action copy.
    long accept = g_xdndHaveUriList ? 1 : 0;
    sendClientMessage(src, g_xdndStatus,
                      static_cast<long>(g_win), accept, 0, 0,
                      static_cast<long>(g_xdndActionCopy));
}

void xdndHandleDrop(const XClientMessageEvent& m) {
    Window src = static_cast<Window>(m.data.l[0]);
    if (!g_xdndHaveUriList) {
        sendClientMessage(src, g_xdndFinished, static_cast<long>(g_win), 0, 0,
                          0, 0);
        xdndReset();
        return;
    }
    Time t = (g_xdndVersion >= 1) ? static_cast<Time>(m.data.l[2])
                                  : CurrentTime;
    // The payload arrives via SelectionNotify on g_dndTargetProp.
    XConvertSelection(g_dpy, g_xdndSelection, g_textUriList, g_dndTargetProp,
                      g_win, t);
}

void xdndHandleSelectionNotify(const XSelectionEvent& se) {
    std::string payload;
    if (se.property == g_dndTargetProp && se.property != 0) {
        Atom actualType = 0;
        int actualFormat = 0;
        unsigned long count = 0, remaining = 0;
        unsigned char* data = nullptr;
        if (XGetWindowProperty(g_dpy, g_win, g_dndTargetProp, 0, 1 << 20,
                               True /* delete */, AnyPropertyType,
                               &actualType, &actualFormat, &count, &remaining,
                               &data) == Success && data) {
            if (actualFormat == 8 && count > 0)
                payload.assign(reinterpret_cast<const char*>(data), count);
            XFree(data);
        }
    }

    if (g_xdndSource) {
        long ok = payload.empty() ? 0 : 1;
        sendClientMessage(g_xdndSource, g_xdndFinished,
                          static_cast<long>(g_win), ok,
                          ok ? static_cast<long>(g_xdndActionCopy) : 0, 0, 0);
    }
    xdndReset();

    if (!payload.empty()) {
        std::string path = uriListToPath(payload);
        if (!path.empty() && g_cb.onDropFile) {
            platform::FileRef f;
            f.path = path;      // Linux paths are already plain bytes (UTF-8)
            f.display = path;
            g_cb.onDropFile(f);
        }
    }
}

// --- event handling ----------------------------------------------------------

void handleKeyPress(XKeyEvent& ke) {
    KeySym ks = XLookupKeysym(&ke, 0);
    platform::Key k = keysymToKey(ks);
    if (k == platform::Key::Unknown) return;
    bool repeat = keyDownSlot(k);   // detectable auto-repeat: press w/o release
    keyDownSlot(k) = true;
    if (g_cb.onKey) g_cb.onKey(k, true, stateToMods(ke.state), repeat);
}

void handleKeyRelease(XKeyEvent& ke) {
    // Without detectable auto-repeat, a repeat shows up as Release+Press with
    // the same timestamp and keycode: swallow the release and flag the press.
    if (XPending(g_dpy)) {
        XEvent next;
        XPeekEvent(g_dpy, &next);
        if (next.type == KeyPress &&
            next.xkey.keycode == ke.keycode &&
            next.xkey.time == ke.time) {
            XNextEvent(g_dpy, &next);   // consume the synthetic press
            KeySym ks = XLookupKeysym(&next.xkey, 0);
            platform::Key k = keysymToKey(ks);
            if (k != platform::Key::Unknown && g_cb.onKey)
                g_cb.onKey(k, true, stateToMods(next.xkey.state), true);
            return;
        }
    }
    KeySym ks = XLookupKeysym(&ke, 0);
    platform::Key k = keysymToKey(ks);
    if (k == platform::Key::Unknown) return;
    keyDownSlot(k) = false;
    if (g_cb.onKey) g_cb.onKey(k, false, stateToMods(ke.state), false);
}

void handleButtonPress(XButtonEvent& be) {
    switch (be.button) {
    case Button1: {
        // Synthesize a double click from two quick, nearby presses.
        int dx = be.x - g_lastClickX, dy = be.y - g_lastClickY;
        bool dbl = g_lastClickTime != 0 &&
                   (be.time - g_lastClickTime) < 400 &&
                   dx > -4 && dx < 4 && dy > -4 && dy < 4;
        if (dbl) {
            g_lastClickTime = 0;
            if (g_cb.onDoubleClick) g_cb.onDoubleClick(be.x, be.y);
        } else {
            g_lastClickTime = be.time;
            g_lastClickX = be.x;
            g_lastClickY = be.y;
            if (g_cb.onMouseButton)
                g_cb.onMouseButton(platform::MouseButton::Left, true,
                                   be.x, be.y);
        }
        break;
    }
    case Button2:
        if (g_cb.onMouseButton)
            g_cb.onMouseButton(platform::MouseButton::Middle, true,
                               be.x, be.y);
        break;
    case Button3:
        if (g_cb.onMouseButton)
            g_cb.onMouseButton(platform::MouseButton::Right, true,
                               be.x, be.y);
        break;
    case Button4:
        if (g_cb.onMouseWheel) g_cb.onMouseWheel(1.f);
        break;
    case Button5:
        if (g_cb.onMouseWheel) g_cb.onMouseWheel(-1.f);
        break;
    default:
        break;
    }
}

void handleButtonRelease(XButtonEvent& be) {
    platform::MouseButton b;
    switch (be.button) {
    case Button1: b = platform::MouseButton::Left; break;
    case Button2: b = platform::MouseButton::Middle; break;
    case Button3: b = platform::MouseButton::Right; break;
    default: return;   // wheel "releases" are ignored
    }
    if (g_cb.onMouseButton) g_cb.onMouseButton(b, false, be.x, be.y);
}

void handleEvent(XEvent& ev) {
    switch (ev.type) {
    case Expose:
        if (ev.xexpose.count == 0) g_dirty = true;
        break;
    case ConfigureNotify:
        if (ev.xconfigure.width != g_width ||
            ev.xconfigure.height != g_height) {
            g_width = ev.xconfigure.width;
            g_height = ev.xconfigure.height;
            if (g_cb.onResize) g_cb.onResize(g_width, g_height);
            g_dirty = true;
        }
        break;
    case MotionNotify:
        if (g_cb.onMouseMove) g_cb.onMouseMove(ev.xmotion.x, ev.xmotion.y);
        break;
    case ButtonPress:
        handleButtonPress(ev.xbutton);
        break;
    case ButtonRelease:
        handleButtonRelease(ev.xbutton);
        break;
    case KeyPress:
        handleKeyPress(ev.xkey);
        break;
    case KeyRelease:
        handleKeyRelease(ev.xkey);
        break;
    case SelectionNotify:
        if (ev.xselection.selection == g_xdndSelection)
            xdndHandleSelectionNotify(ev.xselection);
        break;
    case ClientMessage:
        if (ev.xclient.message_type == g_wmProtocols &&
            static_cast<Atom>(ev.xclient.data.l[0]) == g_wmDeleteWindow) {
            g_running = false;
        } else if (ev.xclient.message_type == g_xdndEnter) {
            xdndHandleEnter(ev.xclient);
        } else if (ev.xclient.message_type == g_xdndPosition) {
            xdndHandlePosition(ev.xclient);
        } else if (ev.xclient.message_type == g_xdndDrop) {
            xdndHandleDrop(ev.xclient);
        } else if (ev.xclient.message_type == g_xdndLeave) {
            xdndReset();
        }
        break;
    case MappingNotify:
        XRefreshKeyboardMapping(&ev.xmapping);
        break;
    default:
        break;
    }
}

// Temporary X error trap around glXCreateContextAttribsARB (it raises X
// errors, not just null returns, when the version/profile is unsupported).
bool s_ctxErrorFired = false;
int ctxErrorHandler(Display* dpy, XErrorEvent* ev) {
    (void)dpy;
    (void)ev;
    s_ctxErrorFired = true;
    return 0;
}

} // namespace

namespace platform {

// --- window ------------------------------------------------------------------

bool createWindow(const char* titleUtf8, int width, int height,
                  const Callbacks& cb) {
    g_cb = cb;

    g_dpy = XOpenDisplay(nullptr);
    if (!g_dpy) {
        std::fprintf(stderr, "3dt: cannot open X display\n");
        return false;
    }

    // Detectable auto-repeat: a held key sends KeyPress only (no synthetic
    // KeyRelease), which keeps isKeyDown() and the repeat flag simple.
    Bool supported = False;
    XkbSetDetectableAutoRepeat(g_dpy, True, &supported);

    // FBConfig with MSAA 8x -> 4x -> none. Chosen at window-creation time
    // because the window visual must match the future GL context.
    const int samplesTry[3] = {8, 4, 0};
    GLXFBConfig chosen = nullptr;
    for (int s : samplesTry) {
        int attribs[] = {
            GLX_X_RENDERABLE,  True,
            GLX_DRAWABLE_TYPE, GLX_WINDOW_BIT,
            GLX_RENDER_TYPE,   GLX_RGBA_BIT,
            GLX_X_VISUAL_TYPE, GLX_TRUE_COLOR,
            GLX_RED_SIZE,      8,
            GLX_GREEN_SIZE,    8,
            GLX_BLUE_SIZE,     8,
            GLX_DEPTH_SIZE,    24,
            GLX_STENCIL_SIZE,  8,
            GLX_DOUBLEBUFFER,  True,
            GLX_SAMPLE_BUFFERS, (s > 0) ? 1 : 0,
            GLX_SAMPLES,        s,
            None
        };
        int count = 0;
        GLXFBConfig* configs =
            glXChooseFBConfig(g_dpy, DefaultScreen(g_dpy), attribs, &count);
        if (configs) {
            if (count > 0) {
                chosen = configs[0];
                g_fbSamples = s;
            }
            XFree(configs);
        }
        if (chosen) break;
    }
    if (!chosen) {
        std::fprintf(stderr, "3dt: no usable GLX framebuffer config\n");
        XCloseDisplay(g_dpy);
        g_dpy = nullptr;
        return false;
    }
    g_fbc = chosen;

    XVisualInfo* vi = glXGetVisualFromFBConfig(g_dpy, g_fbc);
    if (!vi) {
        std::fprintf(stderr, "3dt: no visual for GLX framebuffer config\n");
        XCloseDisplay(g_dpy);
        g_dpy = nullptr;
        return false;
    }

    Window root = RootWindow(g_dpy, vi->screen);
    g_cmap = XCreateColormap(g_dpy, root, vi->visual, AllocNone);

    XSetWindowAttributes swa;
    std::memset(&swa, 0, sizeof(swa));
    swa.colormap = g_cmap;
    swa.event_mask = ExposureMask | StructureNotifyMask |
                     KeyPressMask | KeyReleaseMask |
                     ButtonPressMask | ButtonReleaseMask | PointerMotionMask;

    g_win = XCreateWindow(g_dpy, root, 0, 0,
                          static_cast<unsigned int>(width),
                          static_cast<unsigned int>(height),
                          0, vi->depth, InputOutput, vi->visual,
                          CWColormap | CWEventMask, &swa);
    XFree(vi);
    if (!g_win) {
        XCloseDisplay(g_dpy);
        g_dpy = nullptr;
        return false;
    }
    g_width = width;
    g_height = height;

    // Atoms.
    g_wmProtocols = XInternAtom(g_dpy, "WM_PROTOCOLS", False);
    g_wmDeleteWindow = XInternAtom(g_dpy, "WM_DELETE_WINDOW", False);
    g_netWmName = XInternAtom(g_dpy, "_NET_WM_NAME", False);
    g_utf8String = XInternAtom(g_dpy, "UTF8_STRING", False);
    g_netWmState = XInternAtom(g_dpy, "_NET_WM_STATE", False);
    g_netWmStateFullscreen =
        XInternAtom(g_dpy, "_NET_WM_STATE_FULLSCREEN", False);
    g_xdndAware = XInternAtom(g_dpy, "XdndAware", False);
    g_xdndEnter = XInternAtom(g_dpy, "XdndEnter", False);
    g_xdndPosition = XInternAtom(g_dpy, "XdndPosition", False);
    g_xdndStatus = XInternAtom(g_dpy, "XdndStatus", False);
    g_xdndLeave = XInternAtom(g_dpy, "XdndLeave", False);
    g_xdndDrop = XInternAtom(g_dpy, "XdndDrop", False);
    g_xdndFinished = XInternAtom(g_dpy, "XdndFinished", False);
    g_xdndSelection = XInternAtom(g_dpy, "XdndSelection", False);
    g_xdndTypeList = XInternAtom(g_dpy, "XdndTypeList", False);
    g_xdndActionCopy = XInternAtom(g_dpy, "XdndActionCopy", False);
    g_textUriList = XInternAtom(g_dpy, "text/uri-list", False);
    g_dndTargetProp = XInternAtom(g_dpy, "_3DT_DND_TARGET", False);

    XSetWMProtocols(g_dpy, g_win, &g_wmDeleteWindow, 1);

    // Announce XDND v5 support.
    unsigned long xdndVersion = 5;
    XChangeProperty(g_dpy, g_win, g_xdndAware, XA_ATOM, 32, PropModeReplace,
                    reinterpret_cast<unsigned char*>(&xdndVersion), 1);

    setWindowTitle(titleUtf8);
    return true;
}

void showWindow() {
    if (!g_dpy || !g_win) return;
    XMapWindow(g_dpy, g_win);
    XFlush(g_dpy);
}

void setWindowTitle(const char* titleUtf8) {
    if (!g_dpy || !g_win) return;
    const char* t = titleUtf8 ? titleUtf8 : "";
    XStoreName(g_dpy, g_win, t);
    XChangeProperty(g_dpy, g_win, g_netWmName, g_utf8String, 8,
                    PropModeReplace,
                    reinterpret_cast<const unsigned char*>(t),
                    static_cast<int>(std::strlen(t)));
    XFlush(g_dpy);
}

void getClientSize(int& w, int& h) {
    w = g_width;
    h = g_height;
    if (!g_dpy || !g_win) return;
    XWindowAttributes wa;
    if (XGetWindowAttributes(g_dpy, g_win, &wa)) {
        w = wa.width;
        h = wa.height;
    }
}

void toggleFullscreen() {
    if (!g_dpy || !g_win) return;
    // EWMH: ask the window manager to toggle _NET_WM_STATE_FULLSCREEN.
    XEvent e;
    std::memset(&e, 0, sizeof(e));
    e.xclient.type = ClientMessage;
    e.xclient.window = g_win;
    e.xclient.message_type = g_netWmState;
    e.xclient.format = 32;
    e.xclient.data.l[0] = 2;   // _NET_WM_STATE_TOGGLE
    e.xclient.data.l[1] = static_cast<long>(g_netWmStateFullscreen);
    e.xclient.data.l[2] = 0;
    e.xclient.data.l[3] = 1;   // source: normal application
    e.xclient.data.l[4] = 0;
    XSendEvent(g_dpy, RootWindow(g_dpy, DefaultScreen(g_dpy)), False,
               SubstructureRedirectMask | SubstructureNotifyMask, &e);
    XFlush(g_dpy);
    // The WM resizes the window; ConfigureNotify drives onResize + redraw.
}

void requestRedraw() {
    g_dirty = true;
}

void captureMouse(bool on) {
    // X11 grabs the pointer implicitly during a button drag; nothing to do.
    (void)on;
}

bool isKeyDown(Key k) {
    return keyDownSlot(k);
}

// --- OpenGL context ----------------------------------------------------------

bool createGLContext(GLContextInfo& out) {
    if (!g_dpy || !g_win || !g_fbc) return false;

    // 3.3 core profile via GLX_ARB_create_context, trapping X errors.
    GlXCreateContextAttribsARBProc createCtx =
        glxProc<GlXCreateContextAttribsARBProc>("glXCreateContextAttribsARB");
    if (createCtx) {
        const int ctxAttribs[] = {
            GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
            GLX_CONTEXT_MINOR_VERSION_ARB, 3,
            GLX_CONTEXT_PROFILE_MASK_ARB, GLX_CONTEXT_CORE_PROFILE_BIT_ARB,
            None
        };
        s_ctxErrorFired = false;
        int (*oldHandler)(Display*, XErrorEvent*) =
            XSetErrorHandler(ctxErrorHandler);
        g_ctx = createCtx(g_dpy, g_fbc, nullptr, True, ctxAttribs);
        XSync(g_dpy, False);
        XSetErrorHandler(oldHandler);
        if (s_ctxErrorFired && g_ctx) {
            glXDestroyContext(g_dpy, g_ctx);
            g_ctx = nullptr;
        }
        if (s_ctxErrorFired) g_ctx = nullptr;
    }
    g_ctxModern = (g_ctx != nullptr);

    if (!g_ctx) {
        // Legacy fallback (gives the highest compatibility-profile version).
        g_ctx = glXCreateNewContext(g_dpy, g_fbc, GLX_RGBA_TYPE, nullptr,
                                    True);
        if (!g_ctx) return false;
    }

    if (!glXMakeCurrent(g_dpy, g_win, g_ctx)) {
        glXDestroyContext(g_dpy, g_ctx);
        g_ctx = nullptr;
        return false;
    }

    out.modern = g_ctxModern;
    out.msaaSamples = g_ctxModern ? g_fbSamples : 0;
    return true;
}

bool recreateLegacyGLContext() {
    if (!g_dpy || !g_win || !g_fbc) return false;
    if (g_ctx) {
        glXMakeCurrent(g_dpy, None, nullptr);
        glXDestroyContext(g_dpy, g_ctx);
        g_ctx = nullptr;
    }
    g_ctxModern = false;
    g_ctx = glXCreateNewContext(g_dpy, g_fbc, GLX_RGBA_TYPE, nullptr, True);
    if (!g_ctx) return false;
    if (!glXMakeCurrent(g_dpy, g_win, g_ctx)) {
        glXDestroyContext(g_dpy, g_ctx);
        g_ctx = nullptr;
        return false;
    }
    return true;
}

void destroyGLContext() {
    if (g_dpy && g_ctx) {
        glXMakeCurrent(g_dpy, None, nullptr);
        glXDestroyContext(g_dpy, g_ctx);
        g_ctx = nullptr;
    }
}

void makeContextCurrent() {
    if (g_dpy && g_win && g_ctx) glXMakeCurrent(g_dpy, g_win, g_ctx);
}

void swapBuffers() {
    if (g_dpy && g_win) glXSwapBuffers(g_dpy, g_win);
}

void* getGLProcAddress(const char* name) {
    return glxProc<void*>(name);
}

void setSwapInterval(int interval) {
    if (!g_dpy || !g_win) return;
    GlXSwapIntervalEXTProc ext =
        glxProc<GlXSwapIntervalEXTProc>("glXSwapIntervalEXT");
    if (ext) {
        ext(g_dpy, g_win, interval);
        return;
    }
    GlXSwapIntervalMESAProc mesa =
        glxProc<GlXSwapIntervalMESAProc>("glXSwapIntervalMESA");
    if (mesa) mesa(static_cast<unsigned int>(interval));
}

// --- services ----------------------------------------------------------------

bool openFileDialog(FileRef& out) {
    // zenity, then kdialog; both print the chosen path on stdout. Exit code
    // 127 ("command not found" from /bin/sh) selects the next candidate;
    // any other non-zero code means the user cancelled.
    std::string path;
    int rc = runCommandFirstLine(
        "zenity --file-selection --title='Open 3D model' "
        "--file-filter='3D models (stl step stp obj) | "
        "*.stl *.step *.stp *.obj *.STL *.STEP *.STP *.OBJ' "
        "--file-filter='All files | *' 2>/dev/null",
        path);
    if (rc == 127 || rc == -1) {
        rc = runCommandFirstLine(
            "kdialog --getopenfilename . "
            "'*.stl *.step *.stp *.obj|3D models' 2>/dev/null",
            path);
    }
    if (rc != 0 || path.empty()) return false;  // cancelled or no dialog tool
    out.path = path;
    out.display = path;
    return true;
}

void showMessageBox(const char* title, const char* textUtf8, bool isError) {
    const char* t = title ? title : "";
    const char* m = textUtf8 ? textUtf8 : "";
    std::string ignored;
    std::string cmd = std::string("zenity ") +
                      (isError ? "--error" : "--warning") +
                      " --title=" + shellQuote(t) +
                      " --text=" + shellQuote(m) + " 2>/dev/null";
    int rc = runCommandFirstLine(cmd, ignored);
    if (rc == 0) return;
    if (rc == 127 || rc == -1) {
        std::string cmd2 = std::string("kdialog ") +
                           (isError ? "--error " : "--sorry ") +
                           shellQuote(m) + " --title " + shellQuote(t) +
                           " 2>/dev/null";
        if (runCommandFirstLine(cmd2, ignored) == 0) return;
    }
    std::fprintf(stderr, "3dt %s: %s\n", isError ? "error" : "warning", m);
}

// Expand the embedded 5x7 font into 8x16 cells (rows doubled, 1px margins),
// mirroring the layout contract of the Windows GDI rasterizer: glyph i in
// cell i with a 1px padding offset, 0/255 coverage.
bool rasterizeFontAtlas(int firstChar, int lastChar, int cols, int rows,
                        FontAtlas& out) {
    if (firstChar < 32 || lastChar > 126 || firstChar > lastChar) return false;
    if (cols <= 0 || rows <= 0) return false;
    if ((lastChar - firstChar + 1) > cols * rows) return false;

    const int glyphW = 8, glyphH = 16;
    out.advance = glyphW;
    out.cellW = glyphW + 2;   // 1px padding on each side
    out.cellH = glyphH + 2;
    out.width = out.cellW * cols;
    out.height = out.cellH * rows;
    out.coverage.assign(
        static_cast<size_t>(out.width) * static_cast<size_t>(out.height), 0);

    for (int c = firstChar; c <= lastChar; ++c) {
        int i = c - firstChar;
        int cellX = (i % cols) * out.cellW + 1;   // glyph at +1,+1 in cell
        int cellY = (i / cols) * out.cellH + 1;
        const unsigned char* rowsBits = kFont5x7[c - 32];
        for (int r = 0; r < 7; ++r) {
            unsigned char bits = rowsBits[r];
            for (int x = 0; x < 5; ++x) {
                if (!(bits & (0x10 >> x))) continue;
                // 5x7 -> 8x16: x offset 1 (5px wide in an 8px box), each
                // row doubled starting at y offset 1.
                int px = cellX + 1 + x;
                int py = cellY + 1 + r * 2;
                out.coverage[static_cast<size_t>(py) * out.width + px] = 255;
                out.coverage[static_cast<size_t>(py + 1) * out.width + px] =
                    255;
            }
        }
    }
    return true;
}

void debugLog(const char* msg) {
    if (msg) std::fputs(msg, stderr);
}

// --- event loop --------------------------------------------------------------

int runEventLoop() {
    if (!g_dpy || !g_win) return 1;
    g_running = true;
    while (g_running) {
        // Block for the next event, then drain the queue before drawing so
        // a burst of motion events yields a single frame.
        XEvent ev;
        XNextEvent(g_dpy, &ev);
        handleEvent(ev);
        while (g_running && XPending(g_dpy)) {
            XNextEvent(g_dpy, &ev);
            handleEvent(ev);
        }
        if (g_running && g_dirty) {
            g_dirty = false;
            if (g_cb.onDraw) g_cb.onDraw();
        }
    }

    if (g_cb.onClose) g_cb.onClose();   // releases the GL context
    if (g_win) {
        XDestroyWindow(g_dpy, g_win);
        g_win = 0;
    }
    if (g_cmap) {
        XFreeColormap(g_dpy, g_cmap);
        g_cmap = 0;
    }
    XCloseDisplay(g_dpy);
    g_dpy = nullptr;
    return 0;
}

void quit() {
    g_running = false;
}

} // namespace platform

// --- entry point -------------------------------------------------------------

int main(int argc, char** argv) {
    std::vector<platform::FileRef> files;
    for (int i = 1; i < argc; ++i) {
        platform::FileRef f;
        f.path = argv[i];      // byte paths, passed through untouched
        f.display = argv[i];
        files.push_back(f);
    }
    return appMain(files);
}

#endif // __linux__
