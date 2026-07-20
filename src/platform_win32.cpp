/*
 * platform_win32.cpp
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

// Win32 implementation of platform.h: window + message loop, WGL context
// bootstrap (dummy window to fetch wglChoosePixelFormatARB /
// wglCreateContextAttribsARB, then MSAA pixel format and a 3.3 core context
// with a legacy fallback), GDI glyph-atlas rasterization for the overlay,
// drag&drop, GetOpenFileNameW dialog, and the wWinMain entry point.
//
// Rendering is on-demand: requestRedraw() calls InvalidateRect and WM_PAINT
// invokes the app's onDraw callback (no busy loop).

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <windowsx.h>
#include <commdlg.h>
#include <shellapi.h>

#include <GL/gl.h>
#include <GL/wglext.h>

#include <cstring>
#include <string>
#include <vector>

#include "platform.h"

namespace {

const wchar_t* kWndClass = L"3dtViewerWindow";

HINSTANCE g_hinstance = nullptr;
int g_nCmdShow = SW_SHOW;
HWND g_hwnd = nullptr;
HDC g_hdc = nullptr;
HGLRC g_hglrc = nullptr;
platform::Callbacks g_cb;

// Fullscreen toggle state (F11): saved windowed style and placement.
bool g_fullscreen = false;
LONG g_savedStyle = 0;
WINDOWPLACEMENT g_savedPlacement = {sizeof(WINDOWPLACEMENT), 0, 0,
                                    {0, 0}, {0, 0}, {0, 0, 0, 0}};

// --- string helpers ----------------------------------------------------------

// The loaders take std::string and open files with std::ifstream, which on
// MinGW ends up in the CRT ANSI fopen. So the byte path must be encoded in
// the active ANSI code page (CP_ACP), not UTF-8. If the path contains
// characters that CP_ACP cannot represent, we fall back to the DOS 8.3 short
// name of the file, which is always ANSI-representable when available.
std::string widePathToLoaderPath(const std::wstring& w) {
    if (w.empty()) return std::string();

    auto convert = [](const std::wstring& in, BOOL* lossy) -> std::string {
        int n = WideCharToMultiByte(CP_ACP, WC_NO_BEST_FIT_CHARS, in.c_str(), -1,
                                    nullptr, 0, nullptr, lossy);
        if (n <= 0) return std::string();
        std::string s(static_cast<size_t>(n), '\0');
        WideCharToMultiByte(CP_ACP, WC_NO_BEST_FIT_CHARS, in.c_str(), -1,
                            &s[0], n, nullptr, lossy);
        s.resize(static_cast<size_t>(n) - 1); // drop terminating NUL
        return s;
    };

    BOOL lossy = FALSE;
    std::string s = convert(w, &lossy);
    if (!lossy && !s.empty()) return s;

    // Lossy: try the short (8.3) path instead.
    DWORD len = GetShortPathNameW(w.c_str(), nullptr, 0);
    if (len > 0) {
        std::wstring shortPath(len, L'\0');
        DWORD got = GetShortPathNameW(w.c_str(), &shortPath[0], len);
        if (got > 0 && got < len) {
            shortPath.resize(got);
            lossy = FALSE;
            std::string s2 = convert(shortPath, &lossy);
            if (!lossy && !s2.empty()) return s2;
        }
    }
    return s; // best effort; the loader will report open failure
}

std::wstring utf8ToWide(const std::string& s) {
    if (s.empty()) return std::wstring();
    int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
    if (n <= 0) return std::wstring();
    std::wstring w(static_cast<size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, &w[0], n);
    w.resize(static_cast<size_t>(n) - 1);
    return w;
}

std::string wideToUtf8(const std::wstring& w) {
    if (w.empty()) return std::string();
    int n = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0,
                                nullptr, nullptr);
    if (n <= 0) return std::string();
    std::string s(static_cast<size_t>(n), '\0');
    WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, &s[0], n, nullptr, nullptr);
    s.resize(static_cast<size_t>(n) - 1);
    return s;
}

platform::FileRef makeFileRef(const std::wstring& widePath) {
    platform::FileRef f;
    f.path = widePathToLoaderPath(widePath);
    f.display = wideToUtf8(widePath);
    return f;
}

// --- key mapping -------------------------------------------------------------

platform::Key vkToKey(WPARAM vk) {
    using platform::Key;
    switch (vk) {
    case 'W': return Key::W;
    case 'S': return Key::S;
    case 'G': return Key::G;
    case 'O': return Key::O;
    case 'F': return Key::F;
    case 'H': return Key::H;
    case 'I': return Key::I;
    case 'T': return Key::T;
    case 'X': return Key::X;
    case 'Y': return Key::Y;
    case 'Z': return Key::Z;
    case VK_F11:    return Key::F11;
    case VK_ESCAPE: return Key::Escape;
    default:        return Key::Unknown;
    }
}

int keyToVk(platform::Key k) {
    using platform::Key;
    switch (k) {
    case Key::W: return 'W';
    case Key::S: return 'S';
    case Key::G: return 'G';
    case Key::O: return 'O';
    case Key::F: return 'F';
    case Key::H: return 'H';
    case Key::I: return 'I';
    case Key::T: return 'T';
    case Key::X: return 'X';
    case Key::Y: return 'Y';
    case Key::Z: return 'Z';
    case Key::F11:    return VK_F11;
    case Key::Escape: return VK_ESCAPE;
    default:          return 0;
    }
}

unsigned currentMods() {
    unsigned mods = 0;
    if (GetKeyState(VK_CONTROL) & 0x8000) mods |= platform::ModCtrl;
    if (GetKeyState(VK_SHIFT) & 0x8000)   mods |= platform::ModShift;
    return mods;
}

// --- window procedure --------------------------------------------------------

LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_ERASEBKGND:
        return 1; // GL covers the whole client area

    case WM_PAINT: {
        PAINTSTRUCT ps;
        BeginPaint(hwnd, &ps);
        if (g_cb.onDraw) g_cb.onDraw();
        EndPaint(hwnd, &ps);
        return 0;
    }

    case WM_SIZE:
        if (g_cb.onResize) g_cb.onResize(LOWORD(lParam), HIWORD(lParam));
        InvalidateRect(hwnd, nullptr, FALSE);
        return 0;

    case WM_LBUTTONDOWN:
        if (g_cb.onMouseButton)
            g_cb.onMouseButton(platform::MouseButton::Left, true,
                               GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
        return 0;

    case WM_RBUTTONDOWN:
        if (g_cb.onMouseButton)
            g_cb.onMouseButton(platform::MouseButton::Right, true,
                               GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
        return 0;

    case WM_MBUTTONDOWN:
        if (g_cb.onMouseButton)
            g_cb.onMouseButton(platform::MouseButton::Middle, true,
                               GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
        return 0;

    case WM_LBUTTONUP:
        if (g_cb.onMouseButton)
            g_cb.onMouseButton(platform::MouseButton::Left, false,
                               GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
        return 0;

    case WM_RBUTTONUP:
        if (g_cb.onMouseButton)
            g_cb.onMouseButton(platform::MouseButton::Right, false,
                               GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
        return 0;

    case WM_MBUTTONUP:
        if (g_cb.onMouseButton)
            g_cb.onMouseButton(platform::MouseButton::Middle, false,
                               GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
        return 0;

    case WM_MOUSEMOVE:
        if (g_cb.onMouseMove)
            g_cb.onMouseMove(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
        return 0;

    case WM_MOUSEWHEEL:
        if (g_cb.onMouseWheel) {
            float steps = static_cast<float>(GET_WHEEL_DELTA_WPARAM(wParam)) /
                          static_cast<float>(WHEEL_DELTA);
            g_cb.onMouseWheel(steps);
        }
        return 0;

    case WM_LBUTTONDBLCLK:
        if (g_cb.onDoubleClick)
            g_cb.onDoubleClick(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
        return 0;

    case WM_KEYDOWN: {
        platform::Key k = vkToKey(wParam);
        if (k == platform::Key::Unknown) break;
        bool repeat = (lParam & (1 << 30)) != 0;
        if (g_cb.onKey) g_cb.onKey(k, true, currentMods(), repeat);
        return 0;
    }

    case WM_KEYUP: {
        platform::Key k = vkToKey(wParam);
        if (k == platform::Key::Unknown) break;
        if (g_cb.onKey) g_cb.onKey(k, false, currentMods(), false);
        return 0;
    }

    case WM_DROPFILES: {
        HDROP drop = reinterpret_cast<HDROP>(wParam);
        UINT len = DragQueryFileW(drop, 0, nullptr, 0);
        if (len > 0) {
            std::wstring path(static_cast<size_t>(len) + 1, L'\0');
            if (DragQueryFileW(drop, 0, &path[0], len + 1) > 0) {
                path.resize(len);
                if (g_cb.onDropFile) g_cb.onDropFile(makeFileRef(path));
            }
        }
        DragFinish(drop);
        return 0;
    }

    case WM_DESTROY:
        if (g_cb.onClose) g_cb.onClose();
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

// --- WGL extension bootstrap -------------------------------------------------

PFNWGLCHOOSEPIXELFORMATARBPROC    s_wglChoosePixelFormatARB = nullptr;
PFNWGLCREATECONTEXTATTRIBSARBPROC s_wglCreateContextAttribsARB = nullptr;
PFNWGLSWAPINTERVALEXTPROC         s_wglSwapIntervalEXT = nullptr;

// Cast through void* to silence -Wcast-function-type on PROC conversions.
template <typename T>
T wglProc(const char* name) {
    return reinterpret_cast<T>(
        reinterpret_cast<void*>(wglGetProcAddress(name)));
}

void fillLegacyPfd(PIXELFORMATDESCRIPTOR& pfd) {
    std::memset(&pfd, 0, sizeof(pfd));
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    pfd.cStencilBits = 8;
    pfd.iLayerType = PFD_MAIN_PLANE;
}

// Create a throwaway window + legacy context just to resolve the WGL
// extension entry points; everything is destroyed before returning.
void loadWglExtensions() {
    static bool done = false;
    if (done) return;
    done = true;

    HINSTANCE inst = GetModuleHandleW(nullptr);
    WNDCLASSW wc{};
    wc.style = CS_OWNDC;
    wc.lpfnWndProc = DefWindowProcW;
    wc.hInstance = inst;
    wc.lpszClassName = L"3dtGLBootstrap";
    if (!RegisterClassW(&wc)) return;

    HWND wnd = CreateWindowExW(0, wc.lpszClassName, L"", WS_OVERLAPPED,
                               0, 0, 32, 32, nullptr, nullptr, inst, nullptr);
    if (wnd) {
        HDC dc = GetDC(wnd);
        PIXELFORMATDESCRIPTOR pfd;
        fillLegacyPfd(pfd);
        int fmt = ChoosePixelFormat(dc, &pfd);
        if (fmt != 0 && SetPixelFormat(dc, fmt, &pfd)) {
            HGLRC rc = wglCreateContext(dc);
            if (rc && wglMakeCurrent(dc, rc)) {
                s_wglChoosePixelFormatARB =
                    wglProc<PFNWGLCHOOSEPIXELFORMATARBPROC>(
                        "wglChoosePixelFormatARB");
                s_wglCreateContextAttribsARB =
                    wglProc<PFNWGLCREATECONTEXTATTRIBSARBPROC>(
                        "wglCreateContextAttribsARB");
                wglMakeCurrent(nullptr, nullptr);
            }
            if (rc) wglDeleteContext(rc);
        }
        ReleaseDC(wnd, dc);
        DestroyWindow(wnd);
    }
    UnregisterClassW(wc.lpszClassName, inst);
}

// Modern context: MSAA pixel format (8x -> 4x -> none) + 3.3 core profile.
// On success the context is current and *samples holds the MSAA level.
bool createModernWglContext(int* samples) {
    if (!s_wglChoosePixelFormatARB || !s_wglCreateContextAttribsARB)
        return false;

    int fmt = 0;
    const int samplesTry[3] = {8, 4, 0};
    for (int s : samplesTry) {
        const int attribs[] = {
            WGL_DRAW_TO_WINDOW_ARB, GL_TRUE,
            WGL_SUPPORT_OPENGL_ARB, GL_TRUE,
            WGL_DOUBLE_BUFFER_ARB,  GL_TRUE,
            WGL_ACCELERATION_ARB,   WGL_FULL_ACCELERATION_ARB,
            WGL_PIXEL_TYPE_ARB,     WGL_TYPE_RGBA_ARB,
            WGL_COLOR_BITS_ARB,     32,
            WGL_DEPTH_BITS_ARB,     24,
            WGL_STENCIL_BITS_ARB,   8,
            WGL_SAMPLE_BUFFERS_ARB, (s > 0) ? 1 : 0,
            WGL_SAMPLES_ARB,        s,
            0
        };
        int f = 0;
        UINT count = 0;
        if (s_wglChoosePixelFormatARB(g_hdc, attribs, nullptr, 1, &f, &count) &&
            count > 0) {
            fmt = f;
            *samples = s;
            break;
        }
    }
    if (fmt == 0) return false;

    PIXELFORMATDESCRIPTOR pfd;
    if (!DescribePixelFormat(g_hdc, fmt, sizeof(pfd), &pfd)) return false;
    if (!SetPixelFormat(g_hdc, fmt, &pfd)) return false;

    const int ctxAttribs[] = {
        WGL_CONTEXT_MAJOR_VERSION_ARB, 3,
        WGL_CONTEXT_MINOR_VERSION_ARB, 3,
        WGL_CONTEXT_PROFILE_MASK_ARB,  WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
        0
    };
    g_hglrc = s_wglCreateContextAttribsARB(g_hdc, nullptr, ctxAttribs);
    if (!g_hglrc) return false;
    if (!wglMakeCurrent(g_hdc, g_hglrc)) {
        wglDeleteContext(g_hglrc);
        g_hglrc = nullptr;
        return false;
    }
    return true;
}

// Legacy context on the window DC. Reuses the pixel format if one is
// already set (SetPixelFormat is once-per-window), else picks a plain one.
bool createLegacyWglContext() {
    if (GetPixelFormat(g_hdc) == 0) {
        PIXELFORMATDESCRIPTOR pfd;
        fillLegacyPfd(pfd);
        int fmt = ChoosePixelFormat(g_hdc, &pfd);
        if (fmt == 0 || !SetPixelFormat(g_hdc, fmt, &pfd)) return false;
    }
    g_hglrc = wglCreateContext(g_hdc);
    if (!g_hglrc) return false;
    if (!wglMakeCurrent(g_hdc, g_hglrc)) {
        wglDeleteContext(g_hglrc);
        g_hglrc = nullptr;
        return false;
    }
    return true;
}

} // namespace

namespace platform {

// --- window ------------------------------------------------------------------

bool createWindow(const char* titleUtf8, int width, int height,
                  const Callbacks& cb) {
    g_cb = cb;
    if (!g_hinstance) g_hinstance = GetModuleHandleW(nullptr);

    WNDCLASSEXW wc{};
    wc.cbSize = sizeof(wc);
    // CS_OWNDC: keep one DC per window for the GL context;
    // CS_DBLCLKS: receive WM_LBUTTONDBLCLK (fit view).
    wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC | CS_DBLCLKS;
    wc.lpfnWndProc = WndProc;
    wc.hInstance = g_hinstance;
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    wc.hIcon = LoadIconW(nullptr, IDI_APPLICATION);
    wc.lpszClassName = kWndClass;
    if (!RegisterClassExW(&wc)) return false;

    std::wstring title = utf8ToWide(titleUtf8 ? titleUtf8 : "");
    g_hwnd = CreateWindowExW(0, kWndClass, title.c_str(), WS_OVERLAPPEDWINDOW,
                             CW_USEDEFAULT, CW_USEDEFAULT, width, height,
                             nullptr, nullptr, g_hinstance, nullptr);
    if (!g_hwnd) return false;

    DragAcceptFiles(g_hwnd, TRUE);
    return true;
}

void showWindow() {
    if (!g_hwnd) return;
    ShowWindow(g_hwnd, g_nCmdShow);
    UpdateWindow(g_hwnd);
}

void setWindowTitle(const char* titleUtf8) {
    if (!g_hwnd) return;
    std::wstring title = utf8ToWide(titleUtf8 ? titleUtf8 : "");
    SetWindowTextW(g_hwnd, title.c_str());
}

void getClientSize(int& w, int& h) {
    w = h = 0;
    if (!g_hwnd) return;
    RECT rc;
    if (GetClientRect(g_hwnd, &rc)) {
        w = rc.right - rc.left;
        h = rc.bottom - rc.top;
    }
}

void toggleFullscreen() {
    if (!g_hwnd) return;
    if (!g_fullscreen) {
        // Save the windowed style/placement, then go borderless on the
        // monitor the window currently occupies (multi-monitor friendly).
        g_savedStyle = GetWindowLongW(g_hwnd, GWL_STYLE);
        g_savedPlacement.length = sizeof(g_savedPlacement);
        if (!GetWindowPlacement(g_hwnd, &g_savedPlacement)) return;
        HMONITOR mon = MonitorFromWindow(g_hwnd, MONITOR_DEFAULTTONEAREST);
        MONITORINFO mi{};
        mi.cbSize = sizeof(mi);
        if (!GetMonitorInfoW(mon, &mi)) return;
        SetWindowLongW(g_hwnd, GWL_STYLE, WS_POPUP | WS_VISIBLE);
        SetWindowPos(g_hwnd, HWND_TOP,
                     mi.rcMonitor.left, mi.rcMonitor.top,
                     mi.rcMonitor.right - mi.rcMonitor.left,
                     mi.rcMonitor.bottom - mi.rcMonitor.top,
                     SWP_FRAMECHANGED | SWP_SHOWWINDOW);
        g_fullscreen = true;
    } else {
        // Restore the saved style and placement (placement, not a raw rect,
        // so the shell cannot second-guess the position of the ex-fullscreen
        // window).
        SetWindowLongW(g_hwnd, GWL_STYLE, g_savedStyle);
        SetWindowPlacement(g_hwnd, &g_savedPlacement);
        SetWindowPos(g_hwnd, nullptr, 0, 0, 0, 0,
                     SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER |
                     SWP_NOOWNERZORDER | SWP_FRAMECHANGED | SWP_SHOWWINDOW);
        g_fullscreen = false;
    }
    // WM_SIZE fires from SetWindowPos and triggers onResize + redraw.
}

void requestRedraw() {
    if (g_hwnd) InvalidateRect(g_hwnd, nullptr, FALSE);
}

void captureMouse(bool on) {
    if (!g_hwnd) return;
    if (on) SetCapture(g_hwnd);
    else ReleaseCapture();
}

bool isKeyDown(Key k) {
    int vk = keyToVk(k);
    return vk != 0 && (GetKeyState(vk) & 0x8000) != 0;
}

// --- OpenGL context ----------------------------------------------------------

bool createGLContext(GLContextInfo& out) {
    if (!g_hwnd) return false;
    g_hdc = GetDC(g_hwnd);
    if (!g_hdc) return false;

    loadWglExtensions();

    int samples = 0;
    if (createModernWglContext(&samples)) {
        out.modern = true;
        out.msaaSamples = samples;
        return true;
    }

    // NOTE: SetPixelFormat may already have succeeded on this DC and cannot
    // be changed; the legacy context reuses the same format in that case.
    if (!createLegacyWglContext()) {
        ReleaseDC(g_hwnd, g_hdc);
        g_hdc = nullptr;
        return false;
    }
    out.modern = false;
    out.msaaSamples = 0;
    return true;
}

bool recreateLegacyGLContext() {
    if (!g_hdc) return false;
    if (g_hglrc) {
        wglMakeCurrent(nullptr, nullptr);
        wglDeleteContext(g_hglrc);
        g_hglrc = nullptr;
    }
    return createLegacyWglContext();
}

void destroyGLContext() {
    if (g_hglrc) {
        wglMakeCurrent(nullptr, nullptr);
        wglDeleteContext(g_hglrc);
        g_hglrc = nullptr;
    }
    if (g_hdc) {
        ReleaseDC(g_hwnd, g_hdc);
        g_hdc = nullptr;
    }
}

void makeContextCurrent() {
    if (g_hdc && g_hglrc) wglMakeCurrent(g_hdc, g_hglrc);
}

void swapBuffers() {
    if (g_hdc) SwapBuffers(g_hdc);
}

void* getGLProcAddress(const char* name) {
    // wglGetProcAddress fails for GL 1.1 entry points and may return small
    // sentinel values on failure; fall back to the opengl32.dll export table.
    PROC p = wglGetProcAddress(name);
    intptr_t v = reinterpret_cast<intptr_t>(p);
    if (v == 0 || v == 1 || v == 2 || v == 3 || v == -1) {
        static HMODULE mod = LoadLibraryA("opengl32.dll");
        p = mod ? GetProcAddress(mod, name) : nullptr;
    }
    return reinterpret_cast<void*>(p);
}

void setSwapInterval(int interval) {
    if (!s_wglSwapIntervalEXT)
        s_wglSwapIntervalEXT = wglProc<PFNWGLSWAPINTERVALEXTPROC>(
            "wglSwapIntervalEXT");
    if (s_wglSwapIntervalEXT) s_wglSwapIntervalEXT(interval);
}

// --- services ----------------------------------------------------------------

bool openFileDialog(FileRef& out) {
    wchar_t buf[2048] = L"";
    OPENFILENAMEW ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = g_hwnd;
    ofn.lpstrFilter =
        L"3D models (*.stl;*.step;*.stp;*.obj)\0*.stl;*.step;*.stp;*.obj\0"
        L"All files (*.*)\0*.*\0";
    ofn.lpstrFile = buf;
    ofn.nMaxFile = static_cast<DWORD>(sizeof(buf) / sizeof(buf[0]));
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY |
                OFN_NOCHANGEDIR;
    if (!GetOpenFileNameW(&ofn)) return false;
    out = makeFileRef(buf);
    return true;
}

void showMessageBox(const char* title, const char* textUtf8, bool isError) {
    std::wstring t = utf8ToWide(title ? title : "");
    std::wstring m = utf8ToWide(textUtf8 ? textUtf8 : "");
    MessageBoxW(g_hwnd, m.c_str(), t.c_str(),
                MB_OK | (isError ? MB_ICONERROR : MB_ICONWARNING));
}

// Rasterize the glyph range with GDI (Consolas on a 32-bit DIB section,
// grayscale antialiasing) and extract one channel as 8-bit coverage.
bool rasterizeFontAtlas(int firstChar, int lastChar, int cols, int rows,
                        FontAtlas& out) {
    if (firstChar > lastChar || cols <= 0 || rows <= 0) return false;
    if ((lastChar - firstChar + 1) > cols * rows) return false;

    HDC dc = CreateCompatibleDC(nullptr);
    if (!dc) return false;

    HFONT font = CreateFontW(-16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                             DEFAULT_CHARSET, OUT_TT_PRECIS,
                             CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY,
                             FIXED_PITCH | FF_MODERN, L"Consolas");
    if (!font) {
        DeleteDC(dc);
        return false;
    }
    HGDIOBJ oldFont = SelectObject(dc, font);

    TEXTMETRICW tm{};
    GetTextMetricsW(dc, &tm);
    out.advance = tm.tmAveCharWidth;     // monospace advance
    out.cellW = tm.tmMaxCharWidth + 2;   // 1px padding on each side
    out.cellH = tm.tmHeight + 2;
    out.width = out.cellW * cols;
    out.height = out.cellH * rows;

    BITMAPINFO bmi{};
    bmi.bmiHeader.biSize = sizeof(bmi.bmiHeader);
    bmi.bmiHeader.biWidth = out.width;
    bmi.bmiHeader.biHeight = -out.height;   // top-down
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 32;
    bmi.bmiHeader.biCompression = BI_RGB;
    void* bits = nullptr;
    HBITMAP dib = CreateDIBSection(dc, &bmi, DIB_RGB_COLORS, &bits, nullptr, 0);
    if (!dib || !bits) {
        SelectObject(dc, oldFont);
        DeleteObject(font);
        if (dib) DeleteObject(dib);
        DeleteDC(dc);
        return false;
    }
    HGDIOBJ oldBmp = SelectObject(dc, dib);

    // White glyphs antialiased against the zeroed (black) DIB.
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, RGB(255, 255, 255));
    for (int c = firstChar; c <= lastChar; ++c) {
        int i = c - firstChar;
        wchar_t ch = static_cast<wchar_t>(c);
        TextOutW(dc, (i % cols) * out.cellW + 1, (i / cols) * out.cellH + 1,
                 &ch, 1);
    }
    GdiFlush();

    // Extract one channel as coverage.
    out.coverage.assign(
        static_cast<size_t>(out.width) * static_cast<size_t>(out.height), 0);
    const unsigned char* px = static_cast<const unsigned char*>(bits);
    for (size_t i = 0; i < out.coverage.size(); ++i)
        out.coverage[i] = px[i * 4 + 1];

    SelectObject(dc, oldBmp);
    SelectObject(dc, oldFont);
    DeleteObject(dib);
    DeleteObject(font);
    DeleteDC(dc);
    return true;
}

void debugLog(const char* msg) {
    OutputDebugStringA(msg);
}

// --- event loop --------------------------------------------------------------

int runEventLoop() {
    MSG m;
    while (GetMessageW(&m, nullptr, 0, 0) > 0) {
        TranslateMessage(&m);
        DispatchMessageW(&m);
    }
    return static_cast<int>(m.wParam);
}

void quit() {
    if (g_hwnd) {
        DestroyWindow(g_hwnd);
        g_hwnd = nullptr;
    }
}

} // namespace platform

// --- entry point -------------------------------------------------------------

int WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
                    PWSTR lpCmdLine, int nCmdShow) {
    (void)hPrevInstance;
    (void)lpCmdLine;
    g_hinstance = hInstance;
    g_nCmdShow = nCmdShow;

    // Optional file path(s) on the command line (may contain spaces/unicode).
    std::vector<platform::FileRef> files;
    int argc = 0;
    LPWSTR* argv = CommandLineToArgvW(GetCommandLineW(), &argc);
    if (argv) {
        for (int i = 1; i < argc; ++i) files.push_back(makeFileRef(argv[i]));
        LocalFree(argv);
    }
    return appMain(files);
}

#endif // _WIN32
