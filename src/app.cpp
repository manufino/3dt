/*
 * app.cpp
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

// 3dt - STL/STEP viewer: portable application logic.
//
// Talks only to platform.h (window, input, dialogs) and to the renderer:
// state, loader dispatch by extension, keyboard commands and camera input.
// Rendering is on-demand: input/loading calls platform::requestRedraw() and
// the platform invokes onDraw for one frame (no busy loop).

#include <string>
#include <vector>

#include "mesh.h"
#include "platform.h"
#include "renderer.h"

namespace {

const char* kBaseTitle = "3dt - STL/STEP/OBJ Viewer";

Renderer g_renderer;
Mesh g_mesh;
bool g_hasMesh = false;
std::string g_fileName;      // UTF-8 display name of the loaded file

int g_lastX = 0, g_lastY = 0;
bool g_orbiting = false;
bool g_panning = false;

// --- path helpers (on UTF-8 display paths; separators '/' and '\\') ----------

std::string lowerExtension(const std::string& path) {
    size_t dot = path.find_last_of('.');
    size_t sep = path.find_last_of("\\/");
    if (dot == std::string::npos ||
        (sep != std::string::npos && dot < sep)) return std::string();
    std::string ext = path.substr(dot + 1);
    for (char& c : ext) {
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }
    return ext;
}

std::string fileNameOf(const std::string& path) {
    size_t sep = path.find_last_of("\\/");
    return (sep == std::string::npos) ? path : path.substr(sep + 1);
}

// --- title / loading ---------------------------------------------------------

void updateTitle() {
    if (g_hasMesh) {
        std::string t = "3dt - " + g_fileName + " - " +
                        std::to_string(g_mesh.triangleCount()) + " tris";
        platform::setWindowTitle(t.c_str());
    } else {
        platform::setWindowTitle(kBaseTitle);
    }
}

void loadFile(const platform::FileRef& file) {
    std::string ext = lowerExtension(file.display);
    bool isStl = (ext == "stl");
    bool isStep = (ext == "step" || ext == "stp");
    bool isObj = (ext == "obj");
    if (!isStl && !isStep && !isObj) {
        platform::showMessageBox(
            "3dt", "Unsupported file type. Expected .stl, .step, .stp or .obj.",
            false);
        return;
    }

    // Loading big files can take seconds; show it in the title (synchronous).
    platform::setWindowTitle("3dt - Loading...");

    std::string err;
    Mesh mesh;
    bool ok = isStl   ? loadSTL(file.path, mesh, err)
              : isObj ? loadOBJ(file.path, mesh, err)
                      : loadSTEP(file.path, mesh, err);

    if (!ok) {
        updateTitle(); // restore previous title
        std::string msg = "Failed to load:\n" + file.display + "\n\n" + err;
        platform::showMessageBox("3dt - Load error", msg.c_str(), true);
        return;
    }

    mesh.computeBounds();
    mesh.computeNormalsIfMissing();

    g_mesh = std::move(mesh);
    g_hasMesh = true;
    g_fileName = fileNameOf(file.display);
    g_renderer.setMesh(&g_mesh);
    g_renderer.setModelName(g_fileName);
    g_renderer.fitToBounds();
    updateTitle();
    platform::requestRedraw();
}

void openFileDialog() {
    platform::FileRef file;
    if (platform::openFileDialog(file)) loadFile(file);
}

// --- input handlers ----------------------------------------------------------

void onMouseButton(platform::MouseButton b, bool down, int x, int y) {
    if (down && g_renderer.treeHitTest(x, y)) {
        // Clicks inside the tree panel never reach the camera.
        g_lastX = x;
        g_lastY = y;
        if (b == platform::MouseButton::Left && g_renderer.treeClick(x, y))
            platform::requestRedraw();
        return;
    }
    if (down) {
        if (b == platform::MouseButton::Left) g_orbiting = true;
        else g_panning = true;              // right or middle button pans
        g_lastX = x;
        g_lastY = y;
        platform::captureMouse(true);
    } else {
        if (b == platform::MouseButton::Left) {
            g_orbiting = false;
            if (!g_panning) platform::captureMouse(false);
        } else {
            g_panning = false;
            if (!g_orbiting) platform::captureMouse(false);
        }
    }
}

void onMouseMove(int x, int y) {
    float dx = static_cast<float>(x - g_lastX);
    float dy = static_cast<float>(y - g_lastY);
    g_lastX = x;
    g_lastY = y;
    if (g_orbiting) {
        g_renderer.orbit(dx, dy);
        platform::requestRedraw();
    } else if (g_panning) {
        g_renderer.pan(dx, dy);
        platform::requestRedraw();
    }
}

void onMouseWheel(float steps) {
    // Wheel over the tree panel scrolls the tree, not the camera.
    if (g_renderer.treeWheel(g_lastX, g_lastY, steps)) {
        platform::requestRedraw();
        return;
    }
    // Wheel while holding X/Y/Z moves the active section plane.
    int axis = platform::isKeyDown(platform::Key::X) ? 0
             : platform::isKeyDown(platform::Key::Y) ? 1
             : platform::isKeyDown(platform::Key::Z) ? 2 : -1;
    if (axis >= 0 && g_renderer.sectionEnabled(axis))
        g_renderer.moveSection(axis, steps);
    else
        g_renderer.zoom(steps);
    platform::requestRedraw();
}

void onDoubleClick(int x, int y) {
    // Double click on a tree row fits the view on that part.
    if (g_renderer.treeDoubleClick(x, y)) {
        platform::requestRedraw();
        return;
    }
    g_renderer.fitToBounds();
    platform::requestRedraw();
}

void onKey(platform::Key k, bool down, unsigned mods, bool repeat) {
    using platform::Key;

    if (!down) {
        if (k == Key::X || k == Key::Y || k == Key::Z) {
            g_renderer.setSectionHighlight(-1);
            platform::requestRedraw();
        }
        return;
    }

    switch (k) {
    case Key::O:
        if (mods & platform::ModCtrl) {
            openFileDialog();           // Ctrl+O: open file
        } else {
            g_renderer.toggleShadows(); // O: shadows on/off
            platform::requestRedraw();
        }
        break;
    case Key::W:
        g_renderer.toggleWireframe();
        platform::requestRedraw();
        break;
    case Key::S:
        g_renderer.toggleSmooth();
        platform::requestRedraw();
        break;
    case Key::G:
        g_renderer.toggleGrid();
        platform::requestRedraw();
        break;
    case Key::F:
        g_renderer.fitToBounds();
        platform::requestRedraw();
        break;
    case Key::H:
        g_renderer.toggleHelp();
        platform::requestRedraw();
        break;
    case Key::I:
        g_renderer.toggleInfo();
        platform::requestRedraw();
        break;
    case Key::T:
        // No-op unless the loaded model has a part tree.
        g_renderer.toggleTree();
        platform::requestRedraw();
        break;
    case Key::X:
    case Key::Y:
    case Key::Z: {
        if (repeat) break;              // ignore key auto-repeat
        int axis = (k == Key::X) ? 0 : (k == Key::Y) ? 1 : 2;
        if (mods & platform::ModShift)
            g_renderer.invertSection(axis);   // Shift: flip cut side
        else
            g_renderer.toggleSection(axis);   // plain: on/off
        // Show the plane outline while the key stays held.
        g_renderer.setSectionHighlight(
            g_renderer.sectionEnabled(axis) ? axis : -1);
        platform::requestRedraw();
        break;
    }
    case Key::F11:
        platform::toggleFullscreen();
        break;
    case Key::Escape:
        platform::quit();
        break;
    default:
        break;
    }
}

} // namespace

int appMain(const std::vector<platform::FileRef>& files) {
    platform::Callbacks cb;
    cb.onMouseMove = onMouseMove;
    cb.onMouseButton = onMouseButton;
    cb.onMouseWheel = onMouseWheel;
    cb.onDoubleClick = onDoubleClick;
    cb.onKey = onKey;
    cb.onResize = [](int w, int h) {
        g_renderer.resize(w, h);
        platform::requestRedraw();
    };
    cb.onDraw = []() { g_renderer.render(); };
    cb.onDropFile = [](const platform::FileRef& file) { loadFile(file); };
    cb.onClose = []() { g_renderer.shutdown(); };

    if (!platform::createWindow(kBaseTitle, 1280, 800, cb)) return 1;

    if (!g_renderer.init()) {
        platform::showMessageBox("3dt", "Failed to create the OpenGL context.",
                                 true);
        platform::quit();
        return 1;
    }
    int w = 0, h = 0;
    platform::getClientSize(w, h);
    g_renderer.resize(w, h);

    platform::showWindow();

    if (!files.empty()) loadFile(files[0]);

    return platform::runEventLoop();
}
