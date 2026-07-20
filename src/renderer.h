/*
 * renderer.h
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
// Renderer for the 3dt viewer.
//
// Primary path: OpenGL 3.3 core profile with MSAA (8x, then 4x, then none),
// GLSL Blinn-Phong shading (two directional lights, gamma-correct), a
// directional shadow map with 3x3 PCF, a "shadow catcher" ground plane, a
// gradient background and VBO/VAO geometry (one upload per mesh, one draw
// call per pass).
//
// Fallback path: if a 3.3 core context cannot be created, a legacy
// OpenGL 1.1 fixed-function context is used with the previous minimal
// rendering (no shadows; toggleShadows() then returns false and is a no-op).
//
// The renderer is fully platform neutral: context creation, buffer swapping
// and GL symbol lookup go through platform.h.

#include <string>
#include <utility>
#include <vector>

#include "overlay.h"

struct Mesh;

class Renderer {
public:
    // Create the GL context on the platform window (which must already
    // exist). Returns false on failure.
    bool init();
    void shutdown();

    // Store the framebuffer size (safe to call before init).
    void resize(int w, int h);

    // The mesh is not copied and not owned: it must outlive the renderer or
    // be replaced/cleared with setMesh(nullptr). bboxMin/bboxMax must be
    // valid (computeBounds already called). Vertex data is uploaded to a VBO
    // here (once), not per frame. Non-const: the tree panel stores its
    // per-node visibility state in MeshNode::visible.
    void setMesh(Mesh* mesh);

    // Display name of the loaded file (UTF-8), shown in the info panel.
    // Non-ASCII codepoints are replaced with '?' (the atlas is ASCII-only).
    void setModelName(const std::string& nameUtf8);

    // Place the camera so the whole bounding box is visible.
    void fitToBounds();

    // Camera controls; dx/dy are mouse deltas in pixels.
    void orbit(float dx, float dy);
    void pan(float dx, float dy);
    // wheelSteps: +1 per wheel notch forward (zoom in), exponential.
    void zoom(float wheelSteps);

    // Toggles; each returns the new state.
    bool toggleWireframe();
    bool toggleSmooth();
    bool toggleGrid();
    // Shadow mapping on/off (default on). On the legacy fallback context this
    // does nothing and always returns false.
    bool toggleShadows();

    // Overlay panels (modern path only; no-op visuals on the legacy
    // fallback). Help is visible by default, info hidden by default.
    bool toggleHelp();
    bool toggleInfo();

    // Model tree panel (multi-part models). toggleTree() is a no-op that
    // returns false while the mesh has no node tree; the panel itself is
    // drawn on the modern path only, but the per-node visibility filter
    // works on both paths. Visible by default.
    bool toggleTree();

    // Tree panel mouse interaction; client-area pixel coordinates, origin
    // top-left. Each returns true when the event landed inside the visible
    // panel and was consumed (it must then not reach the camera).
    bool treeHitTest(int x, int y) const;      // point over the panel?
    bool treeClick(int x, int y);              // expand/collapse or toggle
    bool treeWheel(int x, int y, float steps); // scroll the row list
    bool treeDoubleClick(int x, int y);        // fit view on the row's part

    // Axis-aligned section planes; axis: 0=X, 1=Y, 2=Z. Each plane can be
    // toggled independently, moved along its axis (clamped to the model
    // bbox, 2% of the extent per wheel step) and flipped to cut the other
    // side. The planes clip the model (and its shadow) but not grid/ground/
    // overlay.
    bool toggleSection(int axis);              // returns the new state
    void invertSection(int axis);
    void moveSection(int axis, float steps);   // wheel notches (+/-)
    bool sectionEnabled(int axis) const;
    // Axis whose plane outline is highlighted while the key is held
    // (-1 = none).
    void setSectionHighlight(int axis);

    // Draw the scene and swap buffers. No-op if init failed.
    void render();

private:
    // Context / resource setup (modern path).
    bool initModernResources();
    void destroyModernResources();
    bool createPrograms();
    bool createShadowTarget();
    void rebuildSceneBuffers();   // grid/axes + ground VBOs (per scene change)
    void uploadMesh();            // mesh VBO (per setMesh)

    // Frame rendering (modern path).
    void renderModern();
    void renderShadowPass(const float* lightVP, const float* clipPlanes);
    void computeLightVP(float* out) const;

    // Section plane helpers (planes: 3 x vec4, world space, keep dist >= 0).
    void computeClipPlanes(float planes[12]) const;
    void enableClipPlanes() const;
    void disableClipPlanes() const;
    void drawSectionHighlight(const float* viewProj);

    // Overlay panels (help / model info / model tree).
    void renderOverlay();
    std::string sectionSummary() const;

    // --- model tree panel ----------------------------------------------------
    // One displayed row of the tree (collapsed subtrees excluded).
    struct TreeRow {
        int node = -1;
        int depth = 0;
        bool hasChildren = false;
    };
    // Panel geometry shared by drawing and hit-testing (pixels, top-left).
    struct TreeLayout {
        bool valid = false;
        float x = 0.f, y = 0.f, w = 0.f, h = 0.f;  // panel rectangle
        float rowsY = 0.f;                         // top of first shown row
        float rowH = 0.f;
        float indent = 0.f;                        // extra x per depth level
        int first = 0;                             // scroll offset (rows)
        int shown = 0;                             // rows on screen
        std::vector<TreeRow> rows;                 // all expanded rows
    };
    bool treePanelVisible() const;   // panel currently on screen
    void buildChildren(std::vector<std::vector<int>>& kids) const;
    void buildTreeRows(std::vector<TreeRow>& rows) const;
    void computeTreeLayout(TreeLayout& out) const;
    // Per-node aggregated visibility: 0 = hidden, 1 = visible, 2 = mixed.
    void computeVisStates(std::vector<unsigned char>& st) const;
    void toggleNodeVisible(int node);  // recursive; keeps an undo snapshot
    void drawTreePanel();

    // Visible triangle ranges (merged, in vertices) used by all mesh passes.
    void rebuildDrawRanges();
    void drawMeshRanges() const;       // glDrawArrays over drawRanges_
    // Bbox of a node's subtree / of the visible part of the mesh.
    bool nodeBounds(int node, float mn[3], float mx[3]) const;
    bool visibleBounds(float mn[3], float mx[3]) const;
    void fitToBox(const float mn[3], const float mx[3]);

    // Legacy fixed-function path.
    void initLegacyState();
    void renderLegacy();
    void applyProjection() const;
    void applyLights() const;
    void applyView() const;
    void drawMeshLegacy() const;
    void drawGridAndAxesLegacy() const;

    bool ctxValid_ = false;   // a GL context exists (init() succeeded)
    int width_ = 1, height_ = 1;

    Mesh* mesh_ = nullptr;

    // Scene info cached from the mesh bbox.
    float centerX_ = 0.f, centerY_ = 0.f, centerZ_ = 0.f;
    float diag_ = 10.f;      // bbox diagonal (fallback 10)
    float groundZ_ = 0.f;    // grid plane height

    // Orbital camera (Z-up world, CAD style).
    float targetX_ = 0.f, targetY_ = 0.f, targetZ_ = 0.f;
    float dist_ = 25.f;
    float yawDeg_ = 45.f;    // around +Z, from +X axis
    float pitchDeg_ = 30.f;  // elevation, clamped to +/-89

    bool wireframe_ = false;
    bool smooth_ = true;
    bool grid_ = true;
    bool shadows_ = true;    // default on (modern path only)
    bool showHelp_ = true;   // key hints overlay (modern path only)
    bool showInfo_ = false;  // model info overlay (modern path only)
    bool showTree_ = true;   // model tree panel (when the mesh has nodes)

    // Cached per-axis bbox of the current mesh (world space).
    float bbMin_[3] = {0.f, 0.f, 0.f};
    float bbMax_[3] = {0.f, 0.f, 0.f};

    // Section planes. Position is stored normalized (0..1 across the bbox
    // extent) so it survives model reloads; flip selects the kept side.
    struct Section {
        bool enabled = false;
        bool flip = false;
        float t = 0.5f;
    };
    Section sections_[3];
    int sectionHighlight_ = -1;   // axis being adjusted, -1 = none

    std::string modelName_;       // ASCII-safe display name

    // Model tree UI state (parallel to mesh_->nodes; rebuilt per setMesh).
    std::vector<char> treeExpanded_;
    int treeScroll_ = 0;          // first shown row (clamped in layout)
    // Snapshot of the last visibility toggle, so the first click of a
    // double-click can be undone before fitting the view on the row.
    int lastToggleNode_ = -1;
    std::vector<std::pair<int, char>> lastToggleVis_;

    // Merged vertex ranges of the visible leaves: {firstVertex, vertexCount}.
    // Single full range when the mesh has no node tree.
    std::vector<std::pair<int, int>> drawRanges_;

    // Modern-path state.
    bool modern_ = false;
    int  msaaSamples_ = 0;
    bool meshHasNormals_ = false;

    unsigned meshVao_ = 0, meshVbo_ = 0;
    int meshVerts_ = 0;
    unsigned lineVao_ = 0, lineVbo_ = 0;
    int lineVerts_ = 0;
    unsigned groundVao_ = 0, groundVbo_ = 0;
    unsigned bgVao_ = 0;

    unsigned shadowFbo_ = 0, shadowTex_ = 0;
    int shadowSize_ = 2048;

    // Section-plane highlight quad (small dynamic VBO, position only).
    unsigned secVao_ = 0, secVbo_ = 0;

    Overlay overlay_;
    bool overlayOk_ = false;

    unsigned progLit_ = 0, progDepth_ = 0, progLine_ = 0;
    unsigned progWire_ = 0, progBg_ = 0, progGround_ = 0;

    // Cached uniform locations.
    int uLitViewProj_ = -1, uLitLightVP_ = -1, uLitCamPos_ = -1;
    int uLitFlat_ = -1, uLitShadows_ = -1, uLitTexel_ = -1;
    int uLitClip_ = -1;
    int uDepthLightVP_ = -1, uDepthClip_ = -1;
    int uLineViewProj_ = -1;
    int uWireViewProj_ = -1, uWireColor_ = -1, uWireAlpha_ = -1;
    int uWireClip_ = -1;
    int uGroundViewProj_ = -1, uGroundLightVP_ = -1, uGroundTexel_ = -1;
};
