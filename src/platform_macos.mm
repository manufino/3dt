/*
 * platform_macos.mm
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

// macOS (Cocoa/AppKit + NSOpenGLContext) implementation of platform.h,
// written directly against AppKit (no GLFW/SDL/Qt). Objective-C++, compiled
// with ARC (-fobjc-arc): no manual retain/release anywhere; the file-scope
// strong globals below own the Cocoa objects.
//
// - App setup: manual NSApplication (no nib), activation policy Regular so
//   the process gets a Dock icon and menu bar, minimal menu with Quit (Cmd+Q
//   routed through applicationShouldTerminate: -> platform::quit() so the
//   process always exits by returning from main, running onClose on the way).
// - GL: NSOpenGLContext on a plain NSView (mirrors the manual-context style
//   of the Win32/X11 backends). Core profile via NSOpenGLProfileVersion3_2Core
//   - there is no "3.3" selector on macOS; requesting 3.2 core returns the
//   highest core version the driver has (4.1 on any Mac since ~2011), so the
//   renderer's GLSL 330 shaders compile. MSAA 8x -> 4x -> none fallback.
//   Legacy fallback (recreateLegacyGLContext) uses NSOpenGLProfileVersionLegacy.
// - Retina: the app works in PIXELS everywhere (glViewport, overlay, mouse).
//   The view uses wantsBestResolutionOpenGLSurface, sizes are reported via
//   convertRectToBacking, and mouse positions (AppKit points, bottom-left
//   origin) are flipped and scaled by backingScaleFactor to top-left pixels.
// - Input: mouseDown/Up/Dragged for left/right/other buttons, double click
//   via clickCount, scrollWheel with hasPreciseScrollingDeltas handling
//   (trackpad pixel deltas scaled down; the app consumes fractional steps
//   just like fractional WM_MOUSEWHEEL deltas on Windows). Keyboard via
//   charactersIgnoringModifiers; BOTH Command and Control map to ModCtrl, so
//   the file dialog opens with Cmd+O as a Mac user expects and with Ctrl+O
//   as documented. Cmd+letter events are delivered as key equivalents, not
//   keyDown, so performKeyEquivalent: forwards them (with a synthetic
//   release, since macOS suppresses keyUp while Cmd is held).
// - Drag&drop: NSPasteboardTypeFileURL on the content view.
// - Rendering is on-demand: requestRedraw() -> setNeedsDisplay -> drawRect:
//   -> onDraw (the renderer calls swapBuffers() -> flushBuffer itself).
// - Overlay font: the embedded 5x7 bitmap font shared with the X11 backend
//   (src/bitmap_font.*) - identical overlay on Linux and macOS, no CoreText.
//
// NOTE: developed on Windows against the AppKit API from memory; this
// backend is built by CI and still needs testing on a real Mac.

#ifdef __APPLE__

#import <Cocoa/Cocoa.h>

#include <dlfcn.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "platform.h"
#include "bitmap_font.h"

// --- Objective-C interfaces --------------------------------------------------
// Declared before the C++ helpers so the helpers can message the globals.

@interface TDTView : NSView
@end

@interface TDTWindowDelegate : NSObject <NSWindowDelegate>
@end

@interface TDTAppDelegate : NSObject <NSApplicationDelegate>
@end

// --- globals + helpers -------------------------------------------------------

namespace {

platform::Callbacks g_cb;

// ARC: file-scope Objective-C pointers are implicitly __strong; these
// references own the window, view, delegates and GL context.
NSWindow*           g_window = nil;
TDTView*            g_view = nil;
TDTWindowDelegate*  g_windowDelegate = nil;
TDTAppDelegate*     g_appDelegate = nil;
NSOpenGLContext*    g_glContext = nil;

// Async key state (updated from keyDown/keyUp), for wheel-with-key-held.
bool g_keyDown[32] = {};

bool& keyDownSlot(platform::Key k) {
    return g_keyDown[static_cast<int>(k) & 31];
}

// --- key mapping -------------------------------------------------------------

platform::Key mapKeyEvent(NSEvent* e) {
    using platform::Key;
    if ([e keyCode] == 53) return Key::Escape;   // kVK_Escape
    NSString* chars = [e charactersIgnoringModifiers];
    if ([chars length] == 0) return Key::Unknown;
    unichar c = [chars characterAtIndex:0];
    if (c >= 'A' && c <= 'Z') c = static_cast<unichar>(c - 'A' + 'a');
    switch (c) {
    case 'w': return Key::W;
    case 's': return Key::S;
    case 'g': return Key::G;
    case 'o': return Key::O;
    case 'f': return Key::F;
    case 'h': return Key::H;
    case 'i': return Key::I;
    case 't': return Key::T;
    case 'x': return Key::X;
    case 'y': return Key::Y;
    case 'z': return Key::Z;
    case NSF11FunctionKey: return Key::F11;
    case 0x1B: return Key::Escape;
    default:  return Key::Unknown;
    }
}

// Command AND Control both report ModCtrl: "Ctrl+O" works as Cmd+O too,
// matching the platform convention that Cmd plays the Ctrl role on macOS.
unsigned modsOf(NSEvent* e) {
    NSEventModifierFlags f = [e modifierFlags];
    unsigned mods = 0;
    if (f & (NSEventModifierFlagControl | NSEventModifierFlagCommand))
        mods |= platform::ModCtrl;
    if (f & NSEventModifierFlagShift)
        mods |= platform::ModShift;
    return mods;
}

// Dispatch a key event to the app; returns false for keys we do not map.
bool handleKeyEvent(NSEvent* e, bool down) {
    platform::Key k = mapKeyEvent(e);
    if (k == platform::Key::Unknown) return false;
    bool repeat = down && [e isARepeat];
    keyDownSlot(k) = down;
    if (g_cb.onKey) g_cb.onKey(k, down, modsOf(e), repeat);
    return true;
}

// --- coordinates -------------------------------------------------------------

// AppKit reports mouse positions in points with a bottom-left origin; the
// app wants top-left pixels, consistent with the sizes from onResize.
void mousePixelPos(NSEvent* e, int& x, int& y) {
    if (!g_view || !g_window) {
        x = y = 0;
        return;
    }
    NSPoint p = [g_view convertPoint:[e locationInWindow] fromView:nil];
    NSRect b = [g_view bounds];
    CGFloat s = [g_window backingScaleFactor];
    x = static_cast<int>(std::lround(p.x * s));
    y = static_cast<int>(std::lround((b.size.height - p.y) * s));
}

void clientSizePixels(int& w, int& h) {
    w = h = 0;
    if (!g_view) return;
    NSRect r = [g_view convertRectToBacking:[g_view bounds]];
    w = static_cast<int>(std::lround(r.size.width));
    h = static_cast<int>(std::lround(r.size.height));
    if (w < 1) w = 1;
    if (h < 1) h = 1;
}

// Window resized or moved to a display with a different backing scale:
// resync the GL surface and report the new pixel size.
void handleResize() {
    if (g_glContext) [g_glContext update];
    int w = 0, h = 0;
    clientSizePixels(w, h);
    if (g_cb.onResize) g_cb.onResize(w, h);
    if (g_view) [g_view setNeedsDisplay:YES];
}

// --- files -------------------------------------------------------------------

platform::FileRef fileRefFromURL(NSURL* url) {
    platform::FileRef f;
    const char* fsr = [url fileSystemRepresentation];
    if (fsr) f.path = fsr;               // byte path for std::ifstream
    NSString* p = [url path];
    const char* disp = p ? [p UTF8String] : nullptr;
    f.display = disp ? disp : f.path;
    return f;
}

// Quit menu: a minimal menu bar whose only item is Quit (Cmd+Q). terminate:
// is intercepted by the app delegate, which turns it into platform::quit().
void buildMenuBar() {
    NSMenu* menubar = [[NSMenu alloc] initWithTitle:@""];
    NSMenuItem* appMenuItem = [[NSMenuItem alloc] initWithTitle:@""
                                                         action:nil
                                                  keyEquivalent:@""];
    [menubar addItem:appMenuItem];
    NSMenu* appMenu = [[NSMenu alloc] initWithTitle:@""];
    NSMenuItem* quitItem = [[NSMenuItem alloc]
        initWithTitle:@"Quit 3dt"
               action:@selector(terminate:)
        keyEquivalent:@"q"];
    [quitItem setTarget:NSApp];
    [appMenu addItem:quitItem];
    [appMenuItem setSubmenu:appMenu];
    [NSApp setMainMenu:menubar];
}

// Ask [NSApp run] to return. stop: only takes effect once the current event
// finishes processing, so post a no-op event in case the queue is idle.
void stopEventLoop() {
    [NSApp stop:nil];
    NSEvent* e = [NSEvent otherEventWithType:NSEventTypeApplicationDefined
                                    location:NSMakePoint(0, 0)
                               modifierFlags:0
                                   timestamp:0
                                windowNumber:0
                                     context:nil
                                     subtype:0
                                       data1:0
                                       data2:0];
    [NSApp postEvent:e atStart:NO];
}

} // namespace

// --- view --------------------------------------------------------------------

@implementation TDTView

- (BOOL)acceptsFirstResponder {
    return YES;
}

- (BOOL)isOpaque {
    return YES;
}

- (void)drawRect:(NSRect)dirtyRect {
    (void)dirtyRect;
    if (!g_glContext) return;
    // (Re)attach the context: setView is a no-op before the window exists,
    // so make sure the first real draw binds the surface.
    if ([g_glContext view] != self) [g_glContext setView:self];
    [g_glContext makeCurrentContext];
    if (g_cb.onDraw) g_cb.onDraw();   // renderer ends with swapBuffers()
}

// --- mouse ---

- (void)mouseDown:(NSEvent*)e {
    int x, y;
    mousePixelPos(e, x, y);
    if ([e clickCount] == 2) {
        // Like WM_LBUTTONDBLCLK: the second press becomes the double click.
        if (g_cb.onDoubleClick) g_cb.onDoubleClick(x, y);
    } else if (g_cb.onMouseButton) {
        g_cb.onMouseButton(platform::MouseButton::Left, true, x, y);
    }
}

- (void)mouseUp:(NSEvent*)e {
    int x, y;
    mousePixelPos(e, x, y);
    if (g_cb.onMouseButton)
        g_cb.onMouseButton(platform::MouseButton::Left, false, x, y);
}

- (void)rightMouseDown:(NSEvent*)e {
    int x, y;
    mousePixelPos(e, x, y);
    if (g_cb.onMouseButton)
        g_cb.onMouseButton(platform::MouseButton::Right, true, x, y);
}

- (void)rightMouseUp:(NSEvent*)e {
    int x, y;
    mousePixelPos(e, x, y);
    if (g_cb.onMouseButton)
        g_cb.onMouseButton(platform::MouseButton::Right, false, x, y);
}

- (void)otherMouseDown:(NSEvent*)e {
    int x, y;
    mousePixelPos(e, x, y);
    if (g_cb.onMouseButton)
        g_cb.onMouseButton(platform::MouseButton::Middle, true, x, y);
}

- (void)otherMouseUp:(NSEvent*)e {
    int x, y;
    mousePixelPos(e, x, y);
    if (g_cb.onMouseButton)
        g_cb.onMouseButton(platform::MouseButton::Middle, false, x, y);
}

- (void)mouseMoved:(NSEvent*)e {
    int x, y;
    mousePixelPos(e, x, y);
    if (g_cb.onMouseMove) g_cb.onMouseMove(x, y);
}

// AppKit keeps sending *Dragged events even when the pointer leaves the
// window during a button drag, which is exactly what captureMouse() wants.
- (void)mouseDragged:(NSEvent*)e {
    [self mouseMoved:e];
}

- (void)rightMouseDragged:(NSEvent*)e {
    [self mouseMoved:e];
}

- (void)otherMouseDragged:(NSEvent*)e {
    [self mouseMoved:e];
}

- (void)scrollWheel:(NSEvent*)e {
    if (!g_cb.onMouseWheel) return;
    CGFloat dy = [e scrollingDeltaY];
    float steps;
    if ([e hasPreciseScrollingDeltas]) {
        // Trackpad / Magic Mouse: dy is in pixels; scale so a typical swipe
        // is a few notches. Fractional steps are fine - the app consumes
        // float steps (Windows touchpads produce fractions of WHEEL_DELTA
        // the same way).
        steps = static_cast<float>(dy / 30.0);
    } else {
        // Conventional wheel: dy is in lines, one notch = 1.0.
        steps = static_cast<float>(dy);
    }
    if (steps != 0.0f) g_cb.onMouseWheel(steps);
}

// --- keyboard ---

- (void)keyDown:(NSEvent*)e {
    // Unknown keys are swallowed silently (no [super keyDown:] - it beeps),
    // matching the other backends.
    handleKeyEvent(e, true);
}

- (void)keyUp:(NSEvent*)e {
    handleKeyEvent(e, false);
}

- (BOOL)performKeyEquivalent:(NSEvent*)e {
    // Cmd+letter never reaches keyDown: - it is routed as a key equivalent.
    // Handle the keys we map (so Cmd+O opens the dialog); anything else
    // (Cmd+Q) falls through to the menu. macOS suppresses keyUp while Cmd
    // is held, so a synthetic release keeps isKeyDown() consistent.
    if ([e type] == NSEventTypeKeyDown &&
        ([e modifierFlags] & NSEventModifierFlagCommand)) {
        platform::Key k = mapKeyEvent(e);
        if (k != platform::Key::Unknown) {
            handleKeyEvent(e, true);
            keyDownSlot(k) = false;
            if (g_cb.onKey) g_cb.onKey(k, false, modsOf(e), false);
            return YES;
        }
    }
    return [super performKeyEquivalent:e];
}

// --- drag & drop ---

- (NSDragOperation)draggingEntered:(id<NSDraggingInfo>)sender {
    NSPasteboard* pb = [sender draggingPasteboard];
    NSDictionary* opts = @{ NSPasteboardURLReadingFileURLsOnlyKey : @YES };
    if ([pb canReadObjectForClasses:@[ [NSURL class] ] options:opts])
        return NSDragOperationCopy;
    return NSDragOperationNone;
}

- (BOOL)prepareForDragOperation:(id<NSDraggingInfo>)sender {
    (void)sender;
    return YES;
}

- (BOOL)performDragOperation:(id<NSDraggingInfo>)sender {
    NSPasteboard* pb = [sender draggingPasteboard];
    NSDictionary* opts = @{ NSPasteboardURLReadingFileURLsOnlyKey : @YES };
    NSArray* urls = [pb readObjectsForClasses:@[ [NSURL class] ]
                                      options:opts];
    if ([urls count] == 0) return NO;
    NSURL* url = [urls objectAtIndex:0];
    if (![url isFileURL]) return NO;
    if (g_cb.onDropFile) g_cb.onDropFile(fileRefFromURL(url));
    return YES;
}

@end

// --- window delegate ---------------------------------------------------------

@implementation TDTWindowDelegate

- (void)windowDidResize:(NSNotification*)notification {
    (void)notification;
    handleResize();
}

- (void)windowDidChangeBackingProperties:(NSNotification*)notification {
    (void)notification;   // retina <-> non-retina display move
    handleResize();
}

- (void)windowWillClose:(NSNotification*)notification {
    (void)notification;
    if (g_cb.onClose) g_cb.onClose();   // releases the GL resources/context
    stopEventLoop();
}

@end

// --- app delegate ------------------------------------------------------------

@implementation TDTAppDelegate

- (void)applicationDidFinishLaunching:(NSNotification*)notification {
    (void)notification;
    [NSApp activateIgnoringOtherApps:YES];
}

// Cmd+Q / Dock "Quit" arrive here as terminate:. Convert them into the
// regular close path (onClose fires, [NSApp run] returns, main returns) and
// cancel the terminate so the process never exits behind the app's back.
- (NSApplicationTerminateReply)applicationShouldTerminate:
    (NSApplication*)sender {
    (void)sender;
    platform::quit();
    return NSTerminateCancel;
}

@end

// --- platform interface ------------------------------------------------------

namespace platform {

// --- window ------------------------------------------------------------------

bool createWindow(const char* titleUtf8, int width, int height,
                  const Callbacks& cb) {
    g_cb = cb;

    NSRect rect = NSMakeRect(0, 0, width, height);
    NSWindowStyleMask style = NSWindowStyleMaskTitled |
                              NSWindowStyleMaskClosable |
                              NSWindowStyleMaskMiniaturizable |
                              NSWindowStyleMaskResizable;
    g_window = [[NSWindow alloc] initWithContentRect:rect
                                           styleMask:style
                                             backing:NSBackingStoreBuffered
                                               defer:NO];
    if (!g_window) return false;

    // ARC owns the window through g_window; the default close behavior of
    // autoreleasing the window on close would double-free it.
    [g_window setReleasedWhenClosed:NO];
    [g_window setCollectionBehavior:
        [g_window collectionBehavior] |
        NSWindowCollectionBehaviorFullScreenPrimary];
    [g_window center];

    g_view = [[TDTView alloc] initWithFrame:rect];
    if (!g_view) return false;
    // Retina: ask for a full-resolution GL surface; all sizes the app sees
    // are backing pixels, not points.
    [g_view setWantsBestResolutionOpenGLSurface:YES];
    [g_window setContentView:g_view];
    [g_window makeFirstResponder:g_view];
    [g_window setAcceptsMouseMovedEvents:YES];

    g_windowDelegate = [[TDTWindowDelegate alloc] init];
    [g_window setDelegate:g_windowDelegate];

    [g_view registerForDraggedTypes:@[ NSPasteboardTypeFileURL ]];

    setWindowTitle(titleUtf8);
    return true;
}

void showWindow() {
    if (!g_window) return;
    [g_window makeKeyAndOrderFront:nil];
    [NSApp activateIgnoringOtherApps:YES];
}

void setWindowTitle(const char* titleUtf8) {
    if (!g_window) return;
    NSString* t = titleUtf8 ? [NSString stringWithUTF8String:titleUtf8] : nil;
    [g_window setTitle:(t ? t : @"")];
}

void getClientSize(int& w, int& h) {
    clientSizePixels(w, h);
}

void toggleFullscreen() {
    if (!g_window) return;
    // Native fullscreen; windowDidResize reports the new size as usual.
    [g_window toggleFullScreen:nil];
}

void requestRedraw() {
    if (g_view) [g_view setNeedsDisplay:YES];
}

void captureMouse(bool on) {
    // AppKit keeps delivering mouseDragged to the view for the whole drag,
    // even outside the window; nothing to do.
    (void)on;
}

bool isKeyDown(Key k) {
    return keyDownSlot(k);
}

// --- OpenGL context ----------------------------------------------------------

bool createGLContext(GLContextInfo& out) {
    if (!g_view) return false;

    // Core profile with MSAA 8x -> 4x -> none. NSOpenGLProfileVersion3_2Core
    // yields the highest core version available (4.1 in practice), which
    // covers the renderer's GLSL 330; if anything is still missing the
    // renderer falls back through recreateLegacyGLContext().
    const int samplesTry[3] = {8, 4, 0};
    NSOpenGLPixelFormat* pf = nil;
    int gotSamples = 0;
    for (int s : samplesTry) {
        NSOpenGLPixelFormatAttribute attrs[24];
        int i = 0;
        attrs[i++] = NSOpenGLPFAOpenGLProfile;
        attrs[i++] = NSOpenGLProfileVersion3_2Core;
        attrs[i++] = NSOpenGLPFADoubleBuffer;
        attrs[i++] = NSOpenGLPFAColorSize;
        attrs[i++] = 24;
        attrs[i++] = NSOpenGLPFAAlphaSize;
        attrs[i++] = 8;
        attrs[i++] = NSOpenGLPFADepthSize;
        attrs[i++] = 24;
        attrs[i++] = NSOpenGLPFAStencilSize;
        attrs[i++] = 8;
        if (s > 0) {
            attrs[i++] = NSOpenGLPFAMultisample;
            attrs[i++] = NSOpenGLPFASampleBuffers;
            attrs[i++] = 1;
            attrs[i++] = NSOpenGLPFASamples;
            attrs[i++] = static_cast<NSOpenGLPixelFormatAttribute>(s);
        }
        attrs[i] = 0;
        pf = [[NSOpenGLPixelFormat alloc] initWithAttributes:attrs];
        if (pf) {
            gotSamples = s;
            break;
        }
    }
    if (!pf) return false;

    g_glContext = [[NSOpenGLContext alloc] initWithFormat:pf
                                             shareContext:nil];
    if (!g_glContext) return false;

    [g_glContext setView:g_view];
    [g_glContext makeCurrentContext];

    out.modern = true;
    out.msaaSamples = gotSamples;
    return true;
}

bool recreateLegacyGLContext() {
    if (!g_view) return false;
    if (g_glContext) {
        [NSOpenGLContext clearCurrentContext];
        [g_glContext clearDrawable];
        g_glContext = nil;
    }
    NSOpenGLPixelFormatAttribute attrs[] = {
        NSOpenGLPFAOpenGLProfile,
        NSOpenGLProfileVersionLegacy,
        NSOpenGLPFADoubleBuffer,
        NSOpenGLPFAColorSize, 24,
        NSOpenGLPFADepthSize, 24,
        0
    };
    NSOpenGLPixelFormat* pf =
        [[NSOpenGLPixelFormat alloc] initWithAttributes:attrs];
    if (!pf) return false;
    g_glContext = [[NSOpenGLContext alloc] initWithFormat:pf
                                             shareContext:nil];
    if (!g_glContext) return false;
    [g_glContext setView:g_view];
    [g_glContext makeCurrentContext];
    return true;
}

void destroyGLContext() {
    if (!g_glContext) return;
    [NSOpenGLContext clearCurrentContext];
    [g_glContext clearDrawable];
    g_glContext = nil;
}

void makeContextCurrent() {
    if (g_glContext) [g_glContext makeCurrentContext];
}

void swapBuffers() {
    if (g_glContext) [g_glContext flushBuffer];
}

void* getGLProcAddress(const char* name) {
    // Every GL entry point (core and extension) is a plain exported symbol
    // of the OpenGL framework. dlsym on that image specifically - not
    // RTLD_DEFAULT, which would search this executable first and find the
    // loader's own same-named function-pointer VARIABLES instead of the
    // framework functions.
    static void* framework = dlopen(
        "/System/Library/Frameworks/OpenGL.framework/OpenGL",
        RTLD_LAZY | RTLD_LOCAL);
    return framework ? dlsym(framework, name) : nullptr;
}

void setSwapInterval(int interval) {
    if (!g_glContext) return;
    GLint v = interval;
    [g_glContext setValues:&v
              forParameter:NSOpenGLContextParameterSwapInterval];
}

// --- services ----------------------------------------------------------------

bool openFileDialog(FileRef& out) {
    NSOpenPanel* panel = [NSOpenPanel openPanel];
    [panel setCanChooseFiles:YES];
    [panel setCanChooseDirectories:NO];
    [panel setAllowsMultipleSelection:NO];
    [panel setTitle:@"Open 3D model"];
    // allowedContentTypes would drag in UniformTypeIdentifiers (macOS 11+);
    // the older string-extension API keeps the backend dependency-free.
    [panel setAllowedFileTypes:@[ @"stl", @"step", @"stp", @"obj" ]];
    if ([panel runModal] != NSModalResponseOK) return false;
    NSURL* url = [panel URL];
    if (!url) return false;
    out = fileRefFromURL(url);
    return true;
}

void showMessageBox(const char* title, const char* textUtf8, bool isError) {
    NSAlert* alert = [[NSAlert alloc] init];
    [alert setAlertStyle:(isError ? NSAlertStyleCritical
                                  : NSAlertStyleWarning)];
    NSString* t = title ? [NSString stringWithUTF8String:title] : nil;
    NSString* m = textUtf8 ? [NSString stringWithUTF8String:textUtf8] : nil;
    [alert setMessageText:(t ? t : @"")];
    [alert setInformativeText:(m ? m : @"")];
    [alert runModal];
}

// The embedded 5x7 font shared with the X11 backend (src/bitmap_font.*),
// expanded to 8x16 cells with the layout contract of the Windows GDI
// rasterizer: glyph i in cell i with a 1px padding offset, 0/255 coverage.
bool rasterizeFontAtlas(int firstChar, int lastChar, int cols, int rows,
                        FontAtlas& out) {
    return bitmapfont::rasterize(firstChar, lastChar, cols, rows, out);
}

void debugLog(const char* msg) {
    if (msg) std::fputs(msg, stderr);
}

// --- event loop --------------------------------------------------------------

int runEventLoop() {
    if (!g_window) return 1;
    [NSApp run];   // returns after stopEventLoop() (window closed / quit)
    // The window is gone (or closing); drop the strong references.
    if (g_window) [g_window setDelegate:nil];
    g_glContext = nil;
    g_view = nil;
    g_windowDelegate = nil;
    g_window = nil;
    return 0;
}

void quit() {
    if (g_window) {
        // close triggers windowWillClose -> onClose + stopEventLoop, the
        // same path as clicking the close button.
        [g_window close];
    } else {
        stopEventLoop();
    }
}

} // namespace platform

// --- entry point -------------------------------------------------------------

int main(int argc, char** argv) {
    @autoreleasepool {
        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
        g_appDelegate = [[TDTAppDelegate alloc] init];
        [NSApp setDelegate:g_appDelegate];
        buildMenuBar();

        std::vector<platform::FileRef> files;
        for (int i = 1; i < argc; ++i) {
            platform::FileRef f;
            f.path = argv[i];      // POSIX byte paths (UTF-8), untouched
            f.display = argv[i];
            files.push_back(f);
        }
        return appMain(files);
    }
}

#endif // __APPLE__
