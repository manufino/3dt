cmake_minimum_required(VERSION 3.16)
project(3dt CXX)

if(APPLE)
    # platform_macos.mm is Objective-C++.
    enable_language(OBJCXX)
    set(CMAKE_OBJCXX_STANDARD 17)
    set(CMAKE_OBJCXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COMMON_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mesh.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/stl_loader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/obj_loader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/step_loader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/step_tess.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gl_loader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/renderer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/overlay.cpp
    # Embedded overlay font used by the X11 and macOS backends; pure C++
    # with no system includes, so it compiles (unused) on Windows too,
    # keeping this list identical to the Makefile wildcard.
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bitmap_font.cpp
)

if(WIN32)
    add_executable(3dt WIN32 ${COMMON_SRC}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/platform_win32.cpp)
    if(MINGW)
        target_compile_options(3dt PRIVATE -Wall -Wextra -municode)
        target_link_options(3dt PRIVATE -municode -static)
    endif()
    target_link_libraries(3dt PRIVATE opengl32 gdi32 user32 comdlg32 shell32)
elseif(APPLE)
    # macOS: Cocoa/AppKit + NSOpenGLContext backend (Objective-C++, ARC).
    # Plain executable, no .app bundle: run it from the terminal like on
    # the other platforms (drag&drop and the Cmd/Ctrl+O dialog still work).
    add_executable(3dt ${COMMON_SRC}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/platform_macos.mm)
    target_compile_options(3dt PRIVATE -Wall -Wextra)
    # OpenGL is deprecated-but-working on macOS: silence the GL deprecation
    # markers everywhere, and the NSOpenGL* API deprecations in the backend.
    target_compile_definitions(3dt PRIVATE GL_SILENCE_DEPRECATION)
    set_source_files_properties(
        ${CMAKE_CURRENT_SOURCE_DIR}/src/platform_macos.mm PROPERTIES
        COMPILE_OPTIONS "-fobjc-arc;-Wno-deprecated-declarations")
    target_link_libraries(3dt PRIVATE
        "-framework Cocoa" "-framework OpenGL")
else()
    # Linux: Xlib + GLX backend. Requires the X11 and OpenGL dev packages
    # (Debian/Ubuntu: libx11-dev and libgl1-mesa-dev or libglvnd-dev).
    add_executable(3dt ${COMMON_SRC}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/platform_x11.cpp)
    target_compile_options(3dt PRIVATE -Wall -Wextra)
    # Static libstdc++/libgcc so release binaries also run on older distros.
    target_link_options(3dt PRIVATE -static-libstdc++ -static-libgcc)
    target_link_libraries(3dt PRIVATE X11 GL)
endif()
