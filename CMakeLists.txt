cmake_minimum_required(VERSION 3.16)
project(3dt CXX)

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
)

if(WIN32)
    add_executable(3dt WIN32 ${COMMON_SRC}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/platform_win32.cpp)
    if(MINGW)
        target_compile_options(3dt PRIVATE -Wall -Wextra -municode)
        target_link_options(3dt PRIVATE -municode -static)
    endif()
    target_link_libraries(3dt PRIVATE opengl32 gdi32 user32 comdlg32 shell32)
else()
    # Linux: Xlib + GLX backend. Requires the X11 and OpenGL dev packages
    # (Debian/Ubuntu: libx11-dev and libgl1-mesa-dev or libglvnd-dev).
    add_executable(3dt ${COMMON_SRC}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/platform_x11.cpp)
    target_compile_options(3dt PRIVATE -Wall -Wextra)
    target_link_libraries(3dt PRIVATE X11 GL)
endif()
