/*
 * gl_loader.h
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
// Minimal OpenGL 3.3 core function loader (no GLEW/GLAD), platform neutral.
// Uses the PFNGL...PROC typedefs from <GL/glext.h>; pointers are resolved
// through the get-proc-address function supplied by the platform layer
// (wglGetProcAddress+opengl32 on Windows, glXGetProcAddressARB on Linux)
// once a modern context is current.
//
// <GL/gl.h> is included without <windows.h>: on Windows the two macros it
// needs (APIENTRY/WINGDIAPI) are provided here instead.

#ifdef _WIN32
#ifndef APIENTRY
#define APIENTRY __stdcall
#endif
#ifndef WINGDIAPI
#define WINGDIAPI __declspec(dllimport)
#endif
#include <stddef.h>   // wchar_t for gl.h in lean mode
#endif

// Some <GL/gl.h> variants (Mesa) declare glActiveTexture (GL 1.3) as a real
// prototype, which would clash with the function pointer of the same name
// below; hide that declaration while the headers are included.
#define glActiveTexture glActiveTexture_glh_prototype_hidden_
#include <GL/gl.h>
#include <GL/glext.h>
#undef glActiveTexture

// Shaders / programs
extern PFNGLCREATESHADERPROC            glCreateShader;
extern PFNGLDELETESHADERPROC            glDeleteShader;
extern PFNGLSHADERSOURCEPROC            glShaderSource;
extern PFNGLCOMPILESHADERPROC           glCompileShader;
extern PFNGLGETSHADERIVPROC             glGetShaderiv;
extern PFNGLGETSHADERINFOLOGPROC        glGetShaderInfoLog;
extern PFNGLCREATEPROGRAMPROC           glCreateProgram;
extern PFNGLDELETEPROGRAMPROC           glDeleteProgram;
extern PFNGLATTACHSHADERPROC            glAttachShader;
extern PFNGLLINKPROGRAMPROC             glLinkProgram;
extern PFNGLGETPROGRAMIVPROC            glGetProgramiv;
extern PFNGLGETPROGRAMINFOLOGPROC       glGetProgramInfoLog;
extern PFNGLUSEPROGRAMPROC              glUseProgram;
extern PFNGLGETUNIFORMLOCATIONPROC      glGetUniformLocation;
extern PFNGLUNIFORM1IPROC               glUniform1i;
extern PFNGLUNIFORM1FPROC               glUniform1f;
extern PFNGLUNIFORM2FPROC               glUniform2f;
extern PFNGLUNIFORM3FPROC               glUniform3f;
extern PFNGLUNIFORM4FVPROC              glUniform4fv;
extern PFNGLUNIFORMMATRIX4FVPROC        glUniformMatrix4fv;

// Buffers / vertex arrays
extern PFNGLGENBUFFERSPROC              glGenBuffers;
extern PFNGLDELETEBUFFERSPROC           glDeleteBuffers;
extern PFNGLBINDBUFFERPROC              glBindBuffer;
extern PFNGLBUFFERDATAPROC              glBufferData;
extern PFNGLBUFFERSUBDATAPROC           glBufferSubData;
extern PFNGLGENVERTEXARRAYSPROC         glGenVertexArrays;
extern PFNGLDELETEVERTEXARRAYSPROC      glDeleteVertexArrays;
extern PFNGLBINDVERTEXARRAYPROC         glBindVertexArray;
extern PFNGLENABLEVERTEXATTRIBARRAYPROC glEnableVertexAttribArray;
extern PFNGLDISABLEVERTEXATTRIBARRAYPROC glDisableVertexAttribArray;
extern PFNGLVERTEXATTRIBPOINTERPROC     glVertexAttribPointer;
extern PFNGLVERTEXATTRIB3FPROC          glVertexAttrib3f;

// Framebuffers / textures
extern PFNGLGENFRAMEBUFFERSPROC         glGenFramebuffers;
extern PFNGLDELETEFRAMEBUFFERSPROC      glDeleteFramebuffers;
extern PFNGLBINDFRAMEBUFFERPROC         glBindFramebuffer;
extern PFNGLFRAMEBUFFERTEXTURE2DPROC    glFramebufferTexture2D;
extern PFNGLCHECKFRAMEBUFFERSTATUSPROC  glCheckFramebufferStatus;
extern PFNGLACTIVETEXTUREPROC           glActiveTexture;

// Resolve every pointer above through getProc (which must also cover GL 1.x
// entry points). A modern GL context must be current. Returns false if any
// function is missing (caller should fall back to the legacy path).
bool glLoaderInit(void* (*getProc)(const char* name));
