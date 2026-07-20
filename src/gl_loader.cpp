/*
 * gl_loader.cpp
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

// Minimal OpenGL 3.3 core function loader implementation. Fully platform
// neutral: the actual symbol lookup is delegated to the function passed by
// the platform layer.

#include "gl_loader.h"

PFNGLCREATESHADERPROC            glCreateShader = nullptr;
PFNGLDELETESHADERPROC            glDeleteShader = nullptr;
PFNGLSHADERSOURCEPROC            glShaderSource = nullptr;
PFNGLCOMPILESHADERPROC           glCompileShader = nullptr;
PFNGLGETSHADERIVPROC             glGetShaderiv = nullptr;
PFNGLGETSHADERINFOLOGPROC        glGetShaderInfoLog = nullptr;
PFNGLCREATEPROGRAMPROC           glCreateProgram = nullptr;
PFNGLDELETEPROGRAMPROC           glDeleteProgram = nullptr;
PFNGLATTACHSHADERPROC            glAttachShader = nullptr;
PFNGLLINKPROGRAMPROC             glLinkProgram = nullptr;
PFNGLGETPROGRAMIVPROC            glGetProgramiv = nullptr;
PFNGLGETPROGRAMINFOLOGPROC       glGetProgramInfoLog = nullptr;
PFNGLUSEPROGRAMPROC              glUseProgram = nullptr;
PFNGLGETUNIFORMLOCATIONPROC      glGetUniformLocation = nullptr;
PFNGLUNIFORM1IPROC               glUniform1i = nullptr;
PFNGLUNIFORM1FPROC               glUniform1f = nullptr;
PFNGLUNIFORM2FPROC               glUniform2f = nullptr;
PFNGLUNIFORM3FPROC               glUniform3f = nullptr;
PFNGLUNIFORM4FVPROC              glUniform4fv = nullptr;
PFNGLUNIFORMMATRIX4FVPROC        glUniformMatrix4fv = nullptr;

PFNGLGENBUFFERSPROC              glGenBuffers = nullptr;
PFNGLDELETEBUFFERSPROC           glDeleteBuffers = nullptr;
PFNGLBINDBUFFERPROC              glBindBuffer = nullptr;
PFNGLBUFFERDATAPROC              glBufferData = nullptr;
PFNGLBUFFERSUBDATAPROC           glBufferSubData = nullptr;
PFNGLGENVERTEXARRAYSPROC         glGenVertexArrays = nullptr;
PFNGLDELETEVERTEXARRAYSPROC      glDeleteVertexArrays = nullptr;
PFNGLBINDVERTEXARRAYPROC         glBindVertexArray = nullptr;
PFNGLENABLEVERTEXATTRIBARRAYPROC glEnableVertexAttribArray = nullptr;
PFNGLDISABLEVERTEXATTRIBARRAYPROC glDisableVertexAttribArray = nullptr;
PFNGLVERTEXATTRIBPOINTERPROC     glVertexAttribPointer = nullptr;
PFNGLVERTEXATTRIB3FPROC          glVertexAttrib3f = nullptr;

PFNGLGENFRAMEBUFFERSPROC         glGenFramebuffers = nullptr;
PFNGLDELETEFRAMEBUFFERSPROC      glDeleteFramebuffers = nullptr;
PFNGLBINDFRAMEBUFFERPROC         glBindFramebuffer = nullptr;
PFNGLFRAMEBUFFERTEXTURE2DPROC    glFramebufferTexture2D = nullptr;
PFNGLCHECKFRAMEBUFFERSTATUSPROC  glCheckFramebufferStatus = nullptr;
PFNGLACTIVETEXTUREPROC           glActiveTexture = nullptr;

bool glLoaderInit(void* (*getProc)(const char* name)) {
    if (!getProc) return false;
    bool ok = true;
#define GLL_LOAD(type, name) \
    do { \
        name = reinterpret_cast<type>(getProc(#name)); \
        if (!name) ok = false; \
    } while (0)

    GLL_LOAD(PFNGLCREATESHADERPROC, glCreateShader);
    GLL_LOAD(PFNGLDELETESHADERPROC, glDeleteShader);
    GLL_LOAD(PFNGLSHADERSOURCEPROC, glShaderSource);
    GLL_LOAD(PFNGLCOMPILESHADERPROC, glCompileShader);
    GLL_LOAD(PFNGLGETSHADERIVPROC, glGetShaderiv);
    GLL_LOAD(PFNGLGETSHADERINFOLOGPROC, glGetShaderInfoLog);
    GLL_LOAD(PFNGLCREATEPROGRAMPROC, glCreateProgram);
    GLL_LOAD(PFNGLDELETEPROGRAMPROC, glDeleteProgram);
    GLL_LOAD(PFNGLATTACHSHADERPROC, glAttachShader);
    GLL_LOAD(PFNGLLINKPROGRAMPROC, glLinkProgram);
    GLL_LOAD(PFNGLGETPROGRAMIVPROC, glGetProgramiv);
    GLL_LOAD(PFNGLGETPROGRAMINFOLOGPROC, glGetProgramInfoLog);
    GLL_LOAD(PFNGLUSEPROGRAMPROC, glUseProgram);
    GLL_LOAD(PFNGLGETUNIFORMLOCATIONPROC, glGetUniformLocation);
    GLL_LOAD(PFNGLUNIFORM1IPROC, glUniform1i);
    GLL_LOAD(PFNGLUNIFORM1FPROC, glUniform1f);
    GLL_LOAD(PFNGLUNIFORM2FPROC, glUniform2f);
    GLL_LOAD(PFNGLUNIFORM3FPROC, glUniform3f);
    GLL_LOAD(PFNGLUNIFORM4FVPROC, glUniform4fv);
    GLL_LOAD(PFNGLUNIFORMMATRIX4FVPROC, glUniformMatrix4fv);

    GLL_LOAD(PFNGLGENBUFFERSPROC, glGenBuffers);
    GLL_LOAD(PFNGLDELETEBUFFERSPROC, glDeleteBuffers);
    GLL_LOAD(PFNGLBINDBUFFERPROC, glBindBuffer);
    GLL_LOAD(PFNGLBUFFERDATAPROC, glBufferData);
    GLL_LOAD(PFNGLBUFFERSUBDATAPROC, glBufferSubData);
    GLL_LOAD(PFNGLGENVERTEXARRAYSPROC, glGenVertexArrays);
    GLL_LOAD(PFNGLDELETEVERTEXARRAYSPROC, glDeleteVertexArrays);
    GLL_LOAD(PFNGLBINDVERTEXARRAYPROC, glBindVertexArray);
    GLL_LOAD(PFNGLENABLEVERTEXATTRIBARRAYPROC, glEnableVertexAttribArray);
    GLL_LOAD(PFNGLDISABLEVERTEXATTRIBARRAYPROC, glDisableVertexAttribArray);
    GLL_LOAD(PFNGLVERTEXATTRIBPOINTERPROC, glVertexAttribPointer);
    GLL_LOAD(PFNGLVERTEXATTRIB3FPROC, glVertexAttrib3f);

    GLL_LOAD(PFNGLGENFRAMEBUFFERSPROC, glGenFramebuffers);
    GLL_LOAD(PFNGLDELETEFRAMEBUFFERSPROC, glDeleteFramebuffers);
    GLL_LOAD(PFNGLBINDFRAMEBUFFERPROC, glBindFramebuffer);
    GLL_LOAD(PFNGLFRAMEBUFFERTEXTURE2DPROC, glFramebufferTexture2D);
    GLL_LOAD(PFNGLCHECKFRAMEBUFFERSTATUSPROC, glCheckFramebufferStatus);
    GLL_LOAD(PFNGLACTIVETEXTUREPROC, glActiveTexture);

#undef GLL_LOAD
    return ok;
}
