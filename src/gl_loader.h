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
// Uses the Khronos PFNGL...PROC typedefs; pointers are resolved through the
// get-proc-address function supplied by the platform layer
// (wglGetProcAddress+opengl32 on Windows, glXGetProcAddressARB on Linux,
// dlsym on the OpenGL framework on macOS) once a modern context is current.
//
// <GL/gl.h> is included without <windows.h>: on Windows the two macros it
// needs (APIENTRY/WINGDIAPI) are provided here instead.
//
// macOS: <OpenGL/gl.h>/<OpenGL/glext.h> declare every GL entry point as a
// real function (all exported by OpenGL.framework) and provide no
// PFNGL...PROC typedefs (Apple uses its own glXxxProcPtr names). So each
// name re-declared below as a loader pointer is hidden while the system
// headers are included, and the Khronos typedefs are defined here.

#ifdef __APPLE__

#ifndef GL_SILENCE_DEPRECATION
#define GL_SILENCE_DEPRECATION 1
#endif

// Hide the system prototypes of every function the loader re-declares.
#define glCreateShader            glCreateShader_glh_hidden_
#define glDeleteShader            glDeleteShader_glh_hidden_
#define glShaderSource            glShaderSource_glh_hidden_
#define glCompileShader           glCompileShader_glh_hidden_
#define glGetShaderiv             glGetShaderiv_glh_hidden_
#define glGetShaderInfoLog        glGetShaderInfoLog_glh_hidden_
#define glCreateProgram           glCreateProgram_glh_hidden_
#define glDeleteProgram           glDeleteProgram_glh_hidden_
#define glAttachShader            glAttachShader_glh_hidden_
#define glLinkProgram             glLinkProgram_glh_hidden_
#define glGetProgramiv            glGetProgramiv_glh_hidden_
#define glGetProgramInfoLog       glGetProgramInfoLog_glh_hidden_
#define glUseProgram              glUseProgram_glh_hidden_
#define glGetUniformLocation      glGetUniformLocation_glh_hidden_
#define glUniform1i               glUniform1i_glh_hidden_
#define glUniform1f               glUniform1f_glh_hidden_
#define glUniform2f               glUniform2f_glh_hidden_
#define glUniform3f               glUniform3f_glh_hidden_
#define glUniform4fv              glUniform4fv_glh_hidden_
#define glUniformMatrix4fv        glUniformMatrix4fv_glh_hidden_
#define glGenBuffers              glGenBuffers_glh_hidden_
#define glDeleteBuffers           glDeleteBuffers_glh_hidden_
#define glBindBuffer              glBindBuffer_glh_hidden_
#define glBufferData              glBufferData_glh_hidden_
#define glBufferSubData           glBufferSubData_glh_hidden_
#define glGenVertexArrays         glGenVertexArrays_glh_hidden_
#define glDeleteVertexArrays      glDeleteVertexArrays_glh_hidden_
#define glBindVertexArray         glBindVertexArray_glh_hidden_
#define glEnableVertexAttribArray glEnableVertexAttribArray_glh_hidden_
#define glDisableVertexAttribArray glDisableVertexAttribArray_glh_hidden_
#define glVertexAttribPointer     glVertexAttribPointer_glh_hidden_
#define glVertexAttrib3f          glVertexAttrib3f_glh_hidden_
#define glGenFramebuffers         glGenFramebuffers_glh_hidden_
#define glDeleteFramebuffers      glDeleteFramebuffers_glh_hidden_
#define glBindFramebuffer         glBindFramebuffer_glh_hidden_
#define glFramebufferTexture2D    glFramebufferTexture2D_glh_hidden_
#define glCheckFramebufferStatus  glCheckFramebufferStatus_glh_hidden_
#define glActiveTexture           glActiveTexture_glh_hidden_

#include <OpenGL/gl.h>
#include <OpenGL/glext.h>

#undef glCreateShader
#undef glDeleteShader
#undef glShaderSource
#undef glCompileShader
#undef glGetShaderiv
#undef glGetShaderInfoLog
#undef glCreateProgram
#undef glDeleteProgram
#undef glAttachShader
#undef glLinkProgram
#undef glGetProgramiv
#undef glGetProgramInfoLog
#undef glUseProgram
#undef glGetUniformLocation
#undef glUniform1i
#undef glUniform1f
#undef glUniform2f
#undef glUniform3f
#undef glUniform4fv
#undef glUniformMatrix4fv
#undef glGenBuffers
#undef glDeleteBuffers
#undef glBindBuffer
#undef glBufferData
#undef glBufferSubData
#undef glGenVertexArrays
#undef glDeleteVertexArrays
#undef glBindVertexArray
#undef glEnableVertexAttribArray
#undef glDisableVertexAttribArray
#undef glVertexAttribPointer
#undef glVertexAttrib3f
#undef glGenFramebuffers
#undef glDeleteFramebuffers
#undef glBindFramebuffer
#undef glFramebufferTexture2D
#undef glCheckFramebufferStatus
#undef glActiveTexture

// Khronos-style function pointer typedefs (Apple's headers do not define
// PFNGL...PROC names). Signatures match <GL/glext.h>.
typedef GLuint (*PFNGLCREATESHADERPROC)(GLenum type);
typedef void (*PFNGLDELETESHADERPROC)(GLuint shader);
typedef void (*PFNGLSHADERSOURCEPROC)(GLuint shader, GLsizei count,
                                      const GLchar* const* string,
                                      const GLint* length);
typedef void (*PFNGLCOMPILESHADERPROC)(GLuint shader);
typedef void (*PFNGLGETSHADERIVPROC)(GLuint shader, GLenum pname,
                                     GLint* params);
typedef void (*PFNGLGETSHADERINFOLOGPROC)(GLuint shader, GLsizei bufSize,
                                          GLsizei* length, GLchar* infoLog);
typedef GLuint (*PFNGLCREATEPROGRAMPROC)(void);
typedef void (*PFNGLDELETEPROGRAMPROC)(GLuint program);
typedef void (*PFNGLATTACHSHADERPROC)(GLuint program, GLuint shader);
typedef void (*PFNGLLINKPROGRAMPROC)(GLuint program);
typedef void (*PFNGLGETPROGRAMIVPROC)(GLuint program, GLenum pname,
                                      GLint* params);
typedef void (*PFNGLGETPROGRAMINFOLOGPROC)(GLuint program, GLsizei bufSize,
                                           GLsizei* length, GLchar* infoLog);
typedef void (*PFNGLUSEPROGRAMPROC)(GLuint program);
typedef GLint (*PFNGLGETUNIFORMLOCATIONPROC)(GLuint program,
                                             const GLchar* name);
typedef void (*PFNGLUNIFORM1IPROC)(GLint location, GLint v0);
typedef void (*PFNGLUNIFORM1FPROC)(GLint location, GLfloat v0);
typedef void (*PFNGLUNIFORM2FPROC)(GLint location, GLfloat v0, GLfloat v1);
typedef void (*PFNGLUNIFORM3FPROC)(GLint location, GLfloat v0, GLfloat v1,
                                   GLfloat v2);
typedef void (*PFNGLUNIFORM4FVPROC)(GLint location, GLsizei count,
                                    const GLfloat* value);
typedef void (*PFNGLUNIFORMMATRIX4FVPROC)(GLint location, GLsizei count,
                                          GLboolean transpose,
                                          const GLfloat* value);
typedef void (*PFNGLGENBUFFERSPROC)(GLsizei n, GLuint* buffers);
typedef void (*PFNGLDELETEBUFFERSPROC)(GLsizei n, const GLuint* buffers);
typedef void (*PFNGLBINDBUFFERPROC)(GLenum target, GLuint buffer);
typedef void (*PFNGLBUFFERDATAPROC)(GLenum target, GLsizeiptr size,
                                    const void* data, GLenum usage);
typedef void (*PFNGLBUFFERSUBDATAPROC)(GLenum target, GLintptr offset,
                                       GLsizeiptr size, const void* data);
typedef void (*PFNGLGENVERTEXARRAYSPROC)(GLsizei n, GLuint* arrays);
typedef void (*PFNGLDELETEVERTEXARRAYSPROC)(GLsizei n, const GLuint* arrays);
typedef void (*PFNGLBINDVERTEXARRAYPROC)(GLuint array);
typedef void (*PFNGLENABLEVERTEXATTRIBARRAYPROC)(GLuint index);
typedef void (*PFNGLDISABLEVERTEXATTRIBARRAYPROC)(GLuint index);
typedef void (*PFNGLVERTEXATTRIBPOINTERPROC)(GLuint index, GLint size,
                                             GLenum type,
                                             GLboolean normalized,
                                             GLsizei stride,
                                             const void* pointer);
typedef void (*PFNGLVERTEXATTRIB3FPROC)(GLuint index, GLfloat x, GLfloat y,
                                        GLfloat z);
typedef void (*PFNGLGENFRAMEBUFFERSPROC)(GLsizei n, GLuint* framebuffers);
typedef void (*PFNGLDELETEFRAMEBUFFERSPROC)(GLsizei n,
                                            const GLuint* framebuffers);
typedef void (*PFNGLBINDFRAMEBUFFERPROC)(GLenum target, GLuint framebuffer);
typedef void (*PFNGLFRAMEBUFFERTEXTURE2DPROC)(GLenum target,
                                              GLenum attachment,
                                              GLenum textarget,
                                              GLuint texture, GLint level);
typedef GLenum (*PFNGLCHECKFRAMEBUFFERSTATUSPROC)(GLenum target);
typedef void (*PFNGLACTIVETEXTUREPROC)(GLenum texture);

// GL 3.x enum names the renderer uses that Apple's 2.1-era <OpenGL/gl.h>
// may only spell with legacy names (same values).
#ifndef GL_COMPARE_REF_TO_TEXTURE
#define GL_COMPARE_REF_TO_TEXTURE 0x884E
#endif
#ifndef GL_FRAMEBUFFER
#define GL_FRAMEBUFFER 0x8D40
#endif
#ifndef GL_FRAMEBUFFER_COMPLETE
#define GL_FRAMEBUFFER_COMPLETE 0x8CD5
#endif
#ifndef GL_DEPTH_ATTACHMENT
#define GL_DEPTH_ATTACHMENT 0x8D00
#endif
#ifndef GL_DEPTH_COMPONENT24
#define GL_DEPTH_COMPONENT24 0x81A6
#endif
#ifndef GL_CLAMP_TO_BORDER
#define GL_CLAMP_TO_BORDER 0x812D
#endif
#ifndef GL_TEXTURE_COMPARE_MODE
#define GL_TEXTURE_COMPARE_MODE 0x884C
#endif
#ifndef GL_TEXTURE_COMPARE_FUNC
#define GL_TEXTURE_COMPARE_FUNC 0x884D
#endif

#else // !__APPLE__

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

#endif // __APPLE__

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
