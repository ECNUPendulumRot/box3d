// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef BOX3D_GL_COMMON_HPP
#define BOX3D_GL_COMMON_HPP


#include "glad/gl.h"
#include "GLFW/glfw3.h"

#include <cassert>
#include <cstdio>
#include <malloc.h> 

/**
 * @brief Macro to offset a pointer for buffer data.
 * @param x Offset value.
 * @return Pointer to the offset location.
 */
#define BUFFER_OFFSET(x)  ((const void*) (x))

/**
 * @brief Checks for OpenGL errors and prints them.
 * Logs error messages to stderr if any OpenGL error is detected.
 */
inline void check_gl_error()
{
    GLenum errCode = glGetError();
    if (errCode != GL_NO_ERROR)
    {
        fprintf(stderr, "OpenGL error = %d\n", errCode);
        assert(false);
    }
}

/**
 * @brief Prints the log of a shader or program object.
 * @param object The shader or program object to print the log for.
 */
inline void print_log(GLuint object)
{
    GLint log_length = 0;
    if (glIsShader(object))
        glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_length);
    else if (glIsProgram(object))
        glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_length);
    else
    {
        fprintf(stderr, "printlog: Not a shader or a program\n");
        return;
    }

    char* log = (char*)malloc(log_length);

    if (glIsShader(object))
        glGetShaderInfoLog(object, log_length, NULL, log);
    else if (glIsProgram(object))
        glGetProgramInfoLog(object, log_length, NULL, log);

    fprintf(stderr, "%s", log);
    free(log);
}

/**
 * @brief Creates a shader from source code.
 * @param source The source code of the shader.
 * @param type The type of shader (GL_VERTEX_SHADER or GL_FRAGMENT_SHADER).
 * @return The ID of the created shader, or 0 if there was an error.
 */
inline GLuint create_shader_from_string(const char* source, GLenum type)
{
    GLuint res = glCreateShader(type);
    const char* sources[] = { source };
    glShaderSource(res, 1, sources, NULL);
    glCompileShader(res);
    GLint compile_ok = GL_FALSE;
    glGetShaderiv(res, GL_COMPILE_STATUS, &compile_ok);
    if (compile_ok == GL_FALSE)
    {
        fprintf(stderr, "Error compiling shader of type %d!\n", type);
        print_log(res);
        glDeleteShader(res);
        return 0;
    }

    return res;
}

/**
 * @brief Creates a shader program from vertex and fragment shader source code.
 * @param vs The source code for the vertex shader.
 * @param fs The source code for the fragment shader.
 * @return The ID of the created shader program, or 0 if there was an error.
 */
inline GLuint create_shader_program(const char* vs, const char* fs)
{
    GLuint vsId = create_shader_from_string(vs, GL_VERTEX_SHADER);
    GLuint fsId = create_shader_from_string(fs, GL_FRAGMENT_SHADER);
    assert(vsId != 0 && fsId != 0);

    GLuint programId = glCreateProgram();
    glAttachShader(programId, vsId);
    glAttachShader(programId, fsId);
    glBindFragDataLocation(programId, 0, "color");
    glLinkProgram(programId);

    glDeleteShader(vsId);
    glDeleteShader(fsId);

    GLint status = GL_FALSE;
    glGetProgramiv(programId, GL_LINK_STATUS, &status);
    assert(status != GL_FALSE);

    return programId;
}


#endif //BOX3D_GL_COMMON_HPP
