#include "utils.hpp"

#include <cstdio>

#include <iostream>
#include <fstream>
#include <stdexcept>

void _CheckOpenGLError(const char* stmt, const char* fname, int line) {
  GLenum err = glGetError();
  while (err != GL_NO_ERROR) {
    fprintf(stderr, "OpenGL error %08x, at %s:%i - for %s\n", err, fname, line,
            stmt);
    err = glGetError();
  }
}