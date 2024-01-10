#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <memory>

#include <glad/glad.h>

/*
 * https://stackoverflow.com/questions/11256470/define-a-macro-to-facilitate-opengl-command-debugging
 * */
void _CheckOpenGLError(const char* stmt, const char* fname, int line);

#define GL_CHECK(stmt)                            \
  do {                                            \
    stmt;                                         \
    _CheckOpenGLError(#stmt, __FILE__, __LINE__); \
  } while (0)
#define GL_CHECK_ERROR() \
  _CheckOpenGLError("GL_CHECK_ERROR", __FILE__, __LINE__);

#endif