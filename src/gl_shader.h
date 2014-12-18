// NOT USED
// Shader loading helper functions : vertex, fragment and geometry shaders reading
// and loading

#ifndef FLEXIBLE_SURFACE_AUGMENTATION_GL_SHADER_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_GL_SHADER_H_

#include <sstream>

#include "ofMain.h"

namespace gl_shader {

	bool ReadFile(const char* file_path, std::string &src);

	GLuint LoadProgram(const char* vertex_shader_file, const char* fragment_shader_file, const char* geometry_shader_file);

} // namespace gl_shader

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_UTILITIES_H_