#include "gl_shader.h"

#include <iostream>
#include <fstream>

namespace gl_shader {

	bool ReadFile(const char* file_path, std::string &src) {
		ofFile file(file_path);
		if (!file.exists()) {
			ofLogError("gl_shader::ReadFile") << "File doesn't exist : " << file_path;
			return false;
		}

		ofBuffer buffer = file.readToBuffer();
		src = buffer.getText();

		return true;
	}

	GLuint LoadProgram(const char* vertex_shader_file, const char* fragment_shader_file, const char* geometry_shader_file) {
		std::string vertex_shader_src, fragment_shader_src, geometry_shader_src;

		bool read_vertex = ReadFile(vertex_shader_file, vertex_shader_src);
		if (!read_vertex) {
			std::cerr << "Unable to read " << vertex_shader_file << std::endl;
			return GL_FALSE;
		}

		bool read_fragment = ReadFile(fragment_shader_file, fragment_shader_src);
		if (!read_fragment) {
			std::cerr << "Unable to read " << fragment_shader_file << std::endl;
			return GL_FALSE;
		}

		bool read_geometry = ReadFile(geometry_shader_file, geometry_shader_src);
		if (!read_geometry) {
			std::cerr << "Unable to read " << geometry_shader_file << std::endl;
			return GL_FALSE;
		}

		// Creation d'un Vertex Shader
		GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
		const char * vertex_shader_src_char = vertex_shader_src.c_str();
		glShaderSource(vertex_shader, 1, &(vertex_shader_src_char), 0);

		glCompileShader(vertex_shader);
		GLint compile_status;
		glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &compile_status);
		if (compile_status == GL_FALSE) {
			GLint logLength;
			glGetShaderiv(vertex_shader, GL_INFO_LOG_LENGTH, &logLength);

			char* log = new char[logLength];

			glGetShaderInfoLog(vertex_shader, logLength, 0, log);
			std::cerr << "Vertex Shader error:" << log << std::endl;
			std::cerr << vertex_shader_src << std::endl;

			delete[] log;
			return GL_FALSE;
		}

		// Creation d'un Fragment Shader
		GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
		const char * fragment_shader_src_char = fragment_shader_src.c_str();
		glShaderSource(fragment_shader, 1, &(fragment_shader_src_char), 0);

		glCompileShader(fragment_shader);
		glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &compile_status);
		if (compile_status == GL_FALSE) {
			GLint logLength;
			glGetShaderiv(fragment_shader, GL_INFO_LOG_LENGTH, &logLength);

			char* log = new char[logLength];

			glGetShaderInfoLog(fragment_shader, logLength, 0, log);
			std::cerr << "Fragment Shader error:" << log << std::endl;
			std::cerr << fragment_shader_src << std::endl;

			delete[] log;
			return GL_FALSE;
		}

		// Creation d'un Geometry Shader
		GLuint geometry_shader = glCreateShader(GL_GEOMETRY_SHADER);
		const char * geometry_shader_src_char = geometry_shader_src.c_str();
		glShaderSource(geometry_shader, 1, &(geometry_shader_src_char), 0);

		glCompileShader(geometry_shader);
		glGetShaderiv(geometry_shader, GL_COMPILE_STATUS, &compile_status);
		if (compile_status == GL_FALSE) {
			GLint logLength;
			glGetShaderiv(geometry_shader, GL_INFO_LOG_LENGTH, &logLength);

			char* log = new char[logLength];

			glGetShaderInfoLog(geometry_shader, logLength, 0, log);
			std::cerr << "Geometry Shader error:" << log << std::endl;
			std::cerr << geometry_shader_src << std::endl;

			delete[] log;
			return GL_FALSE;
		}

		GLuint program;

		// Creation d'un programme
		program = glCreateProgram();

		// Attachement des shaders au programme
		glAttachShader(program, vertex_shader);
		glAttachShader(program, geometry_shader);
		glAttachShader(program, fragment_shader);

		// Désallocation des shaders: ils ne seront réellement supprimés que lorsque le programme sera supprimé
		glDeleteShader(vertex_shader);
		glDeleteShader(geometry_shader);
		glDeleteShader(fragment_shader);

		// Edition de lien
		glLinkProgram(program);

		/// Vérification que l'édition de liens a bien fonctionnée (très important aussi !)
		GLint linkStatus;
		glGetProgramiv(program, GL_LINK_STATUS, &linkStatus);
		if (linkStatus == GL_FALSE) {
			GLint logLength;
			glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);

			char* log = new char[logLength];

			glGetProgramInfoLog(program, logLength, 0, log);
			std::cerr << "Program link error:" << log << std::endl;

			delete[] log;
			return GL_FALSE;
		}

		return program;
	}

} // namespace gl_shader