/*
 * Shader.cpp
 *
 *  Created on: Oct 26, 2015
 *      Author: steve
 */
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <GL/glew.h>
#include <oculus_ros/Shader.h>

namespace Render {
Shader::Shader(const std::string &name, const std::string &filename,
		ShaderType type) :
		name(name), type(type) {
	switch (type) {
	case VERTEX:
		shaderId = glCreateShader(GL_VERTEX_SHADER);
		break;
	case FRAGMENT:
		shaderId = glCreateShader(GL_FRAGMENT_SHADER);
		break;
	}
	load(filename);
}

Shader::ShaderType const Shader::getType() const {
	return type;
}

GLuint Shader::getShaderId() const {
	return shaderId;
}
const std::string &Shader::getName() const {
	return name;
}

GLuint Shader::load(const std::string &filename) {
	std::ifstream shaderSrcFileHandle(filename);
	if (!shaderSrcFileHandle.is_open()) {
		std::cerr << "File not found " << filename.c_str() << "\n";
		exit(EXIT_FAILURE);
	}

	std::string source;
	source.assign((std::istreambuf_iterator<char>(shaderSrcFileHandle)),
			(std::istreambuf_iterator<char>()));
	shaderSrcFileHandle.close();

	const char *data = source.c_str();
	GLint shader_ok;

	glShaderSource(shaderId, 1, (const GLchar**) &data, NULL);

	glCompileShader(shaderId);

	glGetShaderiv(shaderId, GL_COMPILE_STATUS, &shader_ok);
	if (!shader_ok) {
		std::cerr << "Failed to compile " << filename.c_str() << ":\n";
		show_info_log(shaderId, glGetShaderiv, glGetShaderInfoLog);
		glDeleteShader(shaderId);
		exit(EXIT_FAILURE);
	}
	return shaderId;
}



}
