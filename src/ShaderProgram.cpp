/*
 * ShaderProgram.cpp
 *
 *  Created on: Oct 26, 2015
 *      Author: steve
 */

#include <cstdlib>
#include <iostream>
#include <GL/glew.h>
#define GLM_FORCE_RADIANS
#include <glm/ext.hpp>
#include <oculus_ros/ShaderProgram.h>

namespace Render {

ShaderProgram::ShaderProgram(std::string &name) :
		name(name) {
	shaderProgramId = glCreateProgram();
}

ShaderProgram::~ShaderProgram() {
	unbind();
	glDeleteProgram(shaderProgramId);
}

void ShaderProgram::attachShader(const Shader &shader) {
	glAttachShader(shaderProgramId, shader.getShaderId());
	shaders.push_back(shader);
}

void ShaderProgram::link() {
	GLint program_ok;

	glLinkProgram(shaderProgramId);

	glGetProgramiv(shaderProgramId, GL_LINK_STATUS, &program_ok);
	if (!program_ok) {
		std::cerr << "Failed to compile " << name.c_str() << ":\n";
		Shader::show_info_log(shaderProgramId, glGetShaderiv,
		glGetShaderInfoLog);
		glDeleteProgram(shaderProgramId);
		exit(EXIT_FAILURE);
	}
}

void ShaderProgram::bind() const {
	glUseProgram(shaderProgramId);
}

void ShaderProgram::unbind() const {
	glUseProgram(0);
}

GLint ShaderProgram::getUniformLocation(const std::string &name) const {
	GLint loc = glGetUniformLocation(shaderProgramId, name.c_str());
	if (loc < 0) {
		std::cerr << "Uniform location for " << name.c_str()
				<< "not found in program.";
		exit(EXIT_FAILURE);
	}
	return loc;
}

GLint ShaderProgram::getAttributeLocation(const std::string &name) const {
	GLint loc = glGetAttribLocation(shaderProgramId, name.c_str());
	if (loc < 0) {
		std::cerr << "Attribute location for " << name.c_str()
				<< "not found in program.";
		exit(EXIT_FAILURE);
	}
	return loc;
}

void ShaderProgram::setUniform1f(const std::string &name, GLfloat v) const {
	glUniform1f(getUniformLocation(name), v);
}

void ShaderProgram::setUniform1i(const std::string &name, GLint v) const {
	glUniform1i(getUniformLocation(name), v);
}

void ShaderProgram::setUniform2f(const std::string& name, glm::vec2 v) const {
	glUniform2f(getUniformLocation(name), v.x, v.y);
}

void ShaderProgram::setUniform4f(const std::string& name, glm::vec4 v) const {
	glUniform4f(getUniformLocation(name), v.x, v.y, v.z, v.w);
}


void ShaderProgram::setUniformMatrix4fv(const std::string& name,
		glm::mat4 m) const {
	glUniformMatrix4fv(getUniformLocation(name), 1, GL_TRUE, glm::value_ptr(m));
}

void ShaderProgram::vertexAttribPointer(const std::string &attribName,
		GLint size, GLenum type, bool normalize, GLsizei stride,
		const GLvoid *data) const {
	glVertexAttribPointer(getAttributeLocation(attribName), size, type,
			normalize, stride, data);
}

GLuint ShaderProgram::getProgramId() const {
	return shaderProgramId;
}

const std::string &ShaderProgram::getName() const {
	return name;
}

}
