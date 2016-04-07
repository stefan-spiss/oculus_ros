/*
 * ShaderProgram.h
 *
 *  Created on: Oct 26, 2015
 *      Author: steve
 */
#pragma once

#include <string>
#include <vector>
#include <GL/gl.h>
#include <glm/glm.hpp>
#include "Shader.h"

namespace Render {

class ShaderProgram {
public:
	ShaderProgram(std::string &name);
	~ShaderProgram();
	void attachShader(const Shader &shader);
	void link();
	void bind() const;
	void unbind() const;
	GLint getUniformLocation(const std::string &name) const;
	GLint getAttributeLocation(const std::string &name) const;
	void setUniform1f(const std::string &name, GLfloat v) const;
	void setUniform1i(const std::string &name, GLint v) const;
	void setUniform2f(const std::string &name, glm::vec2 v) const;
	void setUniform4f(const std::string &name, glm::vec4 v) const;
	void setUniformMatrix4fv(const std::string &name, glm::mat4 m) const;
	void vertexAttribPointer(const std::string &attribName, GLint size,
			GLenum type, bool normalize, GLsizei stride,
			const GLvoid *data) const;
	GLuint getProgramId() const;
	const std::string &getName() const;

private:
	GLuint shaderProgramId;
	std::string name;
	std::vector<Shader> shaders;
};

}
