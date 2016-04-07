/*
 * Shader.h
 *
 *  Created on: Oct 26, 2015
 *      Author: steve
 */

#pragma once

#include <GL/gl.h>
#include <string>

namespace Render {

class Shader {
public:
	enum ShaderType {
		VERTEX, FRAGMENT
	};
	Shader(const std::string &name, const std::string &filename,
			ShaderType type);
	ShaderType const getType() const;
	GLuint getShaderId() const;
	const std::string &getName() const;
	static void show_info_log(GLuint object, PFNGLGETSHADERIVPROC glGet__iv,
			PFNGLGETSHADERINFOLOGPROC glGet__InfoLog) {
		GLint log_length;
		char *log;

		glGet__iv(object, GL_INFO_LOG_LENGTH, &log_length);
		log = (char *) malloc(log_length);
		glGet__InfoLog(object, log_length, NULL, log);
		fprintf(stderr, "%s", log);
		free(log);
	}

private:
	std::string name;
	ShaderType type;
	GLuint shaderId;

	GLuint load(const std::string &filename);
};

}
