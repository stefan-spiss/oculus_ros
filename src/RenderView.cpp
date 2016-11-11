/*
 * RenderView.cpp
 *  Created on: Oct 26, 2015
 *      Author: steve
 */

#include <cstdlib>
#include <GL/glew.h>
#include <GL/gl.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <glm/glm.hpp>
#define GLM_FORCE_RADIANS
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <oculus_ros/RenderView.h>
#include <ros/package.h>

namespace Render {

RenderView::RenderView(SDL_Window *window, SDL_GLContext &context,
		unsigned horizontal_resolution, unsigned vertical_resolution) :
		horizontal_resolution(horizontal_resolution), vertical_resolution(
				vertical_resolution), distortion_constant(
				DEFAULT_DISTORTION_CONSTANT), scale(1.2), image_shift_constant(
				DEFAULT_IMAGE_SHIFT_CONSTANT) {
	initialize(window, context);
	increaseInterpupillarDistance(0.0);
}

RenderView::~RenderView() {
	glDeleteBuffers(1, &vbo);
}

void RenderView::initialize(SDL_Window *window, SDL_GLContext &context) {
	context = SDL_GL_CreateContext(window);
	GLenum ret;

	glewExperimental = GL_TRUE;
	ret = glewInit();

	if (ret != GLEW_OK) {
		std::cerr << "Error: " << glewGetErrorString(ret);
		exit(EXIT_FAILURE);
	}

	if (!GLEW_VERSION_2_0) {
		std::cerr << "OpenGL 2.0 not available.";
		exit(EXIT_FAILURE);
	}

	// VSYNC
	SDL_GL_SetSwapInterval(1);

	// Setup shaders
	std::string vs_shader_path = ros::package::getPath("oculus_ros").append(
			"/shaders/defaultVs.glsl");
	std::string fs_shader_path = ros::package::getPath("oculus_ros").append(
			"/shaders/defaultFs.glsl");

	std::string name = "default";
	shaderProgram.reset(new ShaderProgram(name));
	shaderProgram->attachShader(
			Shader("vertex", vs_shader_path, Shader::VERTEX));
	shaderProgram->attachShader(
			Shader("fragment", fs_shader_path, Shader::FRAGMENT));
	shaderProgram->link();
	shaderProgram->bind();

	//all the texture calls from addCVTexture method
	// creates texture, returns texture name
	glGenTextures(1, &texture);

	// binds the texture to GL_TEXTURE_2D
	glBindTexture(GL_TEXTURE_2D, texture);

	// set texture parameters, how the should get sampled, ...
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

}

GLuint RenderView::setupBuffer(GLenum target, const void *buffer_data,
		GLsizei buffer_size) {
	GLuint buffer;

	// create buffer, returns buffer object name
	glGenBuffers(1, &buffer);

	// binds the buffer to a specific target
	glBindBuffer(target, buffer);

	// creates a new data store for the buffer object and fills it with buffer_data
	glBufferData(target, buffer_size, buffer_data, GL_STATIC_DRAW);

	return buffer;
}

void RenderView::setupDrawing2DImage() {
	// setup drawing primitives

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	float aspectRatio = (float) texture_width / (float) texture_height;

	std::vector<GLfloat> vertex_buffer_data;
	getTexturePlane(1, aspectRatio, vertex_buffer_data);

	/*// Print vertex buffer data values:
	 std::cerr << std::endl << "****************************" << std::endl;
	 GLfloat* p = &vertex_buffer_data.front();
	 for (unsigned i = 0; i < vertex_buffer_data.size(); i += 5) {
	 std::cerr << p[i] << ", " << p[i + 1] << ", " << p[i + 2] << std::endl;
	 std::cerr << p[i + 3] << ", " << p[i + 4] << std::endl;
	 }
	 */

	vbo = setupBuffer(GL_ARRAY_BUFFER, &vertex_buffer_data.front(),
			vertex_buffer_data.size() * sizeof(GLfloat));

	shaderProgram->vertexAttribPointer("position", 3, GL_FLOAT, GL_FALSE,
			5 * sizeof(GLfloat), 0);
	shaderProgram->vertexAttribPointer("texcoord", 2, GL_FLOAT, GL_FALSE,
			5 * sizeof(GLfloat), (const GLvoid *) (3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(shaderProgram->getAttributeLocation("position"));
	glEnableVertexAttribArray(shaderProgram->getAttributeLocation("texcoord"));

	// Setup texture:
	glActiveTexture(GL_TEXTURE0);
	shaderProgram->setUniform1i("srcTex", 0);
	glBindTexture(GL_TEXTURE_2D, texture);

	// Setup scale and distortion constant
	shaderProgram->setUniform1f("k1", distortion_constant);
	shaderProgram->setUniform2f("texScale", glm::vec2(scale, scale));

	// Other GL stuff
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glEnable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
}

void RenderView::addCVTexture(cv::Mat &image) {
	cv::flip(image, image, 0);

	if (texture_height != image.rows || texture_width != image.cols) {
		texture_height = image.rows;
		texture_width = image.cols;
		this->setupDrawing2DImage();
	}

	// allocate memory for a texture and fill it with pixels
	glTexImage2D(
	GL_TEXTURE_2D,		// Type of texture
			0,					// Pyramid level for mip-mapping (0 top level)
			GL_RGB,				// Internal color format to convert to
			texture_width,			// Image width
			texture_height,			// Image height
			0,					// Border width in pixels
			GL_BGR,				// Input image format
			GL_UNSIGNED_BYTE,	// Image data type
			image.ptr()			// The actual data
			);

	//std::cerr << "cols " << texture_width << " rows " << texture_height << std::endl;

}

// just translation inward
void RenderView::display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	shaderProgram->bind();

	glBindVertexArray(vao);

	shaderProgram->setUniformMatrix4fv("viewMat",
			glm::transpose(leftViewAdjust));

	glViewport(0, 0, horizontal_resolution / 2, vertical_resolution);
	glDrawArrays(GL_TRIANGLES, 0, 6);

	shaderProgram->setUniformMatrix4fv("viewMat",
			glm::transpose(rightViewAdjust));

	glViewport(horizontal_resolution / 2, 0, horizontal_resolution / 2,
			vertical_resolution);
	glDrawArrays(GL_TRIANGLES, 0, 6);
}

void RenderView::setResolution(unsigned width, unsigned height) {
	horizontal_resolution = width;
	vertical_resolution = height;
}

void RenderView::increaseDistortionConstant(float k1) {
	distortion_constant += k1;
	shaderProgram->setUniform1f("k1", distortion_constant);
//	std::cout << "distortion constant: " << this->distortion_constant << std::endl;
}

void RenderView::increaseScale(float scale) {
	this->scale += scale;
	shaderProgram->setUniform2f("texScale",
			glm::vec2(this->scale, this->scale));
//	std::cout << "scale: " << this->scale << std::endl;
}

void RenderView::increaseInterpupillarDistance(float increase_factor) {
	image_shift_constant += increase_factor;
//	interpupillar_distance /= 20;
	leftViewAdjust = glm::translate(glm::vec3(image_shift_constant, 0, 0));
	rightViewAdjust = glm::translate(glm::vec3(-image_shift_constant, 0, 0));
}

void RenderView::getTexturePlane(float scale, float aspectRatio,
		std::vector<GLfloat>& vertices) {
	// two vectors along the plane, u in x direction, v in y direction
	glm::vec3 u(scale, 0, 0);
	glm::vec3 v(0, scale, 0);
	if (aspectRatio > 1) {
		v.y /= aspectRatio;
	} else {
		u.x *= aspectRatio;
	}
	/*	// Print plane vectors and aspectRatio:
	 std::cerr << aspectRatio << std::endl;
	 std::cerr << u.x << " " << u.y << " " << u.z << std::endl;
	 std::cerr << v.x << " " << v.y << " " << v.z << std::endl;
	 */

	glm::vec3 point(0, 0, 0);	// point on plane
	glm::vec3 pos = point - u - v; // position at the moment => start position
	glm::vec3 ustep = u * 2.0f; // step to take in x direction
	glm::vec3 vstep = v * 2.0f; // step to take in y direction

	vertices.push_back(pos.x);
	vertices.push_back(pos.y);
	vertices.push_back(pos.z);
	vertices.push_back(0.0);
	vertices.push_back(0.0);

	pos += ustep;
	vertices.push_back(pos.x);
	vertices.push_back(pos.y);
	vertices.push_back(pos.z);
	vertices.push_back(1.0);
	vertices.push_back(0.0);

	pos += vstep;
	vertices.push_back(pos.x);
	vertices.push_back(pos.y);
	vertices.push_back(pos.z);
	vertices.push_back(1.0);
	vertices.push_back(1.0);

	pos -= ustep;
	pos -= vstep;
	vertices.push_back(pos.x);
	vertices.push_back(pos.y);
	vertices.push_back(pos.z);
	vertices.push_back(0.0);
	vertices.push_back(0.0);

	pos += ustep;
	pos += vstep;
	vertices.push_back(pos.x);
	vertices.push_back(pos.y);
	vertices.push_back(pos.z);
	vertices.push_back(1.0);
	vertices.push_back(1.0);

	pos -= ustep;
	vertices.push_back(pos.x);
	vertices.push_back(pos.y);
	vertices.push_back(pos.z);
	vertices.push_back(0.0);
	vertices.push_back(1.0);
	/*	// Print the plane vertices and texture coords
	 std::cerr << std::endl << "************" << std::endl;
	 int start = 3;
	 bool was3 = true;
	 for (int i = 0; i < vertices.size(); i++) {
	 std::cerr << vertices[i] << ", ";
	 if (--start == 0) {
	 if (was3) {
	 start = 2;
	 was3 = false;
	 } else {
	 start = 3;
	 was3 = true;
	 }
	 std::cerr << std::endl;
	 }
	 }
	 */
}

}
