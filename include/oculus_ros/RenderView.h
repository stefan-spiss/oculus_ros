/*
 * RenderView.h
 *
 *  Created on: Oct 26, 2015
 *      Author: steve
 */
#pragma once

#include <oculus_ros/ShaderProgram.h>
#include <SDL2/SDL.h>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>

namespace Render {

class RenderView {

#define DEFAULT_DISTORTION_CONSTANT 0.2
#define DEFAULT_SCALE 0.95
#define DEFAULT_IMAGE_SHIFT_CONSTANT 0.05

public:
	RenderView(SDL_Window *window, SDL_GLContext &context,
			unsigned horizontal_resolution, unsigned vertical_resolution);
	~RenderView();
	void setupDrawing2DImage();
	void reshape(int width, int height);
	void display();
	void addCVTexture(cv::Mat &image);
	void setResolution(unsigned width, unsigned height);
	void increaseDistortionConstant(float k1);
	void increaseScale(float scale);
	void increaseInterpupillarDistance(float increase_factor);
private:
	std::shared_ptr<ShaderProgram> shaderProgram;
	GLuint vao;
	GLuint vbo;
	GLuint texture;
	unsigned texture_height;
	unsigned texture_width;
	unsigned horizontal_resolution;
	unsigned vertical_resolution;

	GLfloat distortion_constant;
	float scale;
	float image_shift_constant;
	glm::mat4 leftViewAdjust;
	glm::mat4 rightViewAdjust;

	void initialize(SDL_Window *window, SDL_GLContext &context);
	GLuint setupBuffer(GLenum target, const void *buffer_data,
			GLsizei buffer_size);
	void getTexturePlane(float scale, float aspectRatio,
			std::vector<GLfloat>& vertices);
};

}

