/*
 * Viewer.cpp
 *
 *  Created on: Nov 7, 2015
 *      Author: steve
 */
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <oculus_ros/HMDInfo.h>
#include <oculus_ros/RenderView.h>
#include <SDL2/SDL.h>

oculus_ros::HMDInfoPtr hmd_info;
bool hmd_connected = false;
SDL_Window *mainwindow; // Our window handle
SDL_GLContext maincontext; // Our opengl context handle
cv::Mat texture;
Render::RenderView* renderView;
ros::Time begin;
int renderedFrames = 0;
int receivedFrames = 0;

void HMDInfoCallback(const oculus_ros::HMDInfoPtr& info) {
	hmd_info = info;
	if (hmd_connected == false) {
		if (!mainwindow) {
			return;
		}
		SDL_SetWindowSize(mainwindow, hmd_info->HResolution,
				hmd_info->VResolution);
		SDL_SetWindowPosition(mainwindow, hmd_info->WindowsPosX,
				hmd_info->WindowsPosY);
		//SDL_SetWindowFullscreen(mainwindow, SDL_WINDOW_FULLSCREEN_DESKTOP);

		renderView->setResolution(hmd_info->HResolution, hmd_info->VResolution);
	}
	hmd_connected = true;

	//std::cerr << "hmd callback" << hmd_info->display_id << std::endl;
}

void CameraImageCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr ptr;

	try {
		ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv bridge exception: %s", e.what());
	}

	renderView->addCVTexture(ptr->image);

	receivedFrames++;

	//std::cerr << "cols: " << msg->width << " " << ptr->image.cols  << " rows: " << msg->height << std::endl;
	//std::cerr << "image callback" << std::endl;
}

void checkFrames() {
	if((ros::Time::now() - begin).sec >= 1) {
		std::cout << "received fps: " << receivedFrames << std::endl;
		std::cout << "rendered fps: " << renderedFrames << std::endl;
		begin = ros::Time::now();
		receivedFrames = 0;
		renderedFrames = 0;
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "oculus_image_viewer");

	ros::NodeHandle node;
	image_transport::ImageTransport image_transport(node);
	
	
	std::string camera_topic = "/ardrone/front/image_raw";
	ros::NodeHandle local_node("~");
	local_node.getParam("camera_topic", camera_topic);

	ros::Subscriber sub_hmd_info = node.subscribe("/oculus/hmd_info", 1,
			&HMDInfoCallback);

	if (SDL_Init(SDL_INIT_VIDEO) < 0) { // Initialize SDL's Video subsystem
		std::cout << "Unable to initialize SDL";
		return 1;
	}

	// Request opengl 4.4 context.
	SDL_GL_SetAttribute(SDL_GL_ACCELERATED_VISUAL, 1);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

	if (hmd_connected) {
		mainwindow = SDL_CreateWindow("oculus_viewer",
				SDL_WINDOWPOS_CENTERED_DISPLAY(hmd_info->DisplayId),
				SDL_WINDOWPOS_CENTERED_DISPLAY(hmd_info->DisplayId),
				hmd_info->HResolution, hmd_info->VResolution,
				SDL_WINDOW_FULLSCREEN_DESKTOP | SDL_WINDOW_OPENGL);
	} else {

		// Create our window centered
		mainwindow = SDL_CreateWindow("oculus_viewer", SDL_WINDOWPOS_CENTERED,
		SDL_WINDOWPOS_CENTERED, 1280, 800,
				SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN); // | SDL_WINDOW_RESIZABLE
	}

	if (!mainwindow) { // Die if creation failed
		std::cout << "SDL Error: " << SDL_GetError() << std::endl;
		SDL_Quit();
		return 1;
	}

	bool quit = false;

//	image_transport::Subscriber sub_camera_image = image_transport.subscribe(
//			"/ardrone/front/image_raw", 1, &CameraImageCallback);
	image_transport::Subscriber sub_camera_image = image_transport.subscribe(
			camera_topic, 1, &CameraImageCallback);

	if (hmd_connected) {
		renderView = new Render::RenderView(mainwindow, maincontext,
				hmd_info->HResolution, hmd_info->VResolution);
	} else {
		renderView = new Render::RenderView(mainwindow, maincontext, 1280, 800);
	}

	SDL_Event event;
	while (!quit && ros::ok()) {
		checkFrames();
//		if (hmd_connected) {
//			renderView->display();
//		} else {
//			renderView->display();
//		}
		renderView->display();
		SDL_GL_SwapWindow(mainwindow);
		while (SDL_PollEvent(&event)) {
			switch (event.type) {
			case SDL_QUIT:
				quit = true;
				break;
				/*case SDL_WINDOWEVENT:
				 if (event.window.event == SDL_WINDOWEVENT_RESIZED)
				 renderView->reshape(event.window.data1, event.window.data2);
				 break;
				 */
			case SDL_KEYDOWN:
				switch (event.key.keysym.sym) {
				case SDLK_UP:
					renderView->increaseScale(0.05);
					break;
				case SDLK_DOWN:
					renderView->increaseScale(-0.05);
					break;
				case SDLK_LEFT:
					renderView->increaseDistortionConstant(-0.05);
					break;
				case SDLK_RIGHT:
					renderView->increaseDistortionConstant(0.05);
					break;
				case SDLK_j:
					renderView->increaseInterpupillarDistance(-0.01);
					break;
				case SDLK_k:
					renderView->increaseInterpupillarDistance(0.01);
					break;
				}
				break;
			}
		}
		renderedFrames++;
		ros::spinOnce();
	}
	// Delete our opengl context, destroy our window, and shutdown SDL
	SDL_GL_DeleteContext(maincontext);
	SDL_DestroyWindow(mainwindow);
	SDL_Quit();
	delete (renderView);

	return 0;
}

//
//namespace oculus_viewer {
//Viewer::Viewer(std::string& program_name, std::string& camera_info_topic,
//		std::string& camera_image_topic) :
//		node(), program_name(program_name), camera_info_topic(
//				camera_info_topic), camera_image_topic(camera_image_topic), hmd_connected(
//				false), camera_connected(false), quit(false) {
//	sub_hmd_info = node.subscribe("/oculus/hmd_info", 1,
//			&Viewer::HMDInfoCallback, this);
//	sub_hmd_info = node.subscribe(camera_info_topic, 1,
//			&Viewer::CameraInfoCallback, this);
//}
//Viewer::~Viewer() {
//	SDL_GL_DeleteContext(maincontext);
//	SDL_DestroyWindow(mainwindow);
//	SDL_Quit();
//}
//
//bool Viewer::initialize() {
//	if (SDL_Init(SDL_INIT_VIDEO) < 0) { // Initialize SDL's Video subsystem
//		std::cout << "Unable to initialize SDL";
//		return false;
//	}
//
//	// Request opengl 4.4 context.
//	SDL_GL_SetAttribute(SDL_GL_ACCELERATED_VISUAL, 1);
//	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
//	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
//
//	if (hmd_connected) {
//		mainwindow = SDL_CreateWindow(program_name.c_str(),
//				SDL_WINDOWPOS_CENTERED_DISPLAY(hmd_info->display_id),
//				SDL_WINDOWPOS_CENTERED_DISPLAY(hmd_info->display_id),
//				hmd_info->horizontal_resolution, hmd_info->vertical_resolution,
//				SDL_WINDOW_FULLSCREEN_DESKTOP | SDL_WINDOW_OPENGL);
//	} else {
//		// Create our window centered
//		mainwindow = SDL_CreateWindow(program_name.c_str(),
//		SDL_WINDOWPOS_CENTERED,
//		SDL_WINDOWPOS_CENTERED, 1280, 800,
//				SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN); // | SDL_WINDOW_RESIZABLE
//	}
//
//	if (!mainwindow) { // Die if creation failed
//		std::cout << "SDL Error: " << SDL_GetError() << std::endl;
//		SDL_Quit();
//		return false;
//	}
//
//	renderer.RenderView(mainwindow, ma)
//
//	cv::Mat texture0 =
//				cv::imread(
//						"/home/steve/Documents/Uni/Bachelorwork/OculusImageViewer/OculusImageViewer/hello3.JPG");
//	renderer.addCVTexture(texture0);
//	return true;
//}
//
//void Viewer::shutdown() {
//	quit = true;
//}
//
//void Viewer::startRendering() {
//	renderer.display();
//	/*SDL_Event event;
//	while (!quit) {
//		renderer.display();
//		SDL_GL_SwapWindow(mainwindow);
//		ros::spinOnce();
//		while (SDL_PollEvent(&event)) {
//			switch (event.type) {
//			case SDL_QUIT:
//				quit = true;
//				break;
//			case SDL_WINDOWEVENT:
//				if (event.window.event == SDL_WINDOWEVENT_RESIZED)
//					renderer.reshape(event.window.data1, event.window.data2);
//				break;
//			}
//		}
//	}*/
//}
//
//void Viewer::HMDInfoCallback(const oculus_msgs::HMDInfoPtr& info) {
//	hmd_info = info;
//	hmd_connected = true;
//}
//
//void Viewer::CameraInfoCallback(const sensor_msgs::CameraInfoPtr& info) {
//	camera_info = info;
//	camera_connected = true;
//}
//
//bool Viewer::isQuit() const {
//	return quit;
//}
//
//SDL_Window* Viewer::getMainwindow() const {
//	return mainwindow;
//}
//
//}
//
//int main(int argc, char** argv) {
//	ros::init(argc, argv, "oculus_image_viewer");
//
//	std::string cam_info = "los";
//	std::string cam_image = "lol";
//	std::string prog_name = "oculus viewer";
//	try {
//		oculus_viewer::Viewer viewer = oculus_viewer::Viewer(prog_name,
//				cam_info, cam_image);
//
//		if (viewer.initialize()) {
//			SDL_Event event;
//			while (!viewer.isQuit() && ros::ok()) {
//				viewer.startRendering();
//				SDL_GL_SwapWindow (viewer.getMainwindow());
//				ros::spinOnce();
//				while (SDL_PollEvent(&event)) {
//					switch (event.type) {
//					case SDL_QUIT:
//						viewer.shutdown();
//						break;
//					case SDL_WINDOWEVENT:
//						if (event.window.event == SDL_WINDOWEVENT_RESIZED)
//							//viewer.getRenderer().reshape(event.window.data1, event.window.data2);
//						break;
//					}
//				}
//			}
//		}
//	} catch (ros::Exception& e) {
//		ROS_ERROR("ros error: %s", e.what());
//	} catch (...) {
//		ROS_FATAL("unexpected error");
//	}
//}

