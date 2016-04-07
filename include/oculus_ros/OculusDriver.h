#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "OVR.h"
#include "OVR_CAPI.h"
#include "OVR_DeviceConstants.h"

using namespace OVR;

namespace OculusDriver {
class OculusDriver {
public:
	OculusDriver(ros::NodeHandle& node);
	~OculusDriver();
	bool isInitialized();
	ovrSensorState getSensorState();
	int getDisplayId();
	int getHResolution();
	int getVResolution();
	double getInterPupillarDistance();
	int getWindowsPositionX();
	int getWindowsPositionY();
	ovrHmdDesc getDeviceDescription();
	void publish();
private:
	bool is_info_loaded;
	bool sensor_available;
	bool isInit;
	ovrHmd hmd;
	ovrHmdDesc hmdDesc;
	ovrPosef hmdPose;
//	ovrEyeRenderDesc eyeRenderDesc[2];
	//IPD in cm
	double interPupillarDistance = 6.5; // cm
	ros::NodeHandle node;
	ros::Publisher pub;
	ros::Publisher hmd_pub;
	tf::TransformBroadcaster br;
	std::string parent_frame;
	std::string oculus_frame;
	bool initOculus();
};
}
