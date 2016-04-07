#include <oculus_ros/OculusDriver.h>
#include <oculus_ros/HMDInfo.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>

using namespace OVR;

namespace OculusDriver {
OculusDriver::OculusDriver(ros::NodeHandle& node) :
		is_info_loaded(false), sensor_available(false), isInit(false), node(
				node) {
	isInit = initOculus();
}

OculusDriver::~OculusDriver() {
	ovrHmd_Destroy(hmd);
	ovr_Shutdown();
}

bool OculusDriver::initOculus() {
	std::cout << "Init oculus!" << std::endl;

	ovr_Initialize();
	hmd = ovrHmd_Create(0);
	if (!hmd) {
		std::cout << "No oculus detected!" << std::endl;
		return false;
	}

	ros::NodeHandle private_node("~");
	private_node.getParam("parent_frame", parent_frame);
	private_node.getParam("oculus_frame", oculus_frame);

	// Get more details about the HMD
	ovrHmd_GetDesc(hmd, &hmdDesc);

	if (hmdDesc.Type != ovrHmd_None) {
		is_info_loaded = true;
		hmd_pub = node.advertise<oculus_ros::HMDInfo>("/oculus/hmd_info", 10);
		std::cout << "Connected oculus: " << hmdDesc.ProductName << std::endl;
	}

	std::cout << std::endl << "Setting up sensors!" << std::endl;

	// Start the sensor which provides the Rift’s pose and motion.
	if (ovrHmd_StartSensor(hmd,
			ovrSensorCap_Orientation | ovrSensorCap_YawCorrection
					| ovrSensorCap_Position, ovrSensorCap_Orientation)) {
		sensor_available = true;
		pub = node.advertise<geometry_msgs::Quaternion>("/oculus/orientation",
				10);
		std::cout << "Oculus Sensor available" << std::endl;
	}
	return true;
}

bool OculusDriver::isInitialized() {
	return isInit;
}

ovrSensorState OculusDriver::getSensorState() {
	// Query the HMD for the sensor state at a given time.
	return ovrHmd_GetSensorState(hmd, 0.0);
}

int OculusDriver::getDisplayId() {
	return hmdDesc.DisplayId;
}

int OculusDriver::getHResolution() {
	if (is_info_loaded) {
		return hmdDesc.Resolution.w;
	}
	return 1024;
}

int OculusDriver::getVResolution() {
	if (is_info_loaded) {
		return hmdDesc.Resolution.h;
	}
	return 800;
}

double OculusDriver::getInterPupillarDistance() {
	return interPupillarDistance;
}

int OculusDriver::getWindowsPositionX() {
	return hmdDesc.WindowsPos.x;
}

int OculusDriver::getWindowsPositionY() {
	return hmdDesc.WindowsPos.y;
}

ovrHmdDesc OculusDriver::getDeviceDescription() {
	return hmdDesc;
}

void OculusDriver::publish() {
	ros::Time now = ros::Time::now();
	if (is_info_loaded) {
		oculus_ros::HMDInfo hmd_msg;
		hmd_msg.ProductName = hmdDesc.ProductName;
		hmd_msg.Manufacturer = hmdDesc.Manufacturer;
		hmd_msg.Version = hmdDesc.Type;
		hmd_msg.HResolution = hmdDesc.Resolution.w;
		hmd_msg.VResolution = hmdDesc.Resolution.h;
		hmd_msg.DisplayDeviceName = hmdDesc.DisplayDeviceName;
		hmd_msg.DisplayId = hmdDesc.DisplayId;
		hmd_msg.WindowsPosX = hmdDesc.WindowsPos.x;
		hmd_msg.WindowsPosY = hmdDesc.WindowsPos.y;
		hmd_msg.HmdCaps = hmdDesc.HmdCaps;
		hmd_msg.SensorCaps = hmdDesc.SensorCaps;
		hmd_msg.DistortionCaps = hmdDesc.DistortionCaps;
		hmd_msg.InterpupillaryDistance = interPupillarDistance;
		hmd_msg.DefaultEyeFovLeft = {hmdDesc.DefaultEyeFov[0].UpTan,
			hmdDesc.DefaultEyeFov[0].DownTan,
			hmdDesc.DefaultEyeFov[0].LeftTan,
			hmdDesc.DefaultEyeFov[0].RightTan};
		hmd_msg.DefaultEyeFovRight = {hmdDesc.DefaultEyeFov[1].UpTan,
			hmdDesc.DefaultEyeFov[1].DownTan,
			hmdDesc.DefaultEyeFov[1].LeftTan,
			hmdDesc.DefaultEyeFov[1].RightTan};
		hmd_msg.MaxEyeFovLeft = {hmdDesc.MaxEyeFov[0].UpTan,
			hmdDesc.MaxEyeFov[0].DownTan,
			hmdDesc.MaxEyeFov[0].LeftTan,
			hmdDesc.MaxEyeFov[0].RightTan};
		hmd_msg.MaxEyeFovRight = {hmdDesc.MaxEyeFov[1].UpTan,
			hmdDesc.MaxEyeFov[1].DownTan,
			hmdDesc.MaxEyeFov[1].LeftTan,
			hmdDesc.MaxEyeFov[1].RightTan};
		if (hmdDesc.EyeRenderOrder[0] == ovrEyeType::ovrEye_Left) {
			hmd_msg.EyeRenderOrder.push_back(0);
		} else if (hmdDesc.EyeRenderOrder[0] == ovrEyeType::ovrEye_Right) {
			hmd_msg.EyeRenderOrder.push_back(1);
		} else {
			hmd_msg.EyeRenderOrder.push_back(-1);
		}
		if (hmdDesc.EyeRenderOrder[1] == ovrEyeType::ovrEye_Left) {
			hmd_msg.EyeRenderOrder.push_back(0);
		} else if (hmdDesc.EyeRenderOrder[1] == ovrEyeType::ovrEye_Right) {
			hmd_msg.EyeRenderOrder.push_back(1);
		} else {
			hmd_msg.EyeRenderOrder.push_back(-1);
		}

		hmd_pub.publish(hmd_msg);
	}
	if (sensor_available) {
		geometry_msgs::Quaternion q_msg;
		hmdPose = ovrHmd_GetSensorState(hmd, 0.0).Recorded.Pose;
		q_msg.x = hmdPose.Orientation.x;
		q_msg.y = hmdPose.Orientation.y;
		q_msg.z = hmdPose.Orientation.z;
		q_msg.w = hmdPose.Orientation.w;

		pub.publish(q_msg);

		// tf
		tf::Transform transform;
		transform.setRotation(
				tf::Quaternion(q_msg.x, q_msg.y, q_msg.z, q_msg.w));
		br.sendTransform(
				tf::StampedTransform(transform, now, parent_frame,
						oculus_frame));
	}
}

}
