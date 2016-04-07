#include <string>
#include <iostream>
#include <ros/ros.h>
#include <oculus_ros/OculusDriver.h>

#define PROGRAM_NAME "oculus_driver"

int main(int argc, char** argv) {
	ros::init(argc, argv, PROGRAM_NAME);
	ros::NodeHandle node;
	OculusDriver::OculusDriver oculus(node);
	ros::NodeHandle local_node("~");

	double frequency(10.0);

	local_node.getParam("frequency", frequency);
	ros::Rate r(frequency);
	if (oculus.isInitialized()) {
		while (ros::ok()) {
			oculus.publish();
			r.sleep();
		}
	} else {
		ROS_ERROR("Oculus Rift not found");
	}

	return 0;
}
