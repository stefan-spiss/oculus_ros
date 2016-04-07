# oculus_ros
ROS package providing nodes to stream a video of a camera to the Oculus Rift and to publish the Oculus Rift sensor values over ROS.

The Oculus SDK 0.3.2 version from jherico which can be found at [https://github.com/jherico/OculusSDK/tree/0.3.x](https://github.com/jherico/OculusSDK/tree/0.3.x) was used and adapted for this ROS package.
For the Oculus SDK license see the LICENSEOCULUS.TXT file.

Additionally, for the **ros_driver** node code from the ROS node from Takashi Ogura was used. This can be found here: [https://github.com/OTL/oculus](https://github.com/OTL/oculus)

Finally, the distortion correction was done with help from the work from Scott A. Kuhl, which is available at [http://www.cs.mtu.edu/~kuhl/pincushion-distortion-correction.html](http://www.cs.mtu.edu/~kuhl/pincushion-distortion-correction.html). 

## Nodes
### ros_driver
  Node connects to the Oculus Rift and publishes information and sensor values. The topics for that are:
  - for information about the Oculus Rift: **/oculus/hmd_info**
  - for sensor values when available: **/oculus/orientation**
  Additionally **tf** data is also published when a sensor is available.
  To change the frequency the data is published ROS parameters are used. There is a example launch-file available to see how that works.
### ros_viewer
  Node subscribes to camera topic and renders the video for the Oculus Rift. If the topic **/oculus/hmd_info** is available it is directly viewed on the Oculus, otherwise it is rendered to the normal screen.
  To change the camera topic ROS parameters are used. There is a example launch-file available to see how that works.
  When the node is running, the distortion can be changed with the **arrow-keys** **left** and **right**, it can be zoomed in and out with the **arror-keys** **up** and **down** and finally the **j** and **k** key is mapped to change the distance between both images.
