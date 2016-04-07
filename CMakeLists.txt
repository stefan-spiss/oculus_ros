cmake_minimum_required(VERSION 2.8.3)
project(oculus_ros)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  cv_bridge
  image_transport
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES HMDInfo.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES oculus
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp std_msgs tf
)

###################################
## oculus specific configuration ##
###################################

add_subdirectory(3rdParty/EDID)
set_target_properties(edid PROPERTIES FOLDER "3rdParty")

# The primary SDK artifact, a static library for Oculus access 
add_subdirectory(LibOVR)
set_target_properties(OculusVR PROPERTIES FOLDER "Oculus")

# Cross platform GL access.  Used by the C API and by
# the sample applications
add_definitions(-DGLEW_STATIC)
add_subdirectory(3rdParty/glew)
set_target_properties(glew PROPERTIES FOLDER "3rdParty")

add_subdirectory(Bindings/C)
set_target_properties(OVR_C PROPERTIES FOLDER "Oculus")

include_directories(3rdParty/glew/include)
include_directories(${OculusVR_SOURCE_DIR}/Include)
include_directories(${OculusVR_SOURCE_DIR}/Src)
include_directories(${OculusVR_SOURCE_DIR}/Src/Kernel)
include_directories(${OculusVR_SOURCE_DIR}/Src/Util)
include_directories(Bindings/C/Include)

set(EXTRA_LIBS OVR_C)

# While the samples use the C API, they also use the 
# core SDK functionality quite a bit
list(APPEND EXTRA_LIBS OculusVR)
list(APPEND EXTRA_LIBS ${OVR_LIBRARIES})

# OpenGL, glew, glm, SDL2, OpenCV
#list(APPEND EXTRA_LIBS glew)
# find OpenGL
find_package(OpenGL REQUIRED)
list(APPEND EXTRA_LIBS ${OPENGL_LIBRARIES})
# find GLEW
#find_package(GLEW REQUIRED)
#list(APPEND EXTRA_LIBS ${GLEW_LIBRARIES})
# find GLM
find_package(GLM REQUIRED)
# find SDL2
find_package(SDL2 REQUIRED)
list(APPEND EXTRA_LIBS ${SDL2_LIBRARY})
# find OpenCV
find_package(OpenCV REQUIRED)
list(APPEND EXTRA_LIBS ${OpenCV_LIBS})


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GLM_INCLUDE_DIR}
  #${GLEW_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIR}
  ${OpenCV_LIBS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(oculus_ros oculus_ros_gencpp)

set(SOURCE_FILES
		src/RenderView.cpp 
		src/OculusDriver.cpp 
		src/ShaderProgram.cpp 
		src/Shader.cpp
)

## Declare a C++ executable
add_executable(ros_driver ${SOURCE_FILES} src/RosDriver.cpp)
add_executable(ros_viewer ${SOURCE_FILES} src/RosViewer.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ros_driver ${EXTRA_LIBS})
target_link_libraries(ros_driver ${catkin_LIBRARIES})

target_link_libraries(ros_viewer ${EXTRA_LIBS})
target_link_libraries(ros_viewer ${catkin_LIBRARIES})