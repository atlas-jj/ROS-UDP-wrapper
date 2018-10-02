cmake_minimum_required(VERSION 2.8.3)
project(ros-udp_wrapper)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs cv_bridge image_transport message_generation)

find_package(OpenCV REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES signature_drawing
#)
include_directories ("${PROJECT_SOURCE_DIR}/include")
add_subdirectory ("${PROJECT_SOURCE_DIR}/lib")

include_directories(
include
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(wrapper src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(wrapper
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries (wrapper lib)
