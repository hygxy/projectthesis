cmake_minimum_required (VERSION 2.8)
project (objectdetection)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs  sensor_msgs cv_bridge image_transport message_generation message_filters)

add_message_files(
   FILES
   Location.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs  
 )



find_package( OpenCV REQUIRED)


catkin_package(CATKIN_DEPENDS roscpp  std_msgs sensor_msgs cv_bridge image_transport message_runtime message_filters)

include_directories(${catkin_INCLUDE_DIRS} include)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(objectdetection  src/ObjectDetection.cpp)

target_link_libraries(objectdetection
  ${catkin_LIBRARIES} ${BOOST_LIBS} ${OpenCV_LIBS}
 )
 
 add_dependencies(objectdetection ${objectdetection_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})
