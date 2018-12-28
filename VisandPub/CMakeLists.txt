cmake_minimum_required (VERSION 2.8)
project (KinectHelper)
set(INCLUDE "${CMAKE_SOURCE_DIR}/include")
set(FREENECT2_INCLUDE_DIRS "/home/hy/freenect2/include")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")
set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(CMAKE_BUILD_TYPE DEBUG)

find_package( OpenCV REQUIRED)
find_package( Freenect2 REQUIRED)
find_package( PCL 1.8 REQUIRED)



find_package(catkin REQUIRED COMPONENTS   roscpp std_msgs pcl_conversions  pcl_ros
             sensor_msgs cv_bridge image_transport tf
             message_generation )

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs pcl_conversions  pcl_ros
               sensor_msgs cv_bridge image_transport tf
              cmake_modules
               )
include_directories(${catkin_INCLUDE_DIRS} include)


include_directories(${PCL_INCLUDE_DIRS})
include_directories(${INCLUDE})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${FREENECT2_INCLUDE_DIRS})


link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(KinectHelper  KinectHelper.cpp  main.cpp)

target_link_libraries(KinectHelper ${PCL_LIBRARIES}  ${BOOST_LIBS} ${OpenCV_LIBS} ${FREENECT2_LIBRARY} ${catkin_LIBRARIES})



