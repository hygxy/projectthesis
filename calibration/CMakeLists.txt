cmake_minimum_required (VERSION 2.8)
project (Libfreenect2OpenCV)
set(INCLUDE "${CMAKE_SOURCE_DIR}")
set(FREENECT2_INCLUDE_DIRS "/home/hy/freenect2/include")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")
set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(CMAKE_BUILD_TYPE DEBUG)

find_package( OpenCV REQUIRED)
find_package( Freenect2 REQUIRED)
find_package( PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${INCLUDE})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${FREENECT2_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(Libfreenect2OpenCV   Libfreenect2OpenCV.cpp  main.cpp)

target_link_libraries(Libfreenect2OpenCV ${PCL_LIBRARIES}  ${OpenCV_LIBS} ${FREENECT2_LIBRARY})



