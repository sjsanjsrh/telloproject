cmake_minimum_required(VERSION 3.18)
project(orbslam3_live LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED ORB_SLAM3_ROOT)
	message(FATAL_ERROR "Set ORB_SLAM3_ROOT to the built ORB-SLAM3 source directory")
endif()

find_package(OpenCV REQUIRED)

add_executable(orbslam3_live main.cpp)

target_include_directories(orbslam3_live PRIVATE
	${ORB_SLAM3_ROOT}
	${ORB_SLAM3_ROOT}/include
	${ORB_SLAM3_ROOT}/include/CameraModels
	${ORB_SLAM3_ROOT}/Thirdparty/Sophus
)

target_link_directories(orbslam3_live PRIVATE
	${ORB_SLAM3_ROOT}/lib
	${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib
	${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib
)

target_link_libraries(orbslam3_live PRIVATE
	${OpenCV_LIBS}
	ORB_SLAM3
	DBoW2
	g2o
)
