cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(8Steps3DSelfie.cpp)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
if(APPLE)
	add_executable (8Steps3DSelfie MACOSX_BUNDLE 8Steps3DSelfie)
else()
	
	add_executable (8Steps3DSelfie 8Steps3DSelfie.cpp)
endif()

target_link_libraries (8Steps3DSelfie ${PCL_LIBRARIES})
