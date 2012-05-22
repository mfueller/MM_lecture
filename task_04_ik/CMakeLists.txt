cmake_minimum_required(VERSION 2.6)

PROJECT(MM_TASK_04_IK)

set(CMAKE_BUILD_TYPE Debug)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin)


INCLUDE_DIRECTORIES(
  "/usr/include/eigen3"
)

LINK_DIRECTORIES(
 
)

ADD_EXECUTABLE(mm_task04_ik
	src/MM_IK.cpp
	src/HomogenousTransform.cpp
	src/InverseKinematics.cpp
	src/ForwardKinematics.cpp
	src/SDLSSolver.cpp
)

TARGET_LINK_LIBRARIES(mm_task04_ik

)