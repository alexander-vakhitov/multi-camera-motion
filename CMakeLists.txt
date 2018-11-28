
cmake_minimum_required( VERSION 2.6 )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

if (NOT CMAKE_BUILD_TYPE)
	message(STATUS "No build type selected, default to Release")
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "build type" FORCE)
#	SET(CMAKE_BUILD_TYPE DEBUG)
endif()

#add_definitions(-D_GLIBCXX_DEBUG)

project( multi-camera-motion )

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

include_directories( include /usr/local/include )

add_library( multi-camera-motion
include/multi-camera-motion/approx_relpose_generalized_fast.h
include/multi-camera-motion/approx_relpose_axial.h
include/multi-camera-motion/approx_relpose_axial_fast.h

src/approx_relpose_generalized_fast_computeA.h
src/approx_relpose_axial_fast_computeA.h
src/approx_relpose_generalized_fast_computeA.cpp
src/approx_relpose_generalized_fast.cpp
src/approx_relpose_axial.cpp
src/approx_relpose_axial_fast_computeA.cpp
src/approx_relpose_axial_fast.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
#src/approx_pose_mex.cpp

install(DIRECTORY include/multi-camera-motion DESTINATION include/multi-camera-motion
        FILES_MATCHING PATTERN *.h)
install(TARGETS multi-camera-motion DESTINATION lib)

add_executable(test-mcm
		include/multi-camera-motion/approx_relpose_generalized_fast.h
		include/multi-camera-motion/approx_relpose_axial.h
		include/multi-camera-motion/approx_relpose_axial_fast.h

		src/approx_relpose_generalized_fast_computeA.h
		src/approx_relpose_axial_fast_computeA.h
		src/approx_relpose_generalized_fast_computeA.cpp
		src/approx_relpose_generalized_fast.cpp
		src/approx_relpose_axial.cpp
		src/approx_relpose_axial_fast_computeA.cpp
		src/approx_relpose_axial_fast.cpp
		src/test_approx_pose.cpp)



