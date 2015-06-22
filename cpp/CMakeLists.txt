# requires:
# SFML 2   (for displaying pretty things)
# Eigen 3  (for fast numerics)
# Boost    (for various helper stuff like string conversions)


# source:
# https://github.com/SFML/SFML/wiki/Tutorial:-Build-your-SFML-project-with-CMake

cmake_minimum_required (VERSION 2.8)
project (dense_odometry)

#set(EXTRA_FLAGS "-pg -fprofile-arcs -ftest-coverage")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra ${EXTRA_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_FLAGS}")

set(SOURCES
    "core/image_data.cpp"
    "core/camera_intrinsics.cpp"
    "core/transformation.cpp"
    "core/camera_step.cpp"
    "core/camera_image.cpp"
    "core/warp.cpp"
    "core/warp_streamlined.cpp"
    "core/minimization.cpp"
    "core/weight_functions.cpp"
    "gui/scene.cpp"
    "gui/colormap.cpp"
    "gui/live.cpp"
    "gui/minimization_gui.cpp"
    "gui/sf_image_data.cpp"
    "gui/state_tracker.cpp"
    "utils/progress.cpp"
    "utils/matrix.cpp"
)



# compile all sources to library, so we can use it for unit tests as well
# without having to recompile everything
set(LIBRARY_NAME "dense_odometry_lib")
add_library(${LIBRARY_NAME} STATIC ${SOURCES})

# define final executable
set(EXECUTABLE_NAME "dense_odometry")
add_executable(${EXECUTABLE_NAME} "main.cpp")
target_link_libraries(${EXECUTABLE_NAME} ${LIBRARY_NAME})


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})



# this is a catkin module
#find_package(catkin REQUIRED)
#catkin_package()

find_package(roscpp REQUIRED)
find_package(rosbag REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
target_link_libraries(${EXECUTABLE_NAME} ${roscpp_LIBRARIES})

include_directories(${rosbag_INCLUDE_DIRS})
target_link_libraries(${EXECUTABLE_NAME} ${rosbag_LIBRARIES})

include_directories(${sensor_msgs_INCLUDE_DIRS})
target_link_libraries(${EXECUTABLE_NAME} ${sensor_msgs_LIBRARIES})

include_directories(${stereo_msgs_INCLUDE_DIRS})
target_link_libraries(${EXECUTABLE_NAME} ${stereo_msgs_LIBRARIES})

# Detect and add SFML
#Find any version 2.X of SFML
#See the FindSFML.cmake file for additional details and instructions
find_package(SFML 2 REQUIRED system window graphics network audio)

if(SFML_FOUND)
  include_directories(${SFML_INCLUDE_DIR})
  target_link_libraries(${EXECUTABLE_NAME} ${SFML_LIBRARIES})
endif()


# Detect and add OpenGL (for 3D visualization
find_package(OpenGL REQUIRED)

if(OPENGL_FOUND)
    include_directories(${OPENGL_INCLUDE_DIRS})
    target_link_libraries(${EXECUTABLE_NAME} ${OPENGL_LIBRARIES})
endif()

# somehow, debug build requires GLEW?!
find_package(GLEW REQUIRED)

if(GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIRS})
    target_link_libraries(${EXECUTABLE_NAME} ${GLEW_LIBRARIES})
endif()



# Detect and add Eigen 3
find_package(Eigen3 REQUIRED)

if(EIGEN3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIR})
    message(STATUS "including Eigen3 from '${EIGEN3_INCLUDE_DIR}'")
endif()


# Detect and add Boost
# http://www.cmake.org/cmake/help/v3.0/module/FindBoost.html
find_package(Boost REQUIRED COMPONENTS timer) # TODO: add specific libraries

if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    include_directories(${Boost_Timer_INCLUDE_DIRS})
    target_link_libraries(${EXECUTABLE_NAME} ${Boost_TIMER_LIBRARIES})
    message(STATUS "boost libs for timer: ${Boost_TIMER_LIBRARIES}")

endif()


# unit tests
enable_testing()
add_subdirectory(tests)
