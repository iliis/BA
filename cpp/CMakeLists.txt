# requires:
# SFML 2   (for displaying pretty things)
# Eigen 3  (for fast numerics)
# Boost    (for various helper stuff like string conversions)


# source:
# https://github.com/SFML/SFML/wiki/Tutorial:-Build-your-SFML-project-with-CMake

cmake_minimum_required (VERSION 2.8)
project (dense_odometry)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra")

set(SOURCES
    "core/image_data.cpp"
    "core/camera_intrinsics.cpp"
    "core/transformation.cpp"
    "core/scene.cpp"
    "core/camera_step.cpp"
    "core/camera_image.cpp"
    "core/warp.cpp"
    "core/minimization.cpp"
    "utils/colormap.cpp"
    "utils/progress.cpp"
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

# Detect and add SFML
#Find any version 2.X of SFML
#See the FindSFML.cmake file for additional details and instructions
find_package(SFML 2 REQUIRED system window graphics network audio)

if(SFML_FOUND)
  include_directories(${SFML_INCLUDE_DIR})
  target_link_libraries(${EXECUTABLE_NAME} ${SFML_LIBRARIES})
endif()



# Detect and add Eigen 3
find_package(Eigen3 REQUIRED)

if(EIGEN3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIR})
endif()


# Detect and add Boost
# http://www.cmake.org/cmake/help/v3.0/module/FindBoost.html
find_package(Boost REQUIRED) # TODO: add specific libraries

if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()


enable_testing()
add_subdirectory(tests)
