

build
=====

Requires

* Boost (just a few helpful headers)
* SFML 2 (for visualizing everything)
* Eigen 3 (for all the numeric stuff)

To compile the code, cmake is used:

    mkdir build
    cd build
    cmake -D CMAKE_BUILD_TYPE=Release ..
    make
    ./dense_odometry

Specifying CMAKE_BUILD_TYPE=Release is optional, but greatly speeds up the code.
