# to add new testscases, simply add a file named 'test_something.cpp' in this
# tests/ folder (where something should be replaced with some sensible name ;))

# tests can be run with 'make all test' (or just 'make test' to run the tests without compiling)
# individual tests can be run by simply calling the binaries in build/tests/
# logfiles with detailed output are in build/Testing/Temporary/

# based on: http://neyasystems.com/an-engineers-guide-to-unit-testing-cmake-and-boost-unit-tests/

#Setup CMake to run tests
# must be done in root CMakeLists.txt

#Prep ourselves for compiling boost
find_package(Boost COMPONENTS unit_test_framework REQUIRED)

if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

if(SFML_FOUND)
  include_directories(${SFML_INCLUDE_DIR})
endif()

if(EIGEN3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIR})
endif()

#I like to keep test files in a separate source directory called test
file(GLOB TEST_SRCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" test_*.cpp)

#Run through each source
foreach(testSrc ${TEST_SRCS})
        #Extract the filename without an extension (NAME_WE)
        get_filename_component(testName ${testSrc} NAME_WE)

        message(STATUS "found test case '${testName}'")

        #Add compile target
        add_executable(${testName} "${CMAKE_CURRENT_SOURCE_DIR}/${testSrc}")

        #link to Boost libraries AND your targets and dependencies (normal codebase is in a separate library)
        target_link_libraries(${testName} ${LIBRARY_NAME} ${Boost_LIBRARIES} ${SFML_LIBRARIES})

        #Finally add it to test execution
        add_test(NAME ${testName}
            COMMAND "${CMAKE_CURRENT_BINARY_DIR}/${testName}"
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
endforeach(testSrc)
