cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

file( GLOB common_srcs src/common/*.cpp )
rosbuild_add_library( ${PROJECT_NAME} ${common_srcs} )

file( GLOB test_srcs src/tests/*.cpp )
foreach( test_src ${test_srcs} )
    get_filename_component( test_name ${test_src} NAME_WE )
    set( test_file ${PROJECT_SOURCE_DIR}/src/tests/${test_name}.test )
    if ( EXISTS ${test_file} )
        rosbuild_add_executable( ${test_name} ${test_src} )
        rosbuild_add_gtest_build_flags( ${test_name} )
        target_link_libraries( ${test_name} ${PROJECT_NAME} )
        rosbuild_add_rostest( ${test_file} )
    else()
        rosbuild_add_gtest( ${test_name} ${test_src} )
        target_link_libraries( ${test_name} ${PROJECT_NAME} )
    endif()
endforeach()
