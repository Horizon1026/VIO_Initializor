cmake_minimum_required( VERSION 3.10 )

project( vio_stereo_schur_vins )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall -pthread" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_STANDARD 17 )

# Add integrate navigation system.
if ( NOT TARGET lib_schur_vins )
    add_subdirectory( src ${PROJECT_SOURCE_DIR}/build/lib_schur_vins )
endif()

# Create executable target to test schur vins.
add_executable( test_locator
    test/test_locator.cpp
)
target_link_libraries( test_locator
    lib_schur_vins
)
