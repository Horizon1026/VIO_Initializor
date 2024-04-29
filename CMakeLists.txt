cmake_minimum_required( VERSION 3.10 )

project( vio_stereo_schur_vins )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall -pthread" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_STANDARD 17 )

# Add integrate navigation system.
if ( NOT TARGET lib_ins_fusion )
    add_subdirectory( src ${PROJECT_SOURCE_DIR}/build/lib_ins_fusion )
endif()

# Create executable target to test.
add_executable( test_pure_imu
    test/test_pure_imu.cpp
)
target_link_libraries( test_pure_imu
    lib_ins_fusion
)
