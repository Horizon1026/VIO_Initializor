aux_source_directory( . AUX_SRC_LOCATOR_IMU_MANAGER )

# Add manager of sensor.
set( INS_FUSION_PATH ${PROJECT_SOURCE_DIR}/../INS_Fusion/src )
if ( NOT TARGET lib_locator_sensor_manager )
    add_subdirectory( ${INS_FUSION_PATH}/sensor_manager ${PROJECT_SOURCE_DIR}/build/lib_locator_sensor_manager )
endif()

# Add sensor models from 'Sensor_Model'.
set( SENSOR_MODEL_PATH ${PROJECT_SOURCE_DIR}/../Sensor_Model/src )
if ( NOT TARGET lib_imu_model )
	add_subdirectory( ${SENSOR_MODEL_PATH}/imu ${PROJECT_SOURCE_DIR}/build/lib_imu_model )
endif()

# Add square root kalman filter.
set( SLAM_SOLVER_PATH ${PROJECT_SOURCE_DIR}/../Slam_Solver/src )
if ( NOT TARGET lib_square_root_kalman_filter )
    add_subdirectory( ${SLAM_SOLVER_PATH}/kalman_filter/square_root_kalman_filter ${PROJECT_SOURCE_DIR}/build/lib_square_root_kalman_filter )
endif()

# Create library.
add_library( lib_locator_imu_manager ${AUX_SRC_LOCATOR_IMU_MANAGER} )
target_include_directories( lib_locator_imu_manager PUBLIC . )
target_link_libraries( lib_locator_imu_manager
    lib_locator_sensor_manager
    lib_imu_model
    lib_square_root_kalman_filter
)
