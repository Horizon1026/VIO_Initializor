aux_source_directory( . AUX_SRC_VIO_INITIALIZOR )

# Add all relative components of slam utility.
set( VIO_INITIALIZOR_PATH ${PROJECT_SOURCE_DIR}/../VIO_Initializor )
if( NOT TARGET lib_vio_initializor_data_manager )
    add_subdirectory( ${VIO_INITIALIZOR_PATH}/src/data_manager ${PROJECT_SOURCE_DIR}/build/lib_vio_initializor_data_manager )
endif()
if( NOT TARGET lib_vio_initializor_frontend )
    add_subdirectory( ${VIO_INITIALIZOR_PATH}/src/frontend ${PROJECT_SOURCE_DIR}/build/lib_vio_initializor_frontend )
endif()

# Add general data loader.
set( SLAM_DATALOADER_PATH ${PROJECT_SOURCE_DIR}/../Slam_Dataloader )
if( NOT TARGET lib_general_data_loader )
    add_subdirectory( ${SLAM_DATALOADER_PATH}/src/data_loader ${PROJECT_SOURCE_DIR}/build/lib_general_data_loader )
endif()

# Select target initializor of backend.
set( TARGET_INITIALIZOR
    backend_vins_mono
    # backend_drt_vio_init
)
if( NOT TARGET lib_vio_initializor_backend )
    add_subdirectory( ${VIO_INITIALIZOR_PATH}/src/${TARGET_INITIALIZOR} ${PROJECT_SOURCE_DIR}/build/lib_vio_initializor_backend )
endif()

add_library( lib_vio_initializor ${AUX_SRC_VIO_INITIALIZOR} )
target_include_directories( lib_vio_initializor PUBLIC . )
target_link_libraries( lib_vio_initializor
    lib_vio_initializor_data_manager
    lib_vio_initializor_frontend
    lib_vio_initializor_backend

    lib_general_data_loader
)
