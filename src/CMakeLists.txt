aux_source_directory( . AUX_SRC_VIO_INITIALIZOR )

# Add all relative components of slam utility.
set( SRC_PATH ${PROJECT_SOURCE_DIR}/../VIO_Initializor/src )
if( NOT TARGET lib_vio_initializor_data_loader )
    add_subdirectory( ${SRC_PATH}/data_loader ${PROJECT_SOURCE_DIR}/build/lib_vio_initializor_data_loader )
endif()
if( NOT TARGET lib_vio_initializor_data_manager )
    add_subdirectory( ${SRC_PATH}/data_manager ${PROJECT_SOURCE_DIR}/build/lib_vio_initializor_data_manager )
endif()
if( NOT TARGET lib_vio_initializor_frontend )
    add_subdirectory( ${SRC_PATH}/frontend ${PROJECT_SOURCE_DIR}/build/lib_vio_initializor_frontend )
endif()

# Select target initializor of backend.
set( TARGET_INITIALIZOR
    backend_vins_mono
    # backend_drt_vio_init
)
if( NOT TARGET lib_vio_initializor_backend )
    add_subdirectory( ${SRC_PATH}/${TARGET_INITIALIZOR} ${PROJECT_SOURCE_DIR}/build/lib_vio_initializor_backend )
endif()

add_library( lib_vio_initializor ${AUX_SRC_VIO_INITIALIZOR} )
target_include_directories( lib_vio_initializor PUBLIC . )
target_link_libraries( lib_vio_initializor
    lib_vio_initializor_data_loader
    lib_vio_initializor_data_manager
    lib_vio_initializor_frontend
    lib_vio_initializor_backend
)
