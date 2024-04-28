aux_source_directory( . AUX_SRC_VIO_STEREO_SCHUR_VINS )

# Add all components of slam utility.
set( SRC_PATH ${PROJECT_SOURCE_DIR}/../VIO_Stereo_VINS/src )
if ( NOT TARGET lib_vio_stereo_vins_data_loader )
    add_subdirectory( ${SRC_PATH}/data_loader ${PROJECT_SOURCE_DIR}/build/lib_vio_stereo_vins_data_loader )
endif()
if ( NOT TARGET lib_vio_stereo_vins_frontend )
    add_subdirectory( ${SRC_PATH}/frontend ${PROJECT_SOURCE_DIR}/build/lib_vio_stereo_vins_frontend )
endif()

add_library( lib_vio_stereo_schur_vins ${AUX_SRC_VIO_STEREO_SCHUR_VINS} )
target_include_directories( lib_vio_stereo_schur_vins PUBLIC . )
target_link_libraries( lib_vio_stereo_schur_vins
    lib_vio_stereo_vins_data_loader
    lib_vio_stereo_vins_frontend
)
