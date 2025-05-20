aux_source_directory( . AUX_SRC_VIO_INITIALIZOR_FRONTEND )

# Add stereo visual frontend.
set( VISUAL_FRONTEND_PATH ${PROJECT_SOURCE_DIR}/../Visual_Frontend/src/frontend_stereo )
if( NOT TARGET lib_visual_frontend_stereo )
    add_subdirectory( ${VISUAL_FRONTEND_PATH} ${PROJECT_SOURCE_DIR}/build/lib_visual_frontend_stereo )
endif()

# Create library.
add_library( lib_vio_initializor_frontend ${AUX_SRC_VIO_INITIALIZOR_FRONTEND} )
target_include_directories( lib_vio_initializor_frontend PUBLIC . )
target_link_libraries( lib_vio_initializor_frontend
    lib_visual_frontend_stereo
)
