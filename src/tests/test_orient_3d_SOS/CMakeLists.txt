aux_source_directories(SOURCES "" .)
vor_add_executable(test_orient_3d_SOS ${SOURCES})
target_link_libraries(test_orient_3d_SOS geogram)

set_target_properties(test_orient_3d_SOS PROPERTIES FOLDER "GEOGRAM/Tests")
