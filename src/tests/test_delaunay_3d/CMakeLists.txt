aux_source_directories(SOURCES "" .)
vor_add_executable(test_delaunay_3d ${SOURCES})
target_link_libraries(test_delaunay_3d geogram)

set_target_properties(test_delaunay_3d PROPERTIES FOLDER "GEOGRAM/Tests")
