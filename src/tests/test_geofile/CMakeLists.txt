aux_source_directories(SOURCES "" .)
vor_add_executable(test_geofile ${SOURCES})
target_link_libraries(test_geofile geogram)

set_target_properties(test_geofile PROPERTIES FOLDER "GEOGRAM/Tests")
