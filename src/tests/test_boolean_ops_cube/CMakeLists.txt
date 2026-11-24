aux_source_directories(SOURCES "" .)
vor_add_executable(test_boolean_ops_cube ${SOURCES})
target_link_libraries(test_boolean_ops_cube geogram)

set_target_properties(test_boolean_ops_cube PROPERTIES FOLDER "GEOGRAM/Tests")
