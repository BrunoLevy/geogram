aux_source_directories(SOURCES "" .)
vor_add_executable(test_vertex_contact ${SOURCES})
target_link_libraries(test_vertex_contact geogram)
set_target_properties(test_vertex_contact PROPERTIES FOLDER "GEOGRAM/Tests")
