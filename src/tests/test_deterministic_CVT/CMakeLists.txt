aux_source_directories(SOURCES "" .)
vor_add_executable(test_deterministic_CVT ${SOURCES})
target_link_libraries(test_deterministic_CVT geogram)

set_target_properties(test_deterministic_CVT PROPERTIES FOLDER "GEOGRAM/Tests")
