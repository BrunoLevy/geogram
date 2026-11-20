aux_source_directories(SOURCES "" .)
vor_add_executable(test_expansion_nt_2 ${SOURCES})
target_link_libraries(test_expansion_nt_2 geogram)

set_target_properties(test_expansion_nt_2 PROPERTIES FOLDER "GEOGRAM/Tests")
