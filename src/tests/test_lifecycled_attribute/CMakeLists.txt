aux_source_directories(SOURCES "" .)
vor_add_executable(test_lifecycled_attribute ${SOURCES})
target_link_libraries(test_lifecycled_attribute geogram)

set_target_properties(test_lifecycled_attribute PROPERTIES FOLDER "GEOGRAM/Tests")
