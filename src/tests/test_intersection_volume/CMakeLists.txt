aux_source_directories(SOURCES "" .)
vor_add_executable(test_intersection_volume ${SOURCES})
target_link_libraries(test_intersection_volume geogram)
set_target_properties(test_intersection_volume PROPERTIES FOLDER "GEOGRAM/Tests")
