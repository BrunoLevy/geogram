aux_source_directories(SOURCES "" .)

vor_reset_warning_level()

if(GEOGRAM_USE_BUILTIN_DEPS)
aux_source_directories(SOURCES "" ./third_party/ANN)
endif()

vor_add_executable(test_nn_search ${SOURCES})

if(GEOGRAM_USE_BUILTIN_DEPS)
target_link_libraries(test_nn_search geogram)
else()
target_link_libraries(test_nn_search ann)
endif()

set_target_properties(test_nn_search PROPERTIES FOLDER "GEOGRAM/Tests")
