aux_source_directories(SOURCES "" .)

vor_reset_warning_level()

#if(
#    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR
#    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang"
#)
#  add_definitions(-Wno-zero-as-null-pointer-constant)
#  add_definitions(-Wno-suggest-override)
#  add_definitions(-Wno-suggest-destructor-override)
#  add_definitions(-Wno-weak-vtables)
#endif()

aux_source_directories(SOURCES "" ./third_party/ANN)

vor_add_executable(test_nn_search ${SOURCES})
target_link_libraries(test_nn_search geogram)

set_target_properties(test_nn_search PROPERTIES FOLDER "GEOGRAM/Tests")

