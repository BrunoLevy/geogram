set(APP_NAME geomol)

aux_source_directories(SOURCES "Source Files" .)
aux_source_directories(SOURCES "Source Files\\third_party" third_party)

vor_add_executable(${APP_NAME} ${SOURCES} ${THIRD_PARTY_SOURCES})
target_link_libraries(${APP_NAME} geogram_gfx geogram ${GLFW_LIBRARIES})

set_target_properties(${APP_NAME} PROPERTIES FOLDER "GEOGRAM/Programs")
