set(APP_NAME vorpalite_vtu)
aux_source_directories(SOURCES "" .)
vor_add_executable(${APP_NAME} ${SOURCES})
target_link_libraries(${APP_NAME} geogram)

install_runtime_targets(${APP_NAME})

set_target_properties(${APP_NAME} PROPERTIES FOLDER "GEOGRAM/Programs")

find_package(VTK REQUIRED)
target_link_libraries(${APP_NAME} VTK::CommonDataModel VTK::IOCore VTK::IOXML VTK::IOInfovis VTK::FiltersCore VTK::FiltersGeneral)
