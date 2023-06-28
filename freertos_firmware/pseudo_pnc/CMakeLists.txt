cmake_minimum_required(VERSION 3.5)

project(pseudo_pnc)

find_package(ament_cmake REQUIRED)

include(ExternalProject)
include(CMakeParseArguments)

function(export_executable)
    set(subtree "")
    foreach(arg IN LISTS ARGN)
        set(subtree ${subtree}/${arg})
    endforeach()
    get_filename_component(leaf ${subtree} NAME)
    ExternalProject_Add(${leaf}
        PREFIX
            ${PROJECT_BINARY_DIR}${subtree}
        SOURCE_DIR
            ${PROJECT_SOURCE_DIR}${subtree}
        INSTALL_DIR
            ${PROJECT_BINARY_DIR}/temp_install
        BUILD_ALWAYS 1
        CMAKE_CACHE_ARGS
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
            -DCMAKE_PREFIX_PATH:PATH=${CMAKE_PREFIX_PATH}
            -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    )
endfunction()

# export_executable(control)
# export_executable(planning)
export_executable(pnc)
# export_executable(configuration_example custom_transports)

if(EXISTS ${CMAKE_BINARY_DIR}/temp_install/)
    install(
        DIRECTORY
            ${CMAKE_BINARY_DIR}/temp_install/
        DESTINATION
            lib/${PROJECT_NAME}
        USE_SOURCE_PERMISSIONS
        )
endif()

ament_package()
