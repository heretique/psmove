cmake_minimum_required(VERSION 3.16) # for precompiled headers and unity builds

if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  file(TO_CMAKE_PATH "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CMAKE_TOOLCHAIN_FILE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(psmove VERSION 0.1.0 LANGUAGES CXX C)
include(CMakeToolsHelpers OPTIONAL)
include(CMakePrintHelpers)

set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

option(PSMOVE_SHARED_LIBS "Implement library as shared libs" OFF)
option(PSMOVE_USE_PCH "Use precompiled headers" OFF)
option(PSMOVE_USE_UNITY_BUILDS "Use unity builds" OFF)

#if (USE_PCH)
#  if (BUILD_STATIC OR BUILD_WITH_STATIC_RUNTIME_LIBS)
#    set(USE_PCH FALSE)
#  endif()
#endif()

#set(LIB_TYPE STATIC)

#if (ATLAS_SHARED_LIBS)
#    set(LIB_TYPE SHARED)
#    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
#endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/psmove/psmove_config.h.in
    ${CMAKE_CURRENT_SOURCE_DIR}/include/psmove/psmove_config.h
    @ONLY)
add_library(psmove STATIC 

    psmoveapi.cpp
    psmove_sockets.h
    psmove_private.h
    psmove_port.h
    psmove_orientation.h
    psmove_orientation.cpp
    psmove_calibration.h
    psmove_calibration.c
    psmove.c
    daemon/psmove_moved_protocol.h
    daemon/moved_monitor.h
    daemon/moved_client.h
    daemon/moved_client.cpp
    include/psmove/psmove.h
    include/psmove/psmoveapi.h
    include/psmove/psmove_tracker.h
    include/psmove/psmove_fusion.h
    math/psmove_vector.h
    math/psmove_vector.c
    math/psmove_quaternion.hpp
    math/psmove_quaternion.cpp
    math/psmove_math.h
    math/psmove_math.c
    math/psmove_glm_math.hpp
    math/psmove_alignment.hpp
    math/psmove_alignment.cpp
    platform/psmove_port_windows.c
    )


target_include_directories(psmove 
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/math
        ${CMAKE_CURRENT_SOURCE_DIR}/include/psmove
)

target_compile_definitions(psmove 
    PUBLIC
        PSMOVE_DEBUG
    PRIVATE
        BUILDING_SHARED_LIBRARY
        _USE_MATH_DEFINES
        WIN32_LEAN_AND_MEAN
        NOMINMAX
)


find_package(glm CONFIG REQUIRED)
find_package(hidapi CONFIG REQUIRED)

target_include_directories(psmove PRIVATE ${HIDAPI_INCLUDE_DIRS})

target_link_libraries(psmove PRIVATE glm::glm ${HIDAPI_LIBRARIES} setupapi bthprops kernel32 ws2_32 winmm version imm32)

add_subdirectory(tracker)