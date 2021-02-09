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
option(PSMOVE_BUILD_UTILITIES "Enable this to build utilities" ON)
option(PSMOVE_BUILD_EXAMPLES "Enable this to build examples" ON)

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
add_subdirectory(3rdparty/imguiProj)
add_subdirectory(3rdparty/raylib)
add_subdirectory(psmove)

if (PSMOVE_BUILD_UTILITIES)
    add_subdirectory(utilities)
endif()

if (PSMOVE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()