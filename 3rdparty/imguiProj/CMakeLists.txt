cmake_minimum_required(VERSION 3.16) # for precompiled headers and unity builds

add_library(imgui STATIC "")
add_library(imgui::imgui ALIAS imgui)

target_sources(imgui PRIVATE
    ../imgui/imgui_draw.cpp
    ../imgui/imgui.cpp
    ../imgui/imgui_demo.cpp
    ../imgui/imgui_tables.cpp
    ../imgui/imgui_widgets.cpp
    ../imgui/backends/imgui_impl_opengl3.cpp
    ../imgui/backends/imgui_impl_opengl3.h)

target_compile_definitions(imgui PRIVATE IMGUI_IMPL_OPENGL_LOADER_GLAD)

target_include_directories(imgui PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../imgui>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../imgui/backends>
    $<INSTALL_INTERFACE:include>)

find_package(glad)

target_link_libraries(imgui PRIVATE glad::glad)