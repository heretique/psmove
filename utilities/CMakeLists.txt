find_package(OpenCV CONFIG REQUIRED)
find_package(rttr CONFIG REQUIRED)

add_executable(PSMoveConfigurator 
    imgui/imgui_impl_raylib.cpp
    imgui/imgui_impl_raylib.h
    imgui/RttrImguiInspector.cpp
    imgui/RttrImguiInspector.h
    PSMoveConfiguratorMain.cpp 
    PSMoveConfigurator.cpp
    PSMoveConfigurator.h)

target_include_directories(PSMoveConfigurator PRIVATE imgui)
target_link_libraries(PSMoveConfigurator RTTR::Core raylib opencv_world imgui psmove psmove_tracker)