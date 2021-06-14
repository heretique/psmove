# To show the OpenCV camera capture window everytime (useful for debugging)
option(PSMOVE_USE_DEBUG_CAPTURE "Always show camera capture input" OFF)
option(PSMOVE_USE_CVUI "Use CVUI for settings tweaking" OFF)

find_package(OpenCV CONFIG REQUIRED)
#find_package(rttr CONFIG REQUIRED)

add_library(psmove_tracker STATIC
    platform/camera_control_win32.cpp
    tracker_helpers.h
    tracker_helpers.cpp
    psmove_tracker.cpp
    psmove_fusion.cpp
    camera_control_private.h
    camera_control.h
    camera_control.cpp
    ../include/psmove/psmove_fusion.h
    ../include/psmove/psmove_tracker.h
)

target_include_directories(psmove_tracker
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>    
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/psmove
    )

target_link_libraries(psmove_tracker PRIVATE psmove opencv_world vfw32 comctl32 RTTR::Core_Lib)

if(PSMOVE_USE_DEBUG_CAPTURE)
    target_compile_definitions(psmove_tracker PRIVATE CAMERA_CONTROL_DEBUG_CAPTURED_IMAGE)
endif()

if(PSMOVE_USE_CVUI)
    target_compile_definitions(psmove_tracker PRIVATE USE_CVUI)
endif()

target_compile_definitions(psmove_tracker PRIVATE BUILDING_SHARED_LIBRARY _USE_MATH_DEFINES WIN32_LEAN_AND_MEAN NOMINMAX)
