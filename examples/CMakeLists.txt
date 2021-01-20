

find_package(OpenCV CONFIG REQUIRED)

add_executable(OpenCVCameraCapture OpenCVCameraCapture.cpp )
target_link_libraries(OpenCVCameraCapture opencv_world)

add_executable(test_tracker test_tracker.cpp)
target_link_libraries(test_tracker psmove psmove_tracker opencv_world)