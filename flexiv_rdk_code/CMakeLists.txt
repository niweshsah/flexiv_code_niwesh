cmake_minimum_required(VERSION 3.10)
project(flexiv_visual_servo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- 1. Find Dependencies ---
find_package(realsense2 REQUIRED)
find_package(flexiv_rdk REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)

# --- 2. Include Directories ---
include_directories(include)

# --- 3. Build Executables ---

# Target A: The core Visual Servoing Control Application
add_executable(visp src/visp.cpp)
target_link_libraries(visp 
    flexiv::flexiv_rdk 
    ${VISP_LIBRARIES}
    realsense2::realsense2
    ${OpenCV_LIBS}
)

# Target B: The Yellow Object Tracker (uses the class implementation)
# Here realsense_camera is compiled with track_yellow.cpp
add_executable(track_yellow src/track_yellow.cpp src/realsense_camera.cpp)
target_link_libraries(track_yellow 
    realsense2::realsense2
    ${OpenCV_LIBS}
)

# Target C: Standalone Camera Diagnostics / Verification
# Compiles the class independently using your conditional #ifdef BUILD_STANDALONE main loop
# here realsense_camera is compiled independently
add_executable(realsense_camera_test src/realsense_camera.cpp)
target_compile_definitions(realsense_camera_test PRIVATE BUILD_STANDALONE)
target_link_libraries(realsense_camera_test 
    realsense2::realsense2
    ${OpenCV_LIBS}
)

# Target E: Standalone tool to verify 3D coordinates in the Camera Frame
add_executable(find_yellow_camera src/find_yellow_camera.cpp)
target_link_libraries(find_yellow_camera 
    realsense2::realsense2
    ${OpenCV_LIBS}
)

# Target F: Full kinematics tool to calculate coordinates in Robot Base Frame
add_executable(find_yellow_base src/find_yellow_base.cpp)
target_link_libraries(find_yellow_base 
    flexiv::flexiv_rdk 
    realsense2::realsense2
    ${OpenCV_LIBS}
)

# Target G: Full Integrated ViSP Image-Based Visual Servoing
add_executable(ibvs_basic src/ibvs_basic.cpp)
target_link_libraries(ibvs_basic 
    flexiv::flexiv_rdk 
    realsense2::realsense2
    ${OpenCV_LIBS}
    ${VISP_LIBRARIES}
)

# Target H: Go home
add_executable(go_home src/go_home.cpp)
target_link_libraries(go_home flexiv::flexiv_rdk)

# Target I
add_executable(ibvs_single_point_translation src/ibvs_single_point_translation.cpp)
target_link_libraries(ibvs_single_point_translation 
    flexiv::flexiv_rdk 
    realsense2::realsense2
    ${OpenCV_LIBS}
    ${VISP_LIBRARIES}
)


# Target J
add_executable(ibvs_4_point src/ibvs_4_point.cpp)
target_link_libraries(ibvs_4_point 
    flexiv::flexiv_rdk 
    realsense2::realsense2
    ${OpenCV_LIBS}
    ${VISP_LIBRARIES}
)


add_executable(ibvs_aruco_4_point src/ibvs_aruco_4_point.cpp)
target_link_libraries(ibvs_aruco_4_point 
    flexiv::flexiv_rdk 
    realsense2::realsense2
    ${OpenCV_LIBS}
    ${VISP_LIBRARIES}
)


add_executable(ibvs_aruco_1_point src/ibvs_aruco_1_point.cpp)
target_link_libraries(ibvs_aruco_1_point 
    flexiv::flexiv_rdk 
    realsense2::realsense2
    ${OpenCV_LIBS}
    ${VISP_LIBRARIES}
)



add_executable(teach_ibvs src/teach_ibvs.cpp)
target_link_libraries(teach_ibvs 
    flexiv::flexiv_rdk 
    realsense2::realsense2
    ${OpenCV_LIBS}
    ${VISP_LIBRARIES}
)

add_executable(simple_aruco_code
    src/simple_aruco_code.cpp
)
target_link_libraries(simple_aruco_code
    PRIVATE
    realsense2::realsense2
    ${OpenCV_LIBS}
)