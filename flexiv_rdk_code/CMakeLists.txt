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



# Target H: Go home
add_executable(go_home src/go_home.cpp)
target_link_libraries(go_home flexiv::flexiv_rdk)

#
add_executable(ibvs_aruco_1_point src/ibvs_aruco_1_point.cpp)
target_link_libraries(ibvs_aruco_1_point
    flexiv::flexiv_rdk
    realsense2::realsense2
    ${OpenCV_LIBS}
    ${VISP_LIBRARIES}
)

add_executable(arcuo_pbvs src/arcuo_pbvs.cpp)

target_include_directories(arcuo_pbvs PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${VISP_INCLUDE_DIRS}
)

target_link_libraries(arcuo_pbvs PRIVATE
    flexiv::flexiv_rdk
    realsense2::realsense2
    ${OpenCV_LIBS}
    ${VISP_LIBRARIES}
)