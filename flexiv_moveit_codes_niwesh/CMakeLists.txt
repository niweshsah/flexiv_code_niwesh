cmake_minimum_required(VERSION 3.8)
project(flexiv_moveit_codes_niwesh)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==========================================
# FORCE RDK 1.9.1 BOOST OVERRIDES
# ==========================================
# This stops MoveIt / CMake from pulling in the system's default Boost 1.74
set(Boost_DIR "/home/tcs/flexiv_install/lib/cmake/Boost-1.83.0")
set(boost_date_time_DIR "/home/tcs/flexiv_install/lib/cmake/boost_date_time-1.83.0")
set(BOOST_ROOT "/home/tcs/flexiv_install")
set(Boost_NO_SYSTEM_PATHS TRUE)

# ==========================================
# 1. FIND REQUIRED DEPENDENCIES
# ==========================================
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(flexiv_rdk REQUIRED)

# Dependencies for Pick and Place & Transforms
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Dependencies for PBVS Controller
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui aruco calib3d)

# ==========================================
# 2. GLOBAL INCLUDE DIRECTORIES
# ==========================================
include_directories(
  include
  src/pbvs/include
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ==========================================
# 3. EXECUTABLES & LINKING
# ==========================================



# Target 2: PBVS Controller
add_executable(pbvs_controller_node src/pbvs/pbvs_controller.cpp)
target_compile_features(pbvs_controller_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(pbvs_controller_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

add_executable(aruco_pose_estimator src/pbvs/aruco_pose_estimator.cpp)
target_compile_features(aruco_pose_estimator PUBLIC c_std_99 cxx_std_17)
target_include_directories(aruco_pose_estimator PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(aruco_pose_estimator
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  tf2
)
target_link_libraries(aruco_pose_estimator ${OpenCV_LIBRARIES})




# ==========================================
# 4. INSTALL RULES
# ==========================================

# Install all compiled binaries to the lib folder
install(TARGETS
  pbvs_controller_node
  aruco_pose_estimator
  DESTINATION lib/${PROJECT_NAME}
)

# Install PBVS YAML config files to the share folder
install(
  DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ==========================================
# 5. TESTING & LINTING
# ==========================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skips the linter checks for copyrights & cpplint
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()