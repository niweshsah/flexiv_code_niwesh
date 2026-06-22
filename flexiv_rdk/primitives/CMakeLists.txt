cmake_minimum_required(VERSION 3.16.3)
project(flexiv_primitives)

# Set C++17 Standard (Required by Flexiv v1.9.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the Flexiv RDK package
find_package(flexiv_rdk REQUIRED)

# Add your MoveL executable
add_executable(MoveL MoveL.cpp)

add_executable(MovePTP MovePTP.cpp)


# Link against the official exported target!
target_link_libraries(MoveL flexiv::flexiv_rdk)
target_link_libraries(MovePTP flexiv::flexiv_rdk)
