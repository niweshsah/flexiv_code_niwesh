cmake_minimum_required(VERSION 3.16)
project(GenericPrimitiveExecutor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Dependencies
find_package(flexiv_rdk REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

# Source Files
set(SOURCES
    src/main.cpp
    src/Common/Logger.cpp
    src/Common/SignalHandler.cpp
    src/Parser/ParserFactory.cpp
    src/Parser/MoveLParser.cpp
    src/Parser/MovePTPParser.cpp
    src/Parser/ForceCompParser.cpp
    src/Parser/MoveJParser.cpp
    src/CLI/CLIParser.cpp
    src/Robot/RobotExecutor.cpp
    src/Parser/AlignContactParser.cpp 
    src/Parser/ForceHybridParser.cpp 
    src/Parser/ForceTrajParser.cpp
    src/Parser/VisualServoingParser.cpp
)

# Executable
add_executable(PrimitiveExecutor ${SOURCES})

# Explicitly pass the include directories to our target
target_include_directories(PrimitiveExecutor PRIVATE 
    include
    ${flexiv_rdk_INCLUDE_DIRS}
)

# Link Libraries
target_link_libraries(PrimitiveExecutor PRIVATE 
    flexiv::flexiv_rdk 
    spdlog::spdlog
    yaml-cpp
)

# Compile Options for Code Quality
target_compile_options(PrimitiveExecutor PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -O3
)