cmake_minimum_required(VERSION 3.12)
project(ugv_visualized)

set(CMAKE_CXX_STANDARD 17)

add_executable(ugv_visualized main.cpp)

# Add ODE library
include_directories("/Users/ajc/Documents/projects/simulators/local/lib/")
target_link_libraries(ugv_visualized -lode)

# Add Review logger
include_directories("/Users/ajc/Documents/projects/simulators/ODE-examples/logger-cpp/src/")

# Add cxxopts
include_directories("/Users/ajc/Documents/projects/simulators/ODE-examples/cxxopts/include/")