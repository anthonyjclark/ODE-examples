cmake_minimum_required(VERSION 3.12)
project(ugv)

set(CMAKE_CXX_STANDARD 17)

add_executable(ugv main.cpp)

# Add ODE library
include_directories("/Users/ajc/Documents/projects/simulators/local/lib/")
target_link_libraries(ugv -lode)