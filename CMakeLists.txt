cmake_minimum_required(VERSION 3.20)

project(TrafficSimulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(simulator
    src/main.cpp
)

target_compile_definitions(simulator PRIVATE
    PROJECT_PATH="${CMAKE_SOURCE_DIR}"
)
