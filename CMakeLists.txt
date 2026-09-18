cmake_minimum_required(VERSION 3.20)

project(TrafficSimulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PYBIND11_FINDPYTHON ON)
find_package(Python 3.11 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

add_library(traffic_core STATIC
    src/Graph.cpp
    src/Simulation.cpp
    src/trafficLightSystem.cpp
)

target_include_directories(traffic_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_definitions(traffic_core PUBLIC
    PROJECT_PATH="${CMAKE_SOURCE_DIR}/"
)

add_executable(simulator src/main.cpp)
target_link_libraries(simulator PRIVATE traffic_core)

pybind11_add_module(traffic_rl_native src/bindings.cpp)
target_link_libraries(traffic_rl_native PRIVATE traffic_core)

add_executable(generator
    src/cityGenerator.cpp

)

target_include_directories(generator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_definitions(generator PRIVATE
    PROJECT_PATH="${CMAKE_SOURCE_DIR}"
)

if(MSVC)
    target_compile_options(traffic_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(traffic_core PRIVATE
        -Wall -Wextra -Wpedantic -Wshadow
    )
endif()

if(MSVC)
    target_compile_options(generator PRIVATE /W4 /permissive-)
else()
    target_compile_options(generator PRIVATE
        -Wall -Wextra -Wpedantic -Wshadow
    )
endif()