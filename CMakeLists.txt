cmake_minimum_required(VERSION 3.20)

project(TrafficSimulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(simulator
    src/main.cpp
    src/Graph.cpp
    src/trafficLightSystem.cpp
    src/Simulation.cpp
)

add_executable(generator
    src/cityGenerator.cpp

)
target_include_directories(simulator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(generator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_definitions(generator PRIVATE
    PROJECT_PATH="${CMAKE_SOURCE_DIR}"
)

target_compile_definitions(simulator PRIVATE
    PROJECT_PATH="${CMAKE_SOURCE_DIR}"
)

if(MSVC)
    target_compile_options(simulator PRIVATE /W4 /permissive-)
else()
    target_compile_options(simulator PRIVATE
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