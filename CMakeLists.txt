cmake_minimum_required(VERSION 3.22.1)
project(pathfinder)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_BUILD_TYPE "Release") # force release build - COMMENT FOR DEBUGGING

add_compile_options("-Wall" "-Wextra" "-Wpedantic") # enable more warnings

include_directories(${CMAKE_SOURCE_DIR}/libs/rapid/include 
                    /usr/include/cairo)
link_directories(${CMAKE_SOURCE_DIR}/libs/rapid/)

file(GLOB sources src/*.cpp)
add_executable(${PROJECT_NAME} ${sources})

target_link_libraries(${PROJECT_NAME} cairo RAPID)

