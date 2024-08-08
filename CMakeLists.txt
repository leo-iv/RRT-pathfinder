cmake_minimum_required(VERSION 3.22.1)
project(pathfinder)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_BUILD_TYPE "Debug") # force release build - COMMENT FOR DEBUGGING

add_compile_options("-Wall" "-Wextra" "-Wpedantic") # enable more warnings

find_package(FLANN REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/libs/rapid/include
                    ${CMAKE_SOURCE_DIR}/libs/msf_gif 
                    ${FLANN_INCLUDE_DIRS}
                    ${CMAKE_SOURCE_DIR}/src/graphics
                    ${CMAKE_SOURCE_DIR}/src/rrt
                    ${CMAKE_SOURCE_DIR}/src/test)
link_directories(${CMAKE_SOURCE_DIR}/libs/rapid/)

file(GLOB sources src/*.cpp src/graphics/*.cpp src/graphics/*.hpp src/rrt/*.cpp src/rrt/*.hpp src/test/*.cpp src/test/*.hpp)
add_executable(${PROJECT_NAME} ${sources})

target_link_libraries(${PROJECT_NAME} cairo RAPID flann lz4)

