cmake_minimum_required(VERSION 3.0.0)
project(OCSD)

set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE LIB_SOURCES "src/*.cpp" "src/*.h")
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_library(ocsd ${LIB_SOURCES})

include_directories(include)
find_library(CAMERA_MODEL_LIB camera_model ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ocsd ${OpenCV_LIBS})

add_executable(ocsd_main src/main.cpp)
target_link_libraries(ocsd_main ocsd ${OpenCV_LIBS} ${CAMERA_MODEL_LIB})
