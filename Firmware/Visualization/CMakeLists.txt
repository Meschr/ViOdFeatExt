cmake_minimum_required(VERSION 3.10)

project(Visualization LANGUAGES C CXX)

find_package(OpenCV REQUIRED)
message("OpenCV Include Path: ${OpenCV_INCLUDE_DIRS}")

# Ensure the viz module is available
#if(NOT OpenCV_VIZ_FOUND)
#    message(FATAL_ERROR "OpenCV Viz module not found. Please build OpenCV with Viz module enabled.")
##endif()

# Sources
set(VISUALIZATION_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/VisualizationHelper.cpp
  )

add_library(Visualization STATIC ${VISUALIZATION_SOURCES})

target_include_directories(Visualization
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(Visualization PRIVATE ${OpenCV_LIBS} stdc++fs)

# Ensure a sane C standard for consumers
target_compile_features(Visualization PUBLIC cxx_std_17)

# Optionally set C++ standard
target_compile_features(Visualization PRIVATE cxx_std_11)

