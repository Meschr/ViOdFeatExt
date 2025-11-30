cmake_minimum_required(VERSION 3.10)

project(Utility LANGUAGES C CXX)

find_package(OpenCV REQUIRED)
message("OpenCV Include Path: ${OpenCV_INCLUDE_DIRS}")

# Sources
set(UTILITY_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/DatasetReader.cpp
  )

add_library(Utility STATIC ${UTILITY_SOURCES})

target_include_directories(Utility
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(Utility PRIVATE ${OpenCV_LIBS} stdc++fs)

# Ensure a sane C standard for consumers
target_compile_features(Utility PUBLIC cxx_std_17)
