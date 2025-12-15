cmake_minimum_required(VERSION 3.10)

project(DataProcessing LANGUAGES C CXX)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

# Sources
set(DATAPROCESSING_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/ProcessData.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Calibration.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Transformations.cpp
  )

add_library(dataProcessing STATIC ${DATAPROCESSING_SOURCES})

target_include_directories(dataProcessing
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(dataProcessing PRIVATE ${OpenCV_LIBS} stdc++fs PUBLIC Ceres::ceres Eigen3::Eigen)

# Ensure a sane C standard for consumers
target_compile_features(dataProcessing PUBLIC cxx_std_11)
