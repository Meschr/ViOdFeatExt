cmake_minimum_required(VERSION 3.10)
project(visualization_tests LANGUAGES C CXX)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

# Build only test_imu explicitly:
add_executable(test_viz ${CMAKE_CURRENT_SOURCE_DIR}/test_viz.cpp)
add_executable(test_datasetread ${CMAKE_CURRENT_SOURCE_DIR}/test_datasetread.cpp)

# Ensure headers are findable
target_include_directories(test_viz PRIVATE
  ${CMAKE_SOURCE_DIR}/../Visualization
  ${CMAKE_SOURCE_DIR}/../Utility
  ${CMAKE_SOURCE_DIR}/../DataProcessing
)

# Link the firmware libraries built by parent
target_link_libraries(test_viz PRIVATE Visualization)
target_link_libraries(test_datasetread PRIVATE Visualization Utility dataProcessing)
